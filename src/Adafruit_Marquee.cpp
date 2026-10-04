/*!
 * @file Adafruit_Marquee.cpp
 *
 * @mainpage Adafruit Marquee client for Arduino.
 *
 * @section intro_sec Introduction
 *
 * Client library for the Adafruit IO Marquee feature.
 *
 * @section author Author
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 */
#include "Adafruit_Marquee.h"
#include <CRC.h>
#include <base64.hpp>

Adafruit_Marquee *Adafruit_Marquee::_instance =
    nullptr; ///< Pointer to the instance that the MQTT callbacks dispatch to

// For ESP32 sleep modes
#ifdef ARDUINO_ARCH_ESP32
#include <esp_sleep.h>
#include <esp_wifi.h>
static RTC_DATA_ATTR uint32_t
    prvBmpCrc; ///< CRC32 of the bitmap, stored before sleep
#endif         // ARDUINO_ARCH_ESP32

// Battery monitor. Only the MagTag is wired up so far; any board that defines
// MQ_HAS_BATT_MONITOR, MQ_BATT_PIN and MQ_BATT_DIVIDER gets reporting.
#if defined(ARDUINO_MAGTAG29_ESP32S2)
#define MQ_HAS_BATT_MONITOR 1
#define MQ_BATT_PIN BATT_MONITOR // GPIO4 / A5, ADC1 so it's usable with WiFi on
#define MQ_BATT_DIVIDER 2.0f     // 1:2 resistor divider on the MagTag
#endif
#define MQ_BATT_SAMPLES 16 ///< ADC samples averaged per battery reading

// for flashTransport definition
#define ADAFRUIT_MARQUEE_INTERNAL
#include "flash_config.h"

// Elm Chan's FatFs, vendored under src/fatfs/, is used only to format the
// flash: SdFat can mount FAT12 but cannot create it, and its FatFormatter
// refuses volumes of 6 MB or less. The version check catches an accidental
// pickup of the ESP-IDF copy of ff.h (R0.15, with a different f_mkfs
// signature) that also sits on the include path.
#include "fatfs/ff.h"
// diskio.h uses ff.h's typedefs, so it has to come second
#include "fatfs/diskio.h"
#if FF_DEFINED != 86604
#error "Adafruit_Marquee expects its vendored FatFs R0.13c (src/fatfs)"
#endif

// The flash chip and the USB Mass Storage endpoint are single pieces of
// hardware and nothing outside this file touches them, so they are internal
// to this translation unit. Keep them below the flash_config.h include:
// `flash` captures &flashTransport at construction, and same-TU declaration
// order is what guarantees the transport is built first.
static Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
static FatVolume fatfs;

static Adafruit_USBD_MSC usb_msc; ///< USB Mass Storage device object

bool Adafruit_Marquee::fs_formatted; ///< True when the filesystem is formatted,
                                     ///< False otherwise
volatile bool
    Adafruit_Marquee::fs_changed; ///< True when the filesystem is changed by
                                  ///< the host over USB MSC, False otherwise

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it,
  // yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). used to flush any pending cache.
static void msc_flush_cb(void) {
  // sync with flash
  flash.syncBlocks();
  // clear file system's cache to force refresh
  fatfs.cacheClear();
  Adafruit_Marquee::fs_changed = true;
}

/**
    @brief  Low-level disk i/o for fatfs.
*/
extern "C" {
/**
    @brief  Gets the current status of the disk.
*/
DSTATUS disk_status(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

/**
    @brief  Initializes the disk.
*/
DSTATUS disk_initialize(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
  (void)pdrv;
  return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

/**
    @brief  Reads sectors from the disk.
*/
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
  (void)pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

/**
    @brief  I/O control for the disk.
*/
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
  (void)pdrv;
  switch (cmd) {
  case CTRL_SYNC:
    flash.syncBlocks();
    return RES_OK;
  case GET_SECTOR_COUNT:
    *((DWORD *)buff) = flash.size() / 512;
    return RES_OK;
  case GET_SECTOR_SIZE:
    *((WORD *)buff) = 512;
    return RES_OK;
  case GET_BLOCK_SIZE:
    *((DWORD *)buff) = 8; // 4 KB erase block, in sectors
    return RES_OK;
  default:
    return RES_PARERR;
  }
}
}

/*!
    @brief  Attempts to create a new FAT volume.
    @return True if the volume was created and labeled, False otherwise.
*/
static bool formatFilesystem() {
  bool ok = false;
  const size_t workbuf_len = 4096;
  // Allocate a block for the FS object
  uint8_t *mem = (uint8_t *)malloc(sizeof(FATFS) + workbuf_len);
  if (!mem) {
    return false;
  }
  FATFS *fs = (FATFS *)mem;
  uint8_t *workbuf = mem + sizeof(FATFS);

  ok = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf, workbuf_len) == FR_OK &&
       f_mount(fs, "0:", 1) == FR_OK && f_setlabel("MARQUEE") == FR_OK;
  f_unmount("0:");
  free(mem);
  flash.syncBlocks();
  return ok;
}

/*!
    @brief  Rough state of charge for a 1S LiPo, from its resting voltage.
    @param  volts  Battery voltage, in volts.
    @return Estimated charge, 0-100 percent.
*/
static uint8_t battPercent(float volts) {
  // Resting-voltage curve, linearly interpolated. Good to ~10%, which is
  // as good as a voltage-only estimate gets.
  static const struct {
    float v;
    uint8_t pct;
  } curve[] = {{4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.93f, 70},
               {3.87f, 60},  {3.84f, 50}, {3.80f, 40}, {3.77f, 30},
               {3.73f, 20},  {3.69f, 10}, {3.61f, 5},  {3.30f, 0}};
  const size_t n = sizeof(curve) / sizeof(curve[0]);
  if (volts >= curve[0].v)
    return 100;
  for (size_t i = 1; i < n; i++) {
    if (volts >= curve[i].v) {
      float frac = (volts - curve[i].v) / (curve[i - 1].v - curve[i].v);
      return curve[i].pct + (uint8_t)(frac * (curve[i - 1].pct - curve[i].pct) +
                                      0.5f);
    }
  }
  return 0;
}

/*!
    @brief  Factory function that constructs and begins an Adafruit_EPD panel.
*/
using FnCreateAdafruit_EPD = std::function<Adafruit_EPD *(
    int16_t, int16_t, int16_t, int16_t, int16_t, SPIClass *, thinkinkmode_t)>;

/*! @brief Maps a config panel identifier to its Adafruit_EPD factory. */
using Adafruit_EPDFactory = std::map<std::string, FnCreateAdafruit_EPD>;

/*!
    @brief  Config panel identifier to Adafruit_EPD factory table.
    @return A reference to the factory map.
*/
static const Adafruit_EPDFactory &getAdafruitEPDFactory() {
  static const Adafruit_EPDFactory adafruitEPDFactory = {
      {"290-grayscale4-FPC7519",
       [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs, int16_t busy,
          SPIClass *spi, thinkinkmode_t mode) -> Adafruit_EPD * {
         auto *d = new ThinkInk_290_Grayscale4_FPC7519(dc, rst, cs, sram_cs,
                                                       busy, spi);
         d->begin(mode);
         return d;
       }},
      {"213-tricolor-MFGNR",
       [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs, int16_t busy,
          SPIClass *spi, thinkinkmode_t mode) -> Adafruit_EPD * {
         auto *d =
             new ThinkInk_213_Tricolor_MFGNR(dc, rst, cs, sram_cs, busy, spi);
         d->begin(mode);
         return d;
       }},
      {"290-grayscale4-EAAMFGN",
       [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs, int16_t busy,
          SPIClass *spi, thinkinkmode_t mode) -> Adafruit_EPD * {
         auto *d = new ThinkInk_290_Grayscale4_EAAMFGN(dc, rst, cs, sram_cs,
                                                       busy, spi);
         d->begin(mode);
         return d;
       }},
      {"420-tricolor-MFGNR",
       [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs, int16_t busy,
          SPIClass *spi, thinkinkmode_t mode) -> Adafruit_EPD * {
         auto *d =
             new ThinkInk_420_Tricolor_MFGNR(dc, rst, cs, sram_cs, busy, spi);
         d->begin(mode);
         return d;
       }},
      {"magtag-2025",
       [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs, int16_t busy,
          SPIClass *spi, thinkinkmode_t mode) -> Adafruit_EPD * {
         auto *d = new ThinkInk_290_Grayscale4_EAAMFGN(dc, rst, cs, sram_cs,
                                                       busy, spi);
         d->begin(mode);
         return d;
       }},
      {"magtag-pre-2025",
       [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs, int16_t busy,
          SPIClass *spi, thinkinkmode_t mode) -> Adafruit_EPD * {
         auto *d =
             new ThinkInk_290_Grayscale4_T5(dc, rst, cs, sram_cs, busy, spi);
         // Mono mode renders a solid black screen on this panel, use gray4
         d->begin(mode == THINKINK_MONO ? THINKINK_GRAYSCALE4 : mode);
         return d;
       }},
      {"xteink-x4-pro",
       [](int16_t dc, int16_t rst, int16_t cs, int16_t sram_cs, int16_t busy,
          SPIClass *spi, thinkinkmode_t mode) -> Adafruit_EPD * {
         auto *d =
             new Adafruit_UC8279(800, 480, dc, rst, cs, sram_cs, busy, spi);
         (void)mode;
         d->begin(true);
         return d;
       }}};
  return adafruitEPDFactory;
}

/*!
    @brief  Detects if an SSD1680 EPD is connected using bit-banged SPI.
    @param  cs    Chip Select pin number.
    @param  dc    Data/Command pin number.
    @param  rst   Reset pin number, or -1 if unused.
    @param  mosi  SPI MOSI pin number.
    @param  sck   SPI clock pin number.
    @return True if an SSD1680 is detected, False otherwise (IL0373 or different
            EPD).
*/
static bool detectSSD1680(int16_t cs, int16_t dc, int16_t rst, int16_t mosi,
                          int16_t sck) {
  // Configure SPI pins to bit-bang
  pinMode(mosi, OUTPUT);
  pinMode(sck, OUTPUT);
  pinMode(cs, OUTPUT);
  pinMode(dc, OUTPUT);

  // Reset the display
  digitalWrite(cs, HIGH);
  if (rst >= 0) {
    pinMode(rst, OUTPUT);
    digitalWrite(rst, HIGH);
    delay(10);
    digitalWrite(rst, LOW);
    delay(10);
    digitalWrite(rst, HIGH);
    delay(200);
  }

  // Begin transaction by pulling cs and dc LOW
  digitalWrite(cs, LOW);
  digitalWrite(dc, LOW);
  digitalWrite(mosi, LOW);
  digitalWrite(sck, LOW);

  // Write to read register 0x71
  uint8_t cmd = 0x71;
  for (int i = 0; i < 8; i++) {
    digitalWrite(mosi, (cmd & (1 << (7 - i))) != 0);
    digitalWrite(sck, HIGH);
    digitalWrite(sck, LOW);
  }

  // Set DC high to indicate data and switch MOSI to input with PUR in case
  // SSD1680 does not send data back
  pinMode(mosi, INPUT_PULLUP);
  digitalWrite(dc, HIGH);

  // Read response from register
  uint8_t status = 0;
  for (int i = 0; i < 8; i++) {
    status <<= 1;
    if (digitalRead(mosi)) {
      status |= 1;
    }
    digitalWrite(sck, HIGH);
    digitalWrite(sck, LOW);
  }
  digitalWrite(cs, HIGH);
  pinMode(mosi, OUTPUT);

  return status == 0xFF;
}

/*!
 * @brief Creates a new instance of the Adafruit Marquee client.
 */
Adafruit_Marquee::Adafruit_Marquee() {
  _display = nullptr;
  _device_name = nullptr;

  _mqtt = nullptr;
  _last_ping = 0;
  _last_wifi_attempt = 0;
  _last_mqtt_attempt = 0;
  _mqtt_retry_ms = MQ_MQTT_RETRY_MS;
  _sub_bmp = nullptr;
  _sub_sleep = nullptr;
  _awaiting_bmp = false;
  _awaiting_sleep = false;
  _pending_bmp = nullptr;
  _pending_bmp_len = 0;
  _pending_crc = 0;
  _ssid = nullptr;
  _pass = nullptr;
  _aio_username = nullptr;
  _aio_key = nullptr;

  _thinkInkMode = THINKINK_MONO;
  _status = SUCCESS;
  _pin_cs = -1;
  _pin_dc = -1;
  _pin_rst = -1;
  _pin_busy = -1;
  _pin_sram_cs = -1;
  _pin_sclk = -1;
  _pin_mosi = -1;
  _pin_miso = -1;
  _rotation = 0;
  _width = 0;
  _height = 0;

  fs_formatted = false;
  fs_changed = true;

  _sleep_mode = SLEEP_MODE_NONE;
  _sleep_alarm = SLEEP_ALARM_NONE;
  _is_sleep_pending = false;
  _sleep_time = 60; // default to 60 seconds to avoid rapid wake/sleep cycling

  _batt_enabled = false;
  _batt_as_percent = true;
  _batt_pending = false;
  _batt_volts = -1.0f;
  _batt_feed = nullptr;
  _topic_batt[0] = '\0';
}

/*!
 * @brief Destructor.
 */
Adafruit_Marquee::~Adafruit_Marquee() {
  if (_display) {
    delete _display;
    _display = nullptr;
  }
  if (_pending_bmp) {
    free(_pending_bmp);
    _pending_bmp = nullptr;
  }
  _pending_bmp_len = 0;
  delete _sub_bmp;
  _sub_bmp = nullptr;
  delete _sub_sleep;
  _sub_sleep = nullptr;
  delete _mqtt;
  _mqtt = nullptr;
  if (_instance == this) {
    _instance = nullptr;
  }
}

#if defined(MARQUEE_BOARD_XTEINK_X4_PRO)
/*!
 * @brief Brings up the XTeink X4 Pro's peripheral rails, required for EPD
 * bringup.
 */
void Adafruit_Marquee::marqueeBoardPowerUp() {
  // Turn on the main peripheral power
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  delay(50);

  // Turn off the SD card power (active-low) so we can use the EPD
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
}
#endif // MARQUEE_BOARD_XTEINK_X4_PRO

/*!
 * @brief Initializes the filesystem, USB MSC, and display. Errors from here
 *        happen before the display is up, so they can only be logged.
 * @returns SUCCESS if initialization succeeded, otherwise the
 *          mq_status_t describing the failure.
 */
mq_status_t Adafruit_Marquee::begin() {
#if defined(MARQUEE_BOARD_XTEINK_X4_PRO)
  marqueeBoardPowerUp();
#endif

  // Detach USB *before* touching the flash, mirroring Wippersnapper_FS
  TinyUSBDevice.detach();
  delay(500);

  // Init. the flash and mount the file system on it
  _status = initFilesystem();
  if (_status == ERR_FLASH_INIT) {
    // Failed, attach USB for debugging over serial
    TinyUSBDevice.attach();
    delay(500);
    return _status;
  }

  // Still expose an unformatted drive so the host can format it
  initUSBMSC();
  if (_status != SUCCESS)
    return _status;

  // Bring up the display
  File32 cfg = fatfs.open("/cfg-marquee.json", O_RDONLY);
  if (!cfg)
    return _status = ERR_FS_NO_CFG_FILE;

  _status = parseDisplayCfg(cfg);
  if (_status != SUCCESS)
    return _status;

  // Sample the battery now, before the EPD refresh or the radio load it down
  parseBatteryCfg();
  sampleBattery();

  const char *display_panel = _cfg_doc["display"]["panel"];
  if (!createEPD(display_panel))
    return _status = ERR_EPD_PANEL_UNSUPPORTED;

  // Configure the display using the parsed settings
  _display->setRotation(_rotation);
  _display->setTextSize(3);
  _display->setTextColor(EPD_BLACK);
  _display->setTextWrap(false);
  _height = _display->height();
  _width = _display->width();

  return _status = SUCCESS;
}

/*!
 * @brief Parses the network, Adafruit IO, and device name settings from the
 *        Marquee config file. Call after begin() and before connect().
 * @returns SUCCESS if all were found, otherwise the mq_status_t describing
 *          the failure.
 */
mq_status_t Adafruit_Marquee::parseCreds() {
  _status = parseNetCreds();
  if (_status == SUCCESS)
    _status = parseIOCreds();
  if (_status == SUCCESS && !parseDeviceName())
    _status = ERR_INVALID_DEVICE_NAME;
  return _status;
}

/*!
 * @brief Returns the status of the last begin(), parseCreds(), or connect()
 *        call.
 * @returns The mq_status_t of the last call.
 */
mq_status_t Adafruit_Marquee::getStatus() { return _status; }

/*!
 * @brief Displays the last _status error on the screen and halts (keeping USB
 *        MSC open). Only call when getStatus() is not SUCCESS.
 */
void Adafruit_Marquee::displayStatus() {
  switch (_status) {
  case ERR_MISSING_WIFI_CREDS:
    displayErrorMsg("WiFi Error",
                    "Missing WiFi credentials in cfg-marquee.json",
                    "SSID: ", _ssid);
    break;
  case ERR_INVALID_WIFI_CREDS:
    displayErrorMsg("WiFi Error",
                    "Invalid WiFi credentials in cfg-marquee.json",
                    "SSID: ", _ssid);
    break;
  case ERR_MISSING_IO_CREDS:
    displayErrorMsg("IO Error", "Missing IO credentials in cfg-marquee.json",
                    "Username: ", _aio_username);
    break;
  case ERR_INVALID_IO_CREDS:
    displayErrorMsg("IO Error", "Invalid IO credentials in cfg-marquee.json",
                    "Username: ", _aio_username);
    break;
  case ERR_INVALID_DEVICE_NAME:
    displayErrorMsg("Config Error", "Missing device name in cfg-marquee.json");
    break;
  case ERR_MQTT_INIT:
    displayErrorMsg("IO Error", "Could not set up the MQTT client");
    break;
  case ERR_WIFI_CONNECT:
    displayErrorMsg("WiFi Error", "Could not connect to the network",
                    "SSID: ", _ssid);
    break;
  case ERR_IO_CONNECT:
    displayErrorMsg("IO Error", "Could not connect to AIO",
                    "Username: ", _aio_username);
    break;
  default:
    break;
  }

  for (;;) {
    yield(); // Allows background USB/Wi-Fi tasks to run briefly
    delay(1);
  }
}

/*!
    @brief  Initializes, optionally formats and mounts a fat fs.
    @return SUCCESS, ERR_FLASH_INIT if there is no usable flash partition, or
            ERR_FS_UNFORMATTED if the volume could not be formatted or mounted.
*/
mq_status_t Adafruit_Marquee::initFilesystem() {
  if (!flash.begin() || flash.size() == 0) {
    return ERR_FLASH_INIT;
  }

  if (!fatfs.begin(&flash) && !(formatFilesystem() && fatfs.begin(&flash))) {
    return ERR_FS_UNFORMATTED;
  }

  fs_formatted = true;
  return SUCCESS;
}

/*!
    @brief  Brings up the USB MSC endpoint, attaches the USB device
*/
void Adafruit_Marquee::initUSBMSC() {
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setCapacity(flash.size() / 512, 512);
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
  }
  TinyUSBDevice.attach();

  // Give time for host to enumerate
  delay(500);
}

/*!
 * @brief Builds the MQTT client, via the subclass, and the feed
 *        subscriptions over it.
 * @returns True if initialization succeeded, False otherwise.
 */
bool Adafruit_Marquee::initMqtt() {
  _instance = this;

  // Attempt to create the MQTT client
  setupMQTTClient();
  if (!_mqtt) {
    MQ_DEBUG_PRINTLN("[mqtt] ERROR: could not create the MQTT client");
    return false;
  }
  if (_mqtt->bufferSize() == 0) {
    MQ_DEBUG_PRINTF("[mqtt] ERROR: could not allocate %u byte packet buffer\n",
                    (unsigned)MQ_MQTT_BUFFER_LEN);
    return false;
  }

  if (!_mqtt->setKeepAliveInterval(MQ_MQTT_KEEPALIVE_MS * 0.001)) {
    MQ_DEBUG_PRINTLN("[mqtt] ERROR: could not set the keepalive interval");
    return false;
  }

  // Initialize Adafruit IO feed names
  snprintf(_feed_name_bmp, sizeof(_feed_name_bmp), "%s.bitmap", _device_name);
  snprintf(_topic_bmp, sizeof(_topic_bmp), "%s/f/%s/csv", _aio_username,
           _feed_name_bmp);
  _sub_bmp =
      new Adafruit_MQTT_Subscribe(_mqtt, _topic_bmp, 0, MQ_BITMAP_SUB_LEN);
  if (!_sub_bmp || !_sub_bmp->lastread) {
    MQ_DEBUG_PRINTLN("[bmp] ERROR: Couldn't create the bitmap feed");
    return false;
  }
  _sub_bmp->setCallback(cbBitmapMsg);

  snprintf(_feed_name_sleep, sizeof(_feed_name_sleep), "%s.sleep",
           _device_name);
  snprintf(_topic_sleep, sizeof(_topic_sleep), "%s/f/%s", _aio_username,
           _feed_name_sleep);
  _sub_sleep = new Adafruit_MQTT_Subscribe(_mqtt, _topic_sleep);
  if (!_sub_sleep) {
    MQ_DEBUG_PRINTLN("[sleep] ERROR: could not create the sleep feed");
    return false;
  }
  _sub_sleep->setCallback(cbSleepMsg);

  // Build status feed (publish only)
  snprintf(_feed_name_status, sizeof(_feed_name_status), "%s.status",
           _device_name);
  snprintf(_topic_status, sizeof(_topic_status), "%s/f/%s", _aio_username,
           _feed_name_status);

  // Build battery feed (publish only), if enabled
  if (_batt_enabled) {
    if (_batt_feed && _batt_feed[0] != '\0') {
      snprintf(_topic_batt, sizeof(_topic_batt), "%s/f/%s", _aio_username,
               _batt_feed);
    } else {
      snprintf(_topic_batt, sizeof(_topic_batt), "%s/f/%s.battery",
               _aio_username, _device_name);
    }
  }

  // Attempt to register subscriptions
  if (!_mqtt->subscribe(_sub_bmp) || !_mqtt->subscribe(_sub_sleep)) {
    MQ_DEBUG_PRINTLN("Failed to register MQTT subscriptions");
    return false;
  }

  MQ_DEBUG_PRINT("Subscribed to BMP feed: ");
  MQ_DEBUG_PRINTLN(_topic_bmp);
  MQ_DEBUG_PRINT("Subscribed to sleep feed: ");
  MQ_DEBUG_PRINTLN(_topic_sleep);
  MQ_DEBUG_PRINT("Publishing status to: ");
  MQ_DEBUG_PRINTLN(_topic_status);
  if (_batt_enabled) {
    MQ_DEBUG_PRINT("Publishing battery to: ");
    MQ_DEBUG_PRINTLN(_topic_batt);
  }
  return true;
}

/*!
    @brief  Attempts to connect to the WiFi network
    @param  timeout  Maximum time to wait for a connection, in milliseconds.
    @return True once the platform reports an association, False otherwise
*/
bool Adafruit_Marquee::initWifi(unsigned long timeout) {
  if (!_ssid || strlen(_ssid) == 0) {
    MQ_DEBUG_PRINTLN("[wifi] ERROR: SSID not found in config file!");
    return false;
  }

  // Attempt to connect to WiFi
  MQ_DEBUG_PRINTF("[wifi] connecting to '%s'\n", _ssid);
  _connect();
  unsigned long start = millis();
  while (!isNetConnected() && millis() - start < timeout) {
    delay(MQ_WIFI_POLL_MS);
  }
  _last_wifi_attempt = millis();

  if (!isNetConnected()) {
    MQ_DEBUG_PRINTLN(
        "[wifi] ERROR: timed out, could not connect to WiFi network");
    return false;
  }

  return true;
}

/*!
    @brief  Opens the MQTT session. Credentials went in at construction, so
            this is the no-argument connect().
    @return True on CONNACK success, else False.
*/
bool Adafruit_Marquee::connectMqtt() {
  _last_mqtt_attempt = millis();
  int8_t rc = _mqtt->connect();
  _mqtt_retry_ms = MQ_MQTT_RETRY_MS;
  if (rc != 0) {
    MQ_DEBUG_PRINT("[mqtt] connect failed: ");
    MQ_DEBUG_PRINTLN(_mqtt->connectErrorString(rc));
    return false;
  }

  // Publish to the status feed that the device is awake + why it is awake
  JsonDocument doc;
  doc["state"] = "awake";
  doc["wake_reason"] = wakeupReason();
  char payload[128];
  size_t len = serializeJson(doc, payload, sizeof(payload));
  if (len == 0 || len >= sizeof(payload)) {
    MQ_DEBUG_PRINTLN("[status] ERROR: could not serialize the awake payload");
  } else {
    publishStatus(payload);
  }

  // Report the battery sample taken at boot (or on light-sleep wake)
  publishBattery();

  // Ask for IO to republish the last data point on the bitmap feed.
  // handleSleep() holds a queued sleep until this reply lands, otherwise the
  // small sleep payload beats the ~21KB bitmap to the device and the panel
  // never updates.
  _awaiting_bmp = true;
  getFromFeed(_topic_bmp);

  if (didWakeFromSleep()) {
    MQ_DEBUG_PRINTLN("[sleep] Device woke from sleep!");
  }

  // Allow a cold boot with a sleep config from a feed to enter sleep after
  // drawing
  _awaiting_sleep = true;
  getFromFeed(_topic_sleep);

  return true;
}

/*!
 * @brief Connects to WiFi and the Adafruit IO MQTT broker.
 * @param timeout The maximum time to wait for a connection, in milliseconds.
 * @returns True if connection succeeded, otherwise false. See getStatus() for
 *          the reason.
 */
bool Adafruit_Marquee::connect(unsigned long timeout) {
  if (!initMqtt()) {
    MQ_DEBUG_PRINTLN("Failed to initialize the MQTT client and feeds");
    _status = ERR_MQTT_INIT;
    return false;
  }

  if (!initWifi(timeout)) {
    _status = ERR_WIFI_CONNECT;
    return false;
  }

  // connectMqtt() also asks the feeds to republish, so there is nothing left
  // for this function to do once the session is up.
  if (!connectMqtt()) {
    _status = ERR_IO_CONNECT;
    return false;
  }

  // If we are waking from cold boot, push an empty buffer to clear the display
  // since there aren't any previous bitmaps to show or errors
  if (!didWakeFromSleep()) {
    _display->display();
  }

  _status = SUCCESS;
  return true;
}

/*!
    @brief  Asks a feed to republish its last data point.
    @param  topic  Subscribed topic to republish from.
*/
void Adafruit_Marquee::getFromFeed(const char *topic) {
  if (!_mqtt || !topic || topic[0] == '\0') {
    return;
  }

  // Ask the feed to republish its last stored data point
  char get_topic[sizeof(_topic_bmp) + 8];
  snprintf(get_topic, sizeof(get_topic), "%s/get", topic);
  Adafruit_MQTT_Publish pub_get(_mqtt, get_topic);
  pub_get.publish("\0");
}

/*!
    @brief  Publishes a payload to the device's status feed.
    @param  payload  The message to publish.
    @return True if the publish succeeded, False otherwise.
*/
bool Adafruit_Marquee::publishStatus(const char *payload) {
  if (!_mqtt || _topic_status[0] == '\0') {
    MQ_DEBUG_PRINTLN("[status] ERROR: Cannot publish, MQTT not ready");
    return false;
  }
  if (!_mqtt->connected()) {
    MQ_DEBUG_PRINTLN("[status] ERROR: Cannot publish, MQTT not connected");
    return false;
  }

  Adafruit_MQTT_Publish pub_status(_mqtt, _topic_status, MQTT_QOS_1);
  if (!pub_status.publish(payload)) {
    MQ_DEBUG_PRINTLN("[status] ERROR: Publish failed");
    return false;
  }

  MQ_DEBUG_PRINT("[status] PUBLISHED -> ");
  MQ_DEBUG_PRINTLN(payload);
  return true;
}

/*!
    @brief  Parses the optional "battery" object from the Marquee config.
            Reporting stays off unless "enabled" is true and the board has a
            battery monitor.
*/
void Adafruit_Marquee::parseBatteryCfg() {
  JsonObject batt = _cfg_doc["battery"];
  if (!(batt["enabled"] | false)) {
    return;
  }
#ifdef MQ_HAS_BATT_MONITOR
  _batt_enabled = true;
  _batt_feed = batt["feed"]; // nullptr -> "<name>.battery"
  const char *units = batt["units"] | "percent";
  _batt_as_percent = strcmp(units, "volts") != 0;
#else
  MQ_DEBUG_PRINTLN("[batt] No battery monitor on this board, ignoring");
#endif
}

/*!
    @brief  Samples the battery voltage and queues it for publishing. Call
            while the radio is off; WiFi TX sags the rail.
*/
void Adafruit_Marquee::sampleBattery() {
#ifdef MQ_HAS_BATT_MONITOR
  if (!_batt_enabled) {
    return;
  }
  uint32_t mv = 0;
  for (int i = 0; i < MQ_BATT_SAMPLES; i++) {
    mv += analogReadMilliVolts(MQ_BATT_PIN);
  }
  _batt_volts = (mv / (float)MQ_BATT_SAMPLES) * MQ_BATT_DIVIDER / 1000.0f;
  _batt_pending = true;
  MQ_DEBUG_PRINTF("[batt] %.3f V, ~%u%%\n", _batt_volts,
                  (unsigned)battPercent(_batt_volts));
#endif
}

/*!
    @brief  Publishes the pending battery sample, once. A failed publish is
            retried on the next MQTT connect.
*/
void Adafruit_Marquee::publishBattery() {
  if (!_batt_enabled || !_batt_pending || _topic_batt[0] == '\0' || !_mqtt ||
      !_mqtt->connected()) {
    return;
  }

  char payload[16];
  if (_batt_as_percent) {
    snprintf(payload, sizeof(payload), "%u",
             (unsigned)battPercent(_batt_volts));
  } else {
    snprintf(payload, sizeof(payload), "%.2f", _batt_volts);
  }

  Adafruit_MQTT_Publish pub_batt(_mqtt, _topic_batt, MQTT_QOS_1);
  if (!pub_batt.publish(payload)) {
    MQ_DEBUG_PRINTLN("[batt] ERROR: Publish failed");
    return;
  }
  _batt_pending = false;
  MQ_DEBUG_PRINT("[batt] PUBLISHED -> ");
  MQ_DEBUG_PRINTLN(payload);
}

/*!
    @brief  Reads the CRC32 of the bitmap drawn to the panel (prior to sleep)
    @param  crc  If this library returns True, set to the CRC32 of the b64
   payload.
    @return True if a CRC was successfully stored, False otherwise.
*/
bool Adafruit_Marquee::loadPrvBmpCRC(uint32_t &crc) {
#ifdef ARDUINO_ARCH_ESP32
  if (prvBmpCrc == 0)
    return false;
  crc = prvBmpCrc;
  return true;
#else
  return false;
#endif // ARDUINO_ARCH_ESP32
}

/*!
    @brief  Records the CRC32 of the bitmap drawn to the panel.
    @param  crc  CRC32 of the base64 payload that produced the bitmap.
*/
void Adafruit_Marquee::storeBmpCRC(uint32_t crc) {
#ifdef ARDUINO_ARCH_ESP32
  prvBmpCrc = crc;
#else
  (void)crc;
#endif // ARDUINO_ARCH_ESP32
}

/*!
    @brief  Clears the panel and draws an error message on it, scaled to the
            panel size.
    @param  title    The error title, drawn large.
    @param  details  The error details, drawn smaller and wrapped.
    @param  label    Optional label for a config value (e.g. "SSID: "), drawn
                     below the details at half their size.
    @param  value    The config value drawn after the label, or nullptr to
                     draw "(missing)".
*/
void Adafruit_Marquee::displayErrorMsg(const char *title, const char *details,
                                       const char *label, const char *value) {
  if (!_display || !title || !details)
    return;
  size_t title_len = strlen(title);

  // Resize the title text for the panel
  int title_size = _width / (6 * (title_len ? title_len : 1));
  title_size = max(1, min(title_size, _height / 24));

  // Write the title and detail text to the display
  _display->clearBuffer();
  _display->setTextSize(title_size);
  _display->setCursor(0, 0);
  _display->print(title);
  int details_size = max(1, title_size / 2);
  _display->setTextSize(details_size);
  _display->setTextWrap(true);
  _display->setCursor(0, title_size * 10);
  _display->print(details);
  if (label) {
    // Start on the line after the details, with a small gap
    _display->setTextSize(max(1, details_size / 2));
    _display->setCursor(0, _display->getCursorY() + details_size * 12);
    _display->print(label);
    _display->print(value ? value : "(missing)");
  }
  _display->display();
}

/*!
    @brief  Draws the bitmap queued by the bitmap feed callback, if any.
*/
void Adafruit_Marquee::drawBitmap() {
  if (!_pending_bmp || !_display)
    return;
  MQ_DEBUG_PRINT("[display] Drawing to the panel...");
  // Clear the display buffer before drawing the new bitmap. This only touches
  // the RAM framebuffer; nothing reaches the panel until display() is called.
  _display->clearBuffer();
  uint32_t t_decode_start = millis();
  ImageReturnCode rc =
      _reader.drawBMP(_pending_bmp, _pending_bmp_len, *_display, 0, 0);
  uint32_t t_decode = millis() - t_decode_start;
  if (rc != IMAGE_SUCCESS) {
    MQ_DEBUG_PRINTF("[display] ERROR: drawBMP rc: %d\n", (int)rc);
  } else {
    uint32_t t_refresh_start = millis();
    _display->display(true);
    (void)t_refresh_start;
    MQ_DEBUG_PRINTF("[display] decode ms: %u refresh ms: %u\n",
                    (unsigned)t_decode, (unsigned)(millis() - t_refresh_start));
    storeBmpCRC(_pending_crc);
  }

  // Free the bitmap buffer and clear flags
  free(_pending_bmp);
  _pending_bmp = nullptr;
  _pending_bmp_len = 0;
  MQ_DEBUG_PRINTLN("[display] Draw complete!");
}

/*!
    @brief  Keeps WiFi and the MQTT session up and pumps incoming packets.
*/
void Adafruit_Marquee::handleConnection() {
  if (!_mqtt)
    return;
  // Is the WiFi network still connected?
  if (!isNetConnected()) {
    if (millis() - _last_wifi_attempt >= MQ_WIFI_RETRY_MS) {
      MQ_DEBUG_PRINTLN("[wifi] disconnected, re-associating");
      _last_wifi_attempt = millis();
      _connect();
    }
    // Returns to the loop - retires later
    return;
  }

  // Is the MQTT session still connected?
  if (!_mqtt->connected()) {
    if (millis() - _last_mqtt_attempt < _mqtt_retry_ms) {
      return;
    }
    MQ_DEBUG_PRINTLN("[mqtt] disconnected, reconnecting");
    if (!connectMqtt()) {
      return;
    }
    return;
  }

  // Attempt to process incoming packets
  _mqtt->processPackets(100);

  // Attempt to ping the broker within the keepalive interval.
  if (millis() - _last_ping >= (MQ_MQTT_KEEPALIVE_MS / 4)) {
    _last_ping = millis();
    _mqtt->ping();
  }
}

/*!
    @brief  Blocking application loop for handling network state, drawing to the
   display and sleep modes.
*/
void Adafruit_Marquee::run() {
  // Keeps the WiFi and MQTT session up, dispatches any incoming feed messages
  handleConnection();

  // Draw any bitmap queued by the bitmap feed callback
  drawBitmap();
  // Sleep, if the sleep feed asked for it. On deep sleep this does not return.
  handleSleep();
}

/*!
    @brief  Callback for bitmap feed messages. Forwards the payload to the
            instance registered by initMqtt().
    @param  data  The message payload: a base64-encoded BMP. Adafruit_MQTT
                  nul-terminates this, so it can be treated as a C string.
    @param  len   Payload length, in bytes.
*/
void Adafruit_Marquee::cbBitmapMsg(char *data, size_t len) {
  if (!_instance)
    return;
  _instance->_awaiting_bmp = false;
  if (!data || len == 0)
    return;

  // Process the bitmap message
  _instance->decodeb64Bmp(data, len);
}

/*!
    @brief  Callback for sleep feed messages, parses and stores the sleep feed's
   JSON data into the class instance.
    @param  data  The message payload.
    @param  len   Payload length, in bytes.
*/
void Adafruit_Marquee::cbSleepMsg(char *data, size_t len) {
  if (!_instance)
    return;
  _instance->_awaiting_sleep = false;
  if (!data || len == 0)
    return;
  MQ_DEBUG_PRINT("Sleep feed received <- ");
  MQ_DEBUG_PRINTLN(data);

  // Parse and store the sleep feed JSON in the class' instance
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, data, len);
  if (error) {
    MQ_DEBUG_PRINT("Sleep feed JSON parse failed");
    MQ_DEBUG_PRINTLN(error.c_str());
    return;
  }

  if (doc["alarm_type"] == "timer") {
    _instance->_sleep_alarm = SLEEP_ALARM_TIMER;
  } else {
    _instance->_sleep_alarm = SLEEP_ALARM_NONE;
  }

  if (doc["sleep_mode"] == "deep") {
    _instance->_sleep_mode = SLEEP_MODE_DEEP;
  } else if (doc["sleep_mode"] == "light") {
    _instance->_sleep_mode = SLEEP_MODE_LIGHT;
  } else {
    _instance->_sleep_mode = SLEEP_MODE_NONE;
  }
  _instance->_sleep_time = doc["sleep_time"] | 60;
  _instance->_is_sleep_pending = true;
  MQ_DEBUG_PRINTLN("[sleep] Sleep feed stored, sleep pending");
}

/*!
    @brief  Deserializes the Marquee config file and parses out the display
            and display interface configuration.
    @param  cfg  The opened Marquee config file. Closed before returning.
    @return SUCCESS if the display configuration was parsed, otherwise the
            mq_status_t describing the failure.
*/
mq_status_t Adafruit_Marquee::parseDisplayCfg(File32 &cfg) {
  DeserializationError error = deserializeJson(_cfg_doc, cfg);
  cfg.close();
  if (error) {
    return ERR_JSON_DESERIALIZATION;
  }

  JsonObject display = _cfg_doc["display"];

  // NOTE: `rotation` may be provided as (0 thru 3) or in degrees (0 thru 270)
  _rotation = display["rotation"] | 0;
  if (_rotation > 3) {
    // Rotation provided in degrees, convert to expected 0-3 range for
    // setRotation() call
    int degrees = display["rotation"];
    switch (degrees) {
    case 90:
      _rotation = 1;
      break;
    case 180:
      _rotation = 2;
      break;
    case 270:
      _rotation = 3;
      break;
    default:
      _rotation = 0;
      break;
    }
  }

  const char *display_mode = display["mode"];
  if (!parseThinkInkMode(display_mode)) {
    return ERR_TI_MODE_UNSUPPORTED;
  }

  // Attempt to parse SPI interface
  JsonObject interface = _cfg_doc["interface"];
  const char *interface_type = interface["type"];
  if (!interface_type || (strcmp(interface_type, "spi_epd") != 0 &&
                          strcmp(interface_type, "builtin") != 0)) {
    return ERR_IFACE_UNSUPPORTED;
  }

  JsonObject pins = interface["pins"];
  _pin_cs = pins["cs"] | -1;
  _pin_dc = pins["dc"] | -1;
  _pin_rst = pins["reset"] | -1;
  _pin_busy = pins["busy"] | -1;
  _pin_sram_cs = pins["sram_cs"] | -1;
  _pin_sclk = pins["sclk"] | pins["sck"] | -1;
  _pin_mosi = pins["mosi"] | -1;
  _pin_miso = pins["miso"] | -1;

  return SUCCESS;
}

/*!
    @brief  Finds and initializes the display panel from the configured
            panel identifier.
    @param  panel  The EPD panel name (e.g., "290-grayscale4-FPC7519").
    @return True if the panel was found and initialized, False otherwise.
*/
bool Adafruit_Marquee::createEPD(const char *panel) {
  if (!panel) {
    return false;
  }

  // Probe for the MagTag panel type before initializing the SPI bus
  if (strcmp(panel, "magtag") == 0) {
    bool is_ssd1680 = detectSSD1680(_pin_cs, _pin_dc, _pin_rst,
                                    _pin_mosi >= 0 ? _pin_mosi : MOSI,
                                    _pin_sclk >= 0 ? _pin_sclk : SCK);
    panel = is_ssd1680 ? "magtag-2025" : "magtag-pre-2025";
    MQ_DEBUG_PRINTF("[epd] Detected MagTag panel: %s\n", panel);
  }

  // Look up the panel identifier in the factory table and create the instance
  const Adafruit_EPDFactory &adafruitEPDFactory = getAdafruitEPDFactory();
  Adafruit_EPDFactory::const_iterator it = adafruitEPDFactory.find(panel);
  if (it == adafruitEPDFactory.end()) {
    return false;
  }

  // If the board doesn't use the default SPI pins, re-map the bus to the pins
  // in the config file
  if (_pin_sclk >= 0 || _pin_mosi >= 0)
    SPI.begin(_pin_sclk, _pin_miso, _pin_mosi, /*ss=*/-1);

  // Creates the panel instance using the factory function
  _display = it->second(_pin_dc, _pin_rst, _pin_cs, _pin_sram_cs, _pin_busy,
                        &SPI, _thinkInkMode);
  if (_display == nullptr) {
    return false;
  }

  return true;
}

/*!
 * @brief Parses the network credentials from the Marquee config file.
 * @returns SUCCESS, ERR_MISSING_WIFI_CREDS if the SSID or password is not in
 *          the config file, or ERR_INVALID_WIFI_CREDS if either is the wrong
 *          length.
 */
mq_status_t Adafruit_Marquee::parseNetCreds() {
  _ssid = _cfg_doc["network"]["wifi_ssid"];
  _pass = _cfg_doc["network"]["wifi_password"];
  if (!_ssid || !_pass)
    return ERR_MISSING_WIFI_CREDS;
  // WPA passwords may be empty for open networks
  size_t ssid_len = strlen(_ssid);
  size_t pass_len = strlen(_pass);
  if (ssid_len == 0 || ssid_len > MQ_SSID_MAX_LEN ||
      (pass_len > 0 && pass_len < MQ_WPA_PASS_MIN_LEN) ||
      pass_len > MQ_WPA_PASS_MAX_LEN)
    return ERR_INVALID_WIFI_CREDS;
  return SUCCESS;
}

/*!
 * @brief Parses the Adafruit IO credentials from the Marquee config file.
 * @returns SUCCESS, ERR_MISSING_IO_CREDS if the username or key is not in the
 *          config file, or ERR_INVALID_IO_CREDS if the username is empty or
 *          the key is the wrong length.
 */
mq_status_t Adafruit_Marquee::parseIOCreds() {
  _aio_username = _cfg_doc["adafruit_io"]["username"];
  _aio_key = _cfg_doc["adafruit_io"]["key"];
  if (!_aio_username || !_aio_key)
    return ERR_MISSING_IO_CREDS;
  if (strlen(_aio_username) == 0 || strlen(_aio_key) != MQ_IO_KEY_LEN)
    return ERR_INVALID_IO_CREDS;
  return SUCCESS;
}

/*!
 * @brief Parses the device name from the Marquee config file.
 * @returns true if the device name was found, otherwise false.
 */
bool Adafruit_Marquee::parseDeviceName() {
  _device_name = _cfg_doc["name"];
  return _device_name;
}

/*!
 * @brief Parses the ThinkInk mode from the Marquee config file.
 * @param mode The ThinkInk mode string to parse.
 * @returns true if the mode was successfully parsed, otherwise false.
 */
bool Adafruit_Marquee::parseThinkInkMode(const char *mode) {
  if (!mode)
    return false;

  if (strcmp(mode, "mono") == 0) {
    _thinkInkMode = THINKINK_MONO;
  } else if (strcmp(mode, "tricolor") == 0) {
    _thinkInkMode = THINKINK_TRICOLOR;
  } else if (strcmp(mode, "grayscale4") == 0) {
    _thinkInkMode = THINKINK_GRAYSCALE4;
  } else if (strcmp(mode, "quadcolor") == 0) {
    _thinkInkMode = THINKINK_QUADCOLOR;
  } else {
    return false;
  }
  return true;
}

/*!
    @brief  Decodes and saves a base64-encoded BMP, next run() draws it to the
   display.
    @param  b64      Desired b64-encoded payload to decode.
    @param  b64_len  Expected length of the payload.
    @return True if a bitmap was decoded and queued, False otherwise.
*/
bool Adafruit_Marquee::decodeb64Bmp(const char *b64, size_t b64_len) {
  if (!b64 || b64_len == 0) {
    MQ_DEBUG_PRINTLN("[bmp] ERROR: null payload or zero length");
    return false;
  }

  // Calculate the CRC of the payload
  uint32_t crc = calcCRC32((const uint8_t *)b64, b64_len);
  // Compare the payload's CRC to the last drawn bitmap's CRC
  uint32_t prv_crc;
  if (loadPrvBmpCRC(prv_crc) && prv_crc == crc) {
    MQ_DEBUG_PRINTF("[bmp] unchanged (crc 0x%08lX), skipping redraw\n",
                    (unsigned long)crc);
    return true;
  }

  // First, decode the payload's length without allocating a buffer
  size_t decoded_len =
      decode_base64_length((const unsigned char *)b64, (unsigned int)b64_len);
  if (decoded_len < MIN_SZ_BMP_HEADER || decoded_len > MQ_MQTT_BUFFER_LEN) {
    MQ_DEBUG_PRINTF("[bmp] ERROR: implausible decoded length: %u\n",
                    (unsigned)decoded_len);
    return false;
  }

  // Then, attempt to allocate a buffer for the decoded BMP
  uint8_t *buf = (uint8_t *)ps_malloc(decoded_len);
  if (!buf) {
    buf = (uint8_t *)malloc(decoded_len);
  }
  // Did allocation fail?
  if (!buf) {
    MQ_DEBUG_PRINTF("[bmp] ERROR: alloc of %u bytes failed\n",
                    (unsigned)decoded_len);
    return false;
  }

  // Attempt to decode the base64 payload into thebuffer
  unsigned int write_len =
      decode_base64((const unsigned char *)b64, (unsigned int)b64_len, buf);
  if (write_len != decoded_len) {
    MQ_DEBUG_PRINTF("[bmp] ERROR: decoded %u of %u bytes\n", write_len,
                    (unsigned)decoded_len);
    free(buf);
    return false;
  }

  // Store and queue the decoded BMP for the next run() to draw
  if (_pending_bmp) {
    free(_pending_bmp);
  }
  _pending_bmp = buf;
  _pending_bmp_len = decoded_len;
  _pending_crc = crc;

  MQ_DEBUG_PRINTF("[bmp] decoded %u bytes, queued for draw\n",
                  (unsigned)decoded_len);
  return true;
}

// Sleep API
#ifdef ARDUINO_ARCH_ESP32

/*!
    @brief  Returns why the ESP32 woke from its previous sleep.
    @return The wakeup reason
*/
const char *Adafruit_Marquee::wakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
  case ESP_SLEEP_WAKEUP_EXT0:
    return "ext0"; // external signal (RTC_IO)
  case ESP_SLEEP_WAKEUP_EXT1:
    return "ext1"; // external signal (RTC_CNTL)
  case ESP_SLEEP_WAKEUP_TIMER:
    return "timer";
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    return "touchpad";
  case ESP_SLEEP_WAKEUP_ULP:
    return "ulp";
  case ESP_SLEEP_WAKEUP_GPIO:
    return "gpio";
  case ESP_SLEEP_WAKEUP_UART:
    return "uart";
  case ESP_SLEEP_WAKEUP_UNDEFINED:
    return "cold_boot"; // not a wake from sleep
  default:
    MQ_DEBUG_PRINTF("[sleep] woke for an unhandled reason: %d\n",
                    (int)wakeup_reason);
    return "unknown";
  }
}

/*!
    @brief  Reports whether this boot is a wake from sleep rather than a
            power-on or reset boot.
    @return True if the chip woke from a sleep wakeup source, False otherwise.
*/
bool Adafruit_Marquee::didWakeFromSleep() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  return wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED;
}

/*!
    @brief  Enables the ESP32 timer wakeup source.
    @param  wakeup_time_sec  Expected number of seconds to wait before waking
   up.
    @return True if the timer wakeup was successfully enabled, False otherwise.
*/
bool Adafruit_Marquee::enableTimerWakeup(uint64_t wakeup_time_sec) {
  esp_err_t err = esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000ULL);
  if (err != ESP_OK) {
    MQ_DEBUG_PRINTF("[sleep] ERROR: could not enable timer wakeup: %d\n",
                    (int)err);
    return false;
  }
  return true;
}

/*!
    @brief  Tears down MQTT session, WiFi and USB before entering sleep.
*/
void Adafruit_Marquee::disconnectBeforeSleep() {
  flash.syncDevice();
  _mqtt->disconnect();
  _disconnect();
  // Blocks until the host has had a chance to read the pending log lines,
  // otherwise the detach() below drops them along with the bus.
  MQ_DEBUG_FLUSH();
  TinyUSBDevice.detach();
  delay(10);
}

#else // !ARDUINO_ARCH_ESP32

/*!
    @brief  Returns why the chip came out of its last sleep.
    @return "unknown", always.
*/
const char *Adafruit_Marquee::wakeupReason() { return "unknown"; }

/*!
    @brief  Whether this boot is a wake from sleep rather than a cold-boot.
    @return False, always.
*/
bool Adafruit_Marquee::didWakeFromSleep() { return false; }

#endif // ARDUINO_ARCH_ESP32

/*!
    @brief  Acts on a sleep request queued by the sleep feed callback. On deep
            sleep this does not return; the chip resets on wake.
*/
void Adafruit_Marquee::handleSleep() {
  if (!_is_sleep_pending) {
    return;
  }

  // Do not enter sleep until the bitmap message has been received and drawn
  if (_awaiting_bmp || _awaiting_sleep) {
    MQ_DEBUG_PRINTLN(
        "[sleep] Holding off sleep until bitmap has been received");
    return;
  }

  // If MQTT is not connected, try to reconnect up to 3 times so the "sleeping"
  // status can reach IO
  if (_mqtt && !_mqtt->connected()) {
    MQ_DEBUG_PRINTLN("[sleep] MQTT disconnected, attempting to reconnect...");
    for (uint8_t attempts = 0; attempts < 3 && !_mqtt->connected();
         attempts++) {
      // Attempt to reconnect to MQTT if the network is available
      if (isNetConnected() || initWifi(MQ_WIFI_RETRY_MS)) {
        connectMqtt();
      }
    }
    // Hold off sleeping until the MQTT connection is re-established
    if (!_mqtt->connected()) {
      MQ_DEBUG_PRINTLN(
          "[sleep] ERROR: MQTT still disconnected, sleeping anyway");
    } else {
      return;
    }
  }

  MQ_DEBUG_PRINTLN("[sleep] Entering sleep mode");

#ifdef ARDUINO_ARCH_ESP32
  if (_sleep_mode != SLEEP_MODE_DEEP && _sleep_mode != SLEEP_MODE_LIGHT) {
    MQ_DEBUG_PRINTLN("[sleep] ERROR: unsupported sleep mode, staying awake!");
    _is_sleep_pending = false;
    return;
  }

  MQ_DEBUG_PRINTLN("[sleep] Creating sleep configuration JSON payload");

  // Publish the sleeping status to the status feed before entering sleep
  JsonDocument doc;
  doc["state"] = "sleeping";
  doc["sleep_time"] = _sleep_time;
  doc["alarm_type"] = (_sleep_alarm == SLEEP_ALARM_TIMER) ? "timer" : "none";

  char payload[128];
  size_t len = serializeJson(doc, payload, sizeof(payload));
  if (len == 0 || len >= sizeof(payload)) {
    MQ_DEBUG_PRINTLN("[sleep] ERROR: could not serialize the status payload, "
                     "sleeping anyway");
  } else {
    MQ_DEBUG_PRINTLN("[sleep] Publishing sleep payload...");
    bool published = false;
    for (uint8_t i = 0; i < 3 && !published; i++) {
      published = publishStatus(payload);
    }
    if (!published) {
      MQ_DEBUG_PRINTLN(
          "[sleep] ERROR: Sleep status not sent to IO, sleeping anyway");
    }
  }

  // NOTE/TODO: _sleep_alarm is parsed but not used yet. We only support wake
  // from timer.
  if (!enableTimerWakeup(_sleep_time)) {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    _is_sleep_pending = false;
    return;
  }

  if (_sleep_mode == SLEEP_MODE_DEEP) {
    MQ_DEBUG_PRINTF("[sleep] entering deep sleep for %llu s\n",
                    (unsigned long long)_sleep_time);
    disconnectBeforeSleep();
    esp_deep_sleep_start();
    // Not reached: the chip resets on wake.
  } else {
    // The guard above leaves light sleep as the only remaining mode.
    MQ_DEBUG_PRINTF("[sleep] entering light sleep for %llu s\n",
                    (unsigned long long)_sleep_time);
    disconnectBeforeSleep();
    esp_err_t rc = esp_light_sleep_start();

    // Re-enumerate USB
    TinyUSBDevice.attach();
    delay(500);

    if (rc != ESP_OK) {
      MQ_DEBUG_PRINTF("[sleep] ERROR: could not enter light sleep: %d\n",
                      (int)rc);
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
      _is_sleep_pending = false;
      return;
    }

    MQ_DEBUG_PRINTF("[sleep] wake reason: %s\n", wakeupReason());

    // Radio is still down, so this is a clean reading for the reconnect
    sampleBattery();

    // Reconnect network on the next handleConnection() call
    _last_mqtt_attempt = millis() - _mqtt_retry_ms;
    _last_ping = millis();
  }
#else
  MQ_DEBUG_PRINTLN("[sleep] ERROR: sleep is not implemented for this platform");
#endif // ARDUINO_ARCH_ESP32

  _is_sleep_pending = false;
}
