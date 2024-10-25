#ifndef ADAFRUIT_VCNL4200_H
#define ADAFRUIT_VCNL4200_H

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>

#define VCNL4200_I2CADDR_DEFAULT 0x51 ///< Default I2C address for VCNL4200

// Register addresses
#define VCNL4200_ALS_CONF 0x00 ///< ALS configuration register.
#define VCNL4200_ALS_THDH 0x01 ///< ALS high threshold register.
#define VCNL4200_ALS_THDL 0x02 ///< ALS low threshold register.
#define VCNL4200_PS_CONF12                                                     \
  0x03 ///< Proximity sensor configuration register 1 & 2
#define VCNL4200_PS_CONF3MS                                                    \
  0x04 ///< Proximity sensor configuration register 3 & MS
#define VCNL4200_PS_CANC_LVL 0x05 ///< Proximity cancellation level register.
#define VCNL4200_PS_THDL 0x06     ///< Proximity sensor low threshold register.
#define VCNL4200_PS_THDH 0x07     ///< Proximity sensor high threshold register.
#define VCNL4200_PS_DATA 0x08     ///< Proximity sensor data register.
#define VCNL4200_ALS_DATA 0x09    ///< ALS data register.
#define VCNL4200_WHITE_DATA 0x0A  ///< White sensor register.
#define VCNL4200_INT_FLAG 0x0D    ///< Interrupt flag register.
#define VCNL4200_ID 0x0E          ///< Device ID register.

#define VCNL4200_INTFLAG_PROX_UPFLAG 0x080 ///< Proximity code saturation flag
#define VCNL4200_INTFLAG_PROX_SPFLAG                                           \
  0x40                                   ///< Proximity sunlight protection flag
#define VCNL4200_INTFLAG_ALS_LOW 0x20    ///< ALS interrupt flag
#define VCNL4200_INTFLAG_ALS_HIGH 0x10   ///< Proximity interrupt flag
#define VCNL4200_INTFLAG_PROX_CLOSE 0x02 ///< Proximity THDH trigger
#define VCNL4200_INTFLAG_PROX_AWAY 0x01  ///< Proximity THDL trigger

// ALS Integration Time settings
/**
 * @brief Enumeration for ALS integration time settings.
 */
typedef enum {
  VCNL4200_ALS_IT_50MS = 0x00,  ///< 50 ms integration time
  VCNL4200_ALS_IT_100MS = 0x01, ///< 100 ms integration time
  VCNL4200_ALS_IT_200MS = 0x02, ///< 200 ms integration time
  VCNL4200_ALS_IT_400MS = 0x03  ///< 400 ms integration time
} vcnl4200_als_it_t;

// ALS Persistence settings
/**
 * @brief Enumeration for ALS persistence settings.
 */
typedef enum {
  VCNL4200_ALS_PERS_1 = 0x00, ///< ALS persistence 1 conversion
  VCNL4200_ALS_PERS_2 = 0x01, ///< ALS persistence 2 conversions
  VCNL4200_ALS_PERS_4 = 0x02, ///< ALS persistence 4 conversions
  VCNL4200_ALS_PERS_8 = 0x03  ///< ALS persistence 8 conversions
} vcnl4200_als_pers_t;

// Proximity Sensor Integration Time settings
/**
 * @brief Enumeration for Proximity Sensor integration time settings.
 */
typedef enum {
  VCNL4200_PS_IT_1T = 0x00, ///< Proximity integration time 1T
  VCNL4200_PS_IT_2T = 0x01, ///< Proximity integration time 2T
  VCNL4200_PS_IT_3T = 0x02, ///< Proximity integration time 3T
  VCNL4200_PS_IT_4T = 0x03, ///< Proximity integration time 4T
  VCNL4200_PS_IT_8T = 0x04, ///< Proximity integration time 8T
  VCNL4200_PS_IT_9T = 0x05  ///< Proximity integration time 9T
} vcnl4200_ps_it_t;

// Proximity Sensor Persistence settings
/**
 * @brief Enumeration for Proximity Sensor persistence settings.
 */
typedef enum {
  VCNL4200_PS_PERS_1 = 0x00, ///< Proximity persistence 1 conversion
  VCNL4200_PS_PERS_2 = 0x01, ///< Proximity persistence 2 conversions
  VCNL4200_PS_PERS_3 = 0x02, ///< Proximity persistence 3 conversions
  VCNL4200_PS_PERS_4 = 0x03  ///< Proximity persistence 4 conversions
} vcnl4200_ps_pers_t;

// Proximity Sensor Duty settings
/**
 * @brief Enumeration for Proximity Sensor duty cycle settings.
 */
typedef enum {
  VCNL4200_PS_DUTY_1_160 = 0x00, ///< Proximity duty cycle 1/160
  VCNL4200_PS_DUTY_1_320 = 0x01, ///< Proximity duty cycle 1/320
  VCNL4200_PS_DUTY_1_640 = 0x02, ///< Proximity duty cycle 1/640
  VCNL4200_PS_DUTY_1_1280 = 0x03 ///< Proximity duty cycle 1/1280
} vcnl4200_ps_duty_t;

// Proximity Sensor Interrupt settings
/**
 * @brief Enumeration for Proximity Sensor interrupt settings.
 */
typedef enum {
  VCNL4200_PS_INT_DISABLE = 0x00, ///< Proximity interrupt disabled
  VCNL4200_PS_INT_CLOSE = 0x01, ///< Proximity interrupt when an object is close
  VCNL4200_PS_INT_AWAY = 0x02,  ///< Proximity interrupt when an object is away
  VCNL4200_PS_INT_BOTH = 0x03   ///< Proximity interrupt for both close and away
} vcnl4200_ps_int_t;

// LED Current settings
/**
 * @brief Enumeration for LED current settings.
 */
typedef enum {
  VCNL4200_LED_I_50MA = 0x00,  ///< LED current 50mA
  VCNL4200_LED_I_75MA = 0x01,  ///< LED current 75mA
  VCNL4200_LED_I_100MA = 0x02, ///< LED current 100mA
  VCNL4200_LED_I_120MA = 0x03, ///< LED current 120mA
  VCNL4200_LED_I_140MA = 0x04, ///< LED current 140mA
  VCNL4200_LED_I_160MA = 0x05, ///< LED current 160mA
  VCNL4200_LED_I_180MA = 0x06, ///< LED current 180mA
  VCNL4200_LED_I_200MA = 0x07  ///< LED current 200mA
} vcnl4200_led_i_t;

// Proximity Sensor Multi Pulse settings
/**
 * @brief Enumeration for Proximity Sensor multi-pulse settings.
 */
typedef enum {
  VCNL4200_PS_MPS_1 = 0x00, ///< Proximity multi pulse 1
  VCNL4200_PS_MPS_2 = 0x01, ///< Proximity multi pulse 2
  VCNL4200_PS_MPS_4 = 0x02, ///< Proximity multi pulse 4
  VCNL4200_PS_MPS_8 = 0x03  ///< Proximity multi pulse 8
} vcnl4200_ps_mps_t;

/**! Class to hold interface for VCNL4200 chip */

class Adafruit_VCNL4200 {
public:
  Adafruit_VCNL4200();
  ~Adafruit_VCNL4200();

  bool begin(uint8_t i2c_addr = VCNL4200_I2CADDR_DEFAULT,
             TwoWire *wire = &Wire);
  bool setALSshutdown(bool shutdown);
  bool getALSshutdown();
  bool setALSIntegrationTime(vcnl4200_als_it_t it);
  vcnl4200_als_it_t getALSIntegrationTime();
  bool setALSPersistence(vcnl4200_als_pers_t pers);
  vcnl4200_als_pers_t getALSPersistence();
  bool setALSthresholdHigh(uint16_t threshold);
  uint16_t getALSthresholdHigh();
  bool setALSthresholdLow(uint16_t threshold);
  uint16_t getALSthresholdLow();
  bool setInterrupt(bool enabled, bool whiteChan);
  bool getInterrupt();

  bool setProxShutdown(bool shutdown);
  bool getProxShutdown();
  bool setProxIntegrationTime(vcnl4200_ps_it_t it);
  vcnl4200_ps_it_t getProxIntegrationTime();
  bool setProxPersistence(vcnl4200_ps_pers_t pers);
  vcnl4200_ps_pers_t getProxPersistence();
  bool setProxDuty(vcnl4200_ps_duty_t duty);
  vcnl4200_ps_duty_t getProxDuty();
  bool setProxInterrupt(vcnl4200_ps_int_t interrupt);
  vcnl4200_ps_int_t getProxInterrupt();
  bool setProxHD(bool high);
  bool getProxHD();
  bool setProxSunCancelEnable(bool enable);
  bool getProxSunCancelEnable();
  bool setProxActiveForce(bool enable);
  bool getProxActiveForce();
  bool setProxSunlightDoubleImmunity(bool enable);
  bool getProxSunlightDoubleImmunity();
  bool triggerProx();
  bool setProxSmartPersistence(bool enable);
  bool getProxSmartPersistence();
  bool setProxIntLogicMode(bool mode);
  bool getProxIntLogicMode();
  bool setProxMultiPulse(vcnl4200_ps_mps_t mps);
  vcnl4200_ps_mps_t getProxMultiPulse();
  bool setProxCancellationLevel(uint16_t level);
  uint16_t getProxCancellationLevel();
  bool setProxIntThresholdLow(uint16_t threshold);
  uint16_t getProxIntThresholdLow();
  bool setProxIntThresholdHigh(uint16_t threshold);
  uint16_t getProxIntThresholdHigh();
  bool setProxLEDCurrent(vcnl4200_led_i_t current);
  vcnl4200_led_i_t getProxLEDCurrent();
  bool setProxSunProtectPolarity(bool polarity);
  bool setProxBoostTypicalSunlightCapability(bool enable);
  bool getProxBoostTypicalSunlightCapability();

  uint16_t readALSdata();
  uint16_t readProxData();
  uint16_t readWhiteData();
  uint8_t getInterruptFlags();

private:
  Adafruit_I2CDevice *i2c_dev = NULL;
};

#endif // ADAFRUIT_VCNL4200_H};
