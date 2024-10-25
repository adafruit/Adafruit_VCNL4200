#include "Adafruit_VCNL4200.h"

/**
 * @brief Constructor for Adafruit_VCNL4200 class.
 */
Adafruit_VCNL4200::Adafruit_VCNL4200() {}

/**
 * @brief Destructor for Adafruit_VCNL4200 class. Deletes I2C device object if it exists.
 */
Adafruit_VCNL4200::~Adafruit_VCNL4200() {
  if (i2c_dev) {
    delete i2c_dev;
  }
}

/**
 * @brief Initializes the VCNL4200 sensor with the specified I2C address and bus.
 * 
 * Tries to initialize communication twice if the first attempt fails. Verifies the sensor
 * by checking the device ID and configures default settings for ALS.
 * 
 * @param i2c_addr I2C address of the VCNL4200 sensor (default is 0x51).
 * @param wire Pointer to the I2C bus to be used for communication (default is &Wire).
 * @return True if initialization is successful, false otherwise.
 */
bool Adafruit_VCNL4200::begin(uint8_t i2c_addr, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);

  // Try to find the device twice
  if (!i2c_dev->begin()) {
    delay(10);
    if (!i2c_dev->begin()) {
      Serial.println("Addr not found");
      return false;
    }
  }
  
  // Check device ID
  Adafruit_BusIO_Register id_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ID, 2);
  uint16_t device_id = id_reg.read();
  if (device_id != 0x1058) { // Expected device ID for VCNL4200
    return false;
  }

  // Set ALS shutdown to false (enable ALS)
  if (!setALSshutdown(false)) {
    return false;
  }

  // Configure ALS with default settings
  if (! setALSIntegrationTime(VCNL4200_ALS_IT_50MS) ||
      ! setALSPersistence(VCNL4200_ALS_PERS_1) ||
      ! setALSthresholdLow(0) ||
      ! setALSthresholdHigh(0xFFFF) ||
      ! setInterrupt(false, false)) {
    return false;
  }

  return true;
}

/**
 * @brief Sets the ALS (Ambient Light Sensor) integration time.
 * 
 * Configures the integration time for the ambient light sensor to control sensitivity and range.
 * 
 * @param it Desired ALS integration time setting of type `vcnl4200_als_it_t`.
 * @return True if the integration time was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setALSIntegrationTime(vcnl4200_als_it_t it) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_it_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 6);
  return als_it_bits.write(it);
}

/**
 * @brief Gets the current ALS (Ambient Light Sensor) integration time setting.
 * 
 * Reads the integration time setting of the ALS sensor, which controls sensitivity and maximum detection range.
 * 
 * @return The current ALS integration time setting as a `vcnl4200_als_it_t` enum value.
 */
vcnl4200_als_it_t Adafruit_VCNL4200::getALSIntegrationTime() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_it_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 6);
  return (vcnl4200_als_it_t)als_it_bits.read();
}

/**
 * @brief Configures the ALS interrupt settings.
 * 
 * Enables or disables the ALS (Ambient Light Sensor) interrupt, and selects the interrupt channel.
 * 
 * @param enabled Set to true to enable ALS interrupt, false to disable.
 * @param whiteChan Set to true for white channel interrupt, false for ALS channel interrupt.
 * @return True if the configuration was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setInterrupt(bool enabled, bool whiteChan) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_int_en_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 1); // Bit 1: ALS_INT_EN
  Adafruit_BusIO_RegisterBits als_int_switch_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 5); // Bit 5: ALS_INT_SWITCH
  bool success = als_int_en_bit.write(enabled);
  success &= als_int_switch_bit.write(whiteChan);
  return success;
}

/**
 * @brief Gets the current ALS interrupt enable state.
 * 
 * Checks if the ALS interrupt is currently enabled.
 * 
 * @return True if ALS interrupt is enabled, false otherwise.
 */
bool Adafruit_VCNL4200::getInterrupt() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_int_en_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 1); // Bit 1: ALS_INT_EN
  return als_int_en_bit.read();
}

/**
 * @brief Sets the ALS (Ambient Light Sensor) persistence setting.
 * 
 * Configures the persistence level of the ALS interrupt, which determines how many consecutive
 * ALS threshold triggers are required to activate the interrupt.
 * 
 * @param pers Desired ALS persistence level of type `vcnl4200_als_pers_t`.
 * @return True if the persistence level was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setALSPersistence(vcnl4200_als_pers_t pers) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_pers_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 2); // Bits 2-3: ALS_PERS
  return als_pers_bits.write(pers);
}

/**
 * @brief Gets the current ALS persistence setting.
 * 
 * Reads the persistence level of the ALS interrupt, which defines the number of consecutive triggers
 * required to activate the interrupt.
 * 
 * @return The current ALS persistence level as a `vcnl4200_als_pers_t` enum value.
 */
vcnl4200_als_pers_t Adafruit_VCNL4200::getALSPersistence() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_pers_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 2); // Bits 2-3: ALS_PERS
  return (vcnl4200_als_pers_t)als_pers_bits.read();
}

/**
 * @brief Enables or disables the ALS (Ambient Light Sensor) shutdown mode.
 * 
 * Controls the power state of the ALS, allowing it to be shut down to conserve power.
 * 
 * @param shutdown Set to true to enable shutdown mode (power off ALS), false to disable (power on ALS).
 * @return True if the shutdown state was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setALSshutdown(bool shutdown) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_sd_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 0); // Bit 0: ALS_SD
  return als_sd_bit.write(shutdown);
}

/**
 * @brief Gets the current ALS (Ambient Light Sensor) shutdown state.
 * 
 * Reads the power state of the ALS to check if it is in shutdown mode.
 * 
 * @return True if ALS is currently in shutdown mode, false if it is active.
 */
bool Adafruit_VCNL4200::getALSshutdown() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_sd_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 0); // Bit 0: ALS_SD
  return als_sd_bit.read();
}

/**
 * @brief Sets the high threshold for the ALS (Ambient Light Sensor) interrupt.
 * 
 * Configures the upper limit for ambient light measurement. If the ALS reading exceeds this threshold
 * and ALS interrupt is enabled, an interrupt will be triggered.
 * 
 * @param threshold The 16-bit high threshold value for the ALS interrupt.
 * @return True if the threshold was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setALSthresholdHigh(uint16_t threshold) {
  Adafruit_BusIO_Register als_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDH, 2);
  return als_thdh_reg.write(threshold);
}

/**
 * @brief Gets the current high threshold for the ALS (Ambient Light Sensor) interrupt.
 * 
 * Reads the upper limit value for ambient light measurement, used to trigger an interrupt if the ALS
 * reading exceeds this value.
 * 
 * @return The current 16-bit high threshold value for the ALS interrupt.
 */
uint16_t Adafruit_VCNL4200::getALSthresholdHigh() {
  Adafruit_BusIO_Register als_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDH, 2);
  return als_thdh_reg.read();
}

/**
 * @brief Sets the low threshold for the ALS (Ambient Light Sensor) interrupt.
 * 
 * Configures the lower limit for ambient light measurement. If the ALS reading falls below this threshold
 * and ALS interrupt is enabled, an interrupt will be triggered.
 * 
 * @param threshold The 16-bit low threshold value for the ALS interrupt.
 * @return True if the threshold was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setALSthresholdLow(uint16_t threshold) {
  Adafruit_BusIO_Register als_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDL, 2);
  return als_thdl_reg.write(threshold);
}

/**
 * @brief Gets the current low threshold for the ALS (Ambient Light Sensor) interrupt.
 * 
 * Reads the lower limit value for ambient light measurement, used to trigger an interrupt if the ALS
 * reading falls below this value.
 * 
 * @return The current 16-bit low threshold value for the ALS interrupt.
 */
uint16_t Adafruit_VCNL4200::getALSthresholdLow() {
  Adafruit_BusIO_Register als_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDL, 2);
  return als_thdl_reg.read();
}

/**
 * @brief Enables or disables the proximity sensor (PS) shutdown mode.
 * 
 * Controls the power state of the proximity sensor, allowing it to be shut down to conserve power.
 * 
 * @param shutdown Set to true to enable shutdown mode (power off proximity sensor), false to disable (power on proximity sensor).
 * @return True if the shutdown state was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxShutdown(bool shutdown) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_sd_bit = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 1, 0); // Bit 0: PS_SD
  return ps_sd_bit.write(shutdown);
}

/**
 * @brief Gets the current shutdown state of the proximity sensor (PS).
 * 
 * Reads the power state of the proximity sensor to check if it is in shutdown mode.
 * 
 * @return True if the proximity sensor is currently in shutdown mode, false if it is active.
 */
bool Adafruit_VCNL4200::getProxShutdown() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_sd_bit = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 1, 0); // Bit 0: PS_SD
  return ps_sd_bit.read();
}

/**
 * @brief Sets the integration time for the proximity sensor (PS).
 * 
 * Configures the integration time for the proximity sensor, which affects the duration for which
 * the sensor is sensitive to reflected light.
 * 
 * @param it Desired integration time setting for the proximity sensor of type `vcnl4200_ps_it_t`.
 * @return True if the integration time was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxIntegrationTime(vcnl4200_ps_it_t it) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_it_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 3, 1); // Bits 1-3: PS_IT
  return ps_it_bits.write(it);
}

/**
 * @brief Gets the current integration time setting for the proximity sensor (PS).
 * 
 * Reads the integration time setting of the proximity sensor, which defines the duration of light sensitivity.
 * 
 * @return The current integration time setting for the proximity sensor as a `vcnl4200_ps_it_t` enum value.
 */
vcnl4200_ps_it_t Adafruit_VCNL4200::getProxIntegrationTime() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_it_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 3, 1); // Bits 1-3: PS_IT
  return (vcnl4200_ps_it_t)ps_it_bits.read();
}

/**
 * @brief Sets the persistence setting for the proximity sensor (PS).
 * 
 * Configures the persistence level of the proximity sensor interrupt, defining how many consecutive
 * threshold triggers are required to activate the interrupt.
 * 
 * @param pers Desired persistence level for the proximity sensor of type `vcnl4200_ps_pers_t`.
 * @return True if the persistence level was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxPersistence(vcnl4200_ps_pers_t pers) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_pers_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 4); // Bits 4-5: PS_PERS
  return ps_pers_bits.write(pers);
}

/**
 * @brief Gets the current persistence setting for the proximity sensor (PS).
 * 
 * Reads the persistence level of the proximity sensor interrupt, which defines the required consecutive triggers
 * to activate the interrupt.
 * 
 * @return The current persistence level for the proximity sensor as a `vcnl4200_ps_pers_t` enum value.
 */
vcnl4200_ps_pers_t Adafruit_VCNL4200::getProxPersistence() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_pers_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 4); // Bits 4-5: PS_PERS
  return (vcnl4200_ps_pers_t)ps_pers_bits.read();
}

/**
 * @brief Sets the duty cycle for the proximity sensor (PS).
 * 
 * Configures the duty cycle of the infrared emitter for the proximity sensor, which affects
 * power consumption and response time.
 * 
 * @param duty Desired duty cycle for the proximity sensor of type `vcnl4200_ps_duty_t`.
 * @return True if the duty cycle was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxDuty(vcnl4200_ps_duty_t duty) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_duty_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 6); // Bits 6-7: PS_DUTY
  return ps_duty_bits.write(duty);
}

/**
 * @brief Gets the current duty cycle setting for the proximity sensor (PS).
 * 
 * Reads the duty cycle configuration of the infrared emitter, which controls the balance
 * between power consumption and response time.
 * 
 * @return The current duty cycle setting for the proximity sensor as a `vcnl4200_ps_duty_t` enum value.
 */
vcnl4200_ps_duty_t Adafruit_VCNL4200::getProxDuty() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_duty_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 6); // Bits 6-7: PS_DUTY
  return (vcnl4200_ps_duty_t)ps_duty_bits.read();
}

/**
 * @brief Sets the interrupt mode for the proximity sensor (PS).
 * 
 * Configures the interrupt condition for the proximity sensor, which determines when
 * an interrupt will be triggered based on the detected proximity.
 * 
 * @param interrupt Desired interrupt condition for the proximity sensor of type `vcnl4200_ps_int_t`.
 * @return True if the interrupt mode was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxInterrupt(vcnl4200_ps_int_t interrupt) {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_int_bits = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 2, 0+8); // Bits 0-1: PS_INT
  return ps_int_bits.write(interrupt);
}

/**
 * @brief Gets the current interrupt mode for the proximity sensor (PS).
 * 
 * Reads the configured interrupt condition for the proximity sensor, which specifies
 * the proximity conditions under which an interrupt will be triggered.
 * 
 * @return The current interrupt mode for the proximity sensor as a `vcnl4200_ps_int_t` enum value.
 */
vcnl4200_ps_int_t Adafruit_VCNL4200::getProxInterrupt() {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_int_bits = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 2, 0+8); // Bits 0-1: PS_INT
  return (vcnl4200_ps_int_t)ps_int_bits.read();
}

/**
 * @brief Sets the proximity sensor (PS) resolution to high definition (HD).
 * 
 * Configures the proximity sensor's output resolution, enabling higher precision measurements
 * by selecting high definition mode.
 * 
 * @param high Set to true to enable high definition mode (16-bit resolution), false for standard resolution (12-bit).
 * @return True if the resolution setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxHD(bool high) {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_hd_bit = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 1, 3+8); // Bit 3: PS_HD
  return ps_hd_bit.write(high);
}

/**
 * @brief Gets the current resolution mode of the proximity sensor (PS).
 * 
 * Checks if the proximity sensor is set to high definition (16-bit) mode or standard (12-bit) resolution.
 * 
 * @return True if high definition mode is enabled, false if standard resolution is active.
 */
bool Adafruit_VCNL4200::getProxHD() {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_hd_bit = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 1, 3+8); // Bit 3: PS_HD
  return ps_hd_bit.read();
}

/**
 * @brief Enables or disables the sunlight cancellation feature for the proximity sensor (PS).
 * 
 * Controls the sunlight cancellation feature, which improves proximity detection accuracy in bright
 * or sunny conditions by mitigating background light interference.
 * 
 * @param enable Set to true to enable sunlight cancellation, false to disable it.
 * @return True if the setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxSunCancelEnable(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_en_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 0); // Bit 0: PS_SC_EN
  return ps_sc_en_bit.write(enable);
}

/**
 * @brief Gets the current state of the sunlight cancellation feature for the proximity sensor (PS).
 * 
 * Checks if the sunlight cancellation feature is enabled to improve proximity detection accuracy
 * in bright lighting conditions.
 * 
 * @return True if sunlight cancellation is enabled, false if it is disabled.
 */
bool Adafruit_VCNL4200::getProxSunCancelEnable() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_en_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 0); // Bit 0: PS_SC_EN
  return ps_sc_en_bit.read();
}

/**
 * @brief Enables or disables double immunity to sunlight for the proximity sensor (PS).
 * 
 * Configures an enhanced sunlight immunity mode, which increases the sensor’s ability to filter
 * out interference from bright sunlight for improved proximity detection.
 * 
 * @param enable Set to true to enable double sunlight immunity, false to disable it.
 * @return True if the setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxSunlightDoubleImmunity(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_adv_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 1); // Bit 1: PS_SC_ADV
  return ps_sc_adv_bit.write(enable);
}

/**
 * @brief Gets the current state of double immunity to sunlight for the proximity sensor (PS).
 * 
 * Checks if the enhanced sunlight immunity mode is enabled to further reduce interference from
 * bright sunlight during proximity detection.
 * 
 * @return True if double sunlight immunity is enabled, false if it is disabled.
 */
bool Adafruit_VCNL4200::getProxSunlightDoubleImmunity() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_adv_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 1); // Bit 1: PS_SC_ADV
  return ps_sc_adv_bit.read();
}

/**
 * @brief Triggers a single proximity measurement manually.
 * 
 * Initiates a one-time proximity measurement in active force mode. This can be used when
 * proximity measurements are not continuous and need to be triggered individually.
 * 
 * @return True if the trigger command was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::triggerProx() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_trig_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 2); // Bit 2: PS_TRIG
  return ps_trig_bit.write(true);
}

/**
 * @brief Enables or disables active force mode for the proximity sensor (PS).
 * 
 * Configures the proximity sensor to operate in active force mode, where measurements are taken
 * only when manually triggered by `triggerProx`.
 * 
 * @param enable Set to true to enable active force mode, false to disable it.
 * @return True if the setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxActiveForce(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_af_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 3); // Bit 3: PS_AF
  return ps_af_bit.write(enable);
}

/**
 * @brief Gets the current active force mode state of the proximity sensor (PS).
 * 
 * Checks if the proximity sensor is configured in active force mode, where measurements
 * are triggered manually instead of continuously.
 * 
 * @return True if active force mode is enabled, false if it is disabled.
 */
bool Adafruit_VCNL4200::getProxActiveForce() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_af_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 3); // Bit 3: PS_AF
  return ps_af_bit.read();
}

/**
 * @brief Enables or disables smart persistence for the proximity sensor (PS).
 * 
 * Configures the smart persistence feature, which helps reduce false triggers by adjusting
 * the persistence behavior based on ambient conditions.
 * 
 * @param enable Set to true to enable smart persistence, false to disable it.
 * @return True if the setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxSmartPersistence(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_smart_pers_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 4); // Bit 4: PS_SMART_PERS
  return ps_smart_pers_bit.write(enable);
}

/**
 * @brief Gets the current state of the smart persistence feature for the proximity sensor (PS).
 * 
 * Checks if the smart persistence feature is enabled, which adjusts persistence behavior to
 * reduce false triggers.
 * 
 * @return True if smart persistence is enabled, false if it is disabled.
 */
bool Adafruit_VCNL4200::getProxSmartPersistence() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_smart_pers_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 4); // Bit 4: PS_SMART_PERS
  return ps_smart_pers_bit.read();
}

/**
 * @brief Sets the multi-pulse mode for the proximity sensor (PS).
 * 
 * Configures the number of infrared pulses used by the proximity sensor in a single measurement.
 * Increasing the pulse count can improve accuracy, especially in high ambient light conditions.
 * 
 * @param mps Desired multi-pulse setting for the proximity sensor of type `vcnl4200_ps_mps_t`.
 * @return True if the multi-pulse setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxMultiPulse(vcnl4200_ps_mps_t mps) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_mps_bits = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 2, 5); // Bits 5-6: PS_MPS
  return ps_mps_bits.write(mps);
}

/**
 * @brief Gets the current multi-pulse setting of the proximity sensor (PS).
 * 
 * Reads the multi-pulse mode of the proximity sensor, which indicates the number of infrared pulses
 * used in each proximity measurement.
 * 
 * @return The current multi-pulse setting for the proximity sensor as a `vcnl4200_ps_mps_t` enum value.
 */
vcnl4200_ps_mps_t Adafruit_VCNL4200::getProxMultiPulse() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_mps_bits = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 2, 5); // Bits 5-6: PS_MPS
  return (vcnl4200_ps_mps_t)ps_mps_bits.read();
}

/**
 * @brief Sets the LED current for the proximity sensor (PS).
 * 
 * Configures the driving current for the infrared LED used in proximity detection, which affects
 * the range and power consumption of the sensor.
 * 
 * @param current Desired LED current setting for the proximity sensor of type `vcnl4200_led_i_t`.
 * @return True if the LED current setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxLEDCurrent(vcnl4200_led_i_t current) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits led_i_bits = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 3, 0+8); // Bits 0-2: LED_I
  return led_i_bits.write(current);
}

/**
 * @brief Gets the current LED driving current setting for the proximity sensor (PS).
 * 
 * Reads the LED current configuration, which controls the power and range of the infrared LED
 * used in proximity detection.
 * 
 * @return The current LED current setting for the proximity sensor as a `vcnl4200_led_i_t` enum value.
 */
vcnl4200_led_i_t Adafruit_VCNL4200::getProxLEDCurrent() {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits led_i_bits = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 3, 0+8); // Bits 0-2: LED_I
  return (vcnl4200_led_i_t)led_i_bits.read();
}

/**
 * @brief Sets the polarity of the sunlight protection output for the proximity sensor (PS).
 * 
 * Configures the polarity of the sunlight protection output signal, which affects how
 * sunlight interference is managed in proximity detection.
 * 
 * @param polarity Set to true for active high polarity, false for active low polarity.
 * @return True if the polarity setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxSunProtectPolarity(bool polarity) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_spo_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 3+8); // Bit 3: PS_SPO
  return ps_spo_bit.write(polarity);
}

/**
 * @brief Enables or disables boosted sunlight protection capability for the proximity sensor (PS).
 * 
 * Boosts the proximity sensor's resistance to typical sunlight interference for more reliable
 * proximity measurements in bright ambient conditions.
 * 
 * @param boost Set to true to enable boosted sunlight protection, false to disable it.
 * @return True if the boosted sunlight protection setting was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxBoostTypicalSunlightCapability(bool boost) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sp_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 4+8); // Bit 4: PS_SP
  return ps_sp_bit.write(boost);
}

/**
 * @brief Gets the current state of boosted sunlight protection capability for the proximity sensor (PS).
 * 
 * Checks if the proximity sensor's boosted sunlight protection feature is enabled, which enhances
 * measurement reliability in bright ambient light conditions.
 * 
 * @return True if boosted sunlight protection is enabled, false if it is disabled.
 */
bool Adafruit_VCNL4200::getProxBoostTypicalSunlightCapability() {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sp_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 4+8); // Bit 4: PS_SP
  return ps_sp_bit.read();
}

/**
 * @brief Sets the interrupt logic mode for the proximity sensor (PS).
 * 
 * Configures the interrupt output logic mode for the proximity sensor, determining
 * if the interrupt signal is active high or active low.
 * 
 * @param mode Set to true for active high logic, false for active low logic.
 * @return True if the interrupt logic mode was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxIntLogicMode(bool mode) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_int_logic_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 7+8); // Bit 7: PS_MS
  return ps_int_logic_bit.write(mode);
}

/**
 * @brief Gets the current interrupt logic mode for the proximity sensor (PS).
 * 
 * Checks if the interrupt output logic mode is configured as active high or active low.
 * 
 * @return True if interrupt logic is active high, false if it is active low.
 */
bool Adafruit_VCNL4200::getProxIntLogicMode() {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_int_logic_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 7+8); // Bit 7: PS_MS
  return ps_int_logic_bit.read();
}

/**
 * @brief Sets the proximity sensor (PS) cancellation level.
 * 
 * Configures the cancellation level for the proximity sensor, which helps to reduce
 * interference from background light by subtracting a baseline level.
 * 
 * @param level Desired cancellation level for the proximity sensor, as a 16-bit value.
 * @return True if the cancellation level was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxCancellationLevel(uint16_t level) {
  Adafruit_BusIO_Register ps_canc_level_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CANC_LVL, 2);
  return ps_canc_level_reg.write(level);
}

/**
 * @brief Gets the current cancellation level for the proximity sensor (PS).
 * 
 * Reads the configured cancellation level, which reduces background light interference by
 * setting a baseline level for proximity detection.
 * 
 * @return The current 16-bit cancellation level for the proximity sensor.
 */
uint16_t Adafruit_VCNL4200::getProxCancellationLevel() {
  Adafruit_BusIO_Register ps_canc_level_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CANC_LVL, 2);
  return ps_canc_level_reg.read();
}

/**
 * @brief Sets the low threshold for the proximity sensor (PS) interrupt.
 * 
 * Configures the lower limit for proximity detection. If the proximity reading falls below this
 * threshold and the interrupt is enabled, an interrupt will be triggered.
 * 
 * @param threshold The 16-bit low threshold value for the proximity sensor interrupt.
 * @return True if the threshold was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxIntThresholdLow(uint16_t threshold) {
  Adafruit_BusIO_Register ps_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDL, 2);
  return ps_thdl_reg.write(threshold);
}

/**
 * @brief Gets the current low threshold for the proximity sensor (PS) interrupt.
 * 
 * Reads the configured lower limit for proximity detection, which is used to trigger
 * an interrupt if the proximity reading falls below this value.
 * 
 * @return The current 16-bit low threshold value for the proximity sensor interrupt.
 */
uint16_t Adafruit_VCNL4200::getProxIntThresholdLow() {
  Adafruit_BusIO_Register ps_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDL, 2);
  return ps_thdl_reg.read();
}

/**
 * @brief Sets the high threshold for the proximity sensor (PS) interrupt.
 * 
 * Configures the upper limit for proximity detection. If the proximity reading exceeds this
 * threshold and the interrupt is enabled, an interrupt will be triggered.
 * 
 * @param threshold The 16-bit high threshold value for the proximity sensor interrupt.
 * @return True if the threshold was successfully written, false otherwise.
 */
bool Adafruit_VCNL4200::setProxIntThresholdHigh(uint16_t threshold) {
  Adafruit_BusIO_Register ps_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDH, 2);
  return ps_thdh_reg.write(threshold);
}

/**
 * @brief Gets the current high threshold for the proximity sensor (PS) interrupt.
 * 
 * Reads the configured upper limit for proximity detection, which is used to trigger
 * an interrupt if the proximity reading exceeds this value.
 * 
 * @return The current 16-bit high threshold value for the proximity sensor interrupt.
 */
uint16_t Adafruit_VCNL4200::getProxIntThresholdHigh() {
  Adafruit_BusIO_Register ps_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDH, 2);
  return ps_thdh_reg.read();
}

/**
 * @brief Reads the current proximity data from the proximity sensor (PS).
 * 
 * Retrieves the raw proximity measurement from the sensor.
 * 
 * @return The 16-bit proximity data value.
 */
uint16_t Adafruit_VCNL4200::readProxData() {
  Adafruit_BusIO_Register ps_data_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_DATA, 2);
  return ps_data_reg.read();
}

/**
 * @brief Reads the current ambient light sensor (ALS) data.
 * 
 * Retrieves the raw ambient light measurement from the sensor.
 * 
 * @return The 16-bit ALS data value.
 */
uint16_t Adafruit_VCNL4200::readALSdata() {
  Adafruit_BusIO_Register als_data_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_DATA, 2);
  return als_data_reg.read();
}

/**
 * @brief Reads the current white light data.
 * 
 * Retrieves the raw white light measurement, representing the sensor’s sensitivity to white light.
 * 
 * @return The 16-bit white light data value.
 */
uint16_t Adafruit_VCNL4200::readWhiteData() {
  Adafruit_BusIO_Register white_data_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_WHITE_DATA, 2);
  return white_data_reg.read();
}

/**
 * @brief Reads the interrupt flags from the sensor.
 * 
 * Retrieves the current interrupt status flags, which indicate various sensor states,
 * such as threshold crossings or sunlight protection events.
 * 
 * @return The 8-bit interrupt flags.
 */
uint8_t Adafruit_VCNL4200::getInterruptFlags() {
  Adafruit_BusIO_Register int_flag_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_INT_FLAG, 2);
  return int_flag_reg.read() >> 8;
}
