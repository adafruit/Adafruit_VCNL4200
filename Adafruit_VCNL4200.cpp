#include "Adafruit_VCNL4200.h"

Adafruit_VCNL4200::Adafruit_VCNL4200() {}

Adafruit_VCNL4200::~Adafruit_VCNL4200() {
  if (i2c_dev) {
    delete i2c_dev;
  }
}

bool Adafruit_VCNL4200::begin(uint8_t i2c_addr, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);

  // Try to find it twice
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

  // Set shutdown to false (enable ALS)
  if (!setALSshutdown(false)) {
    return false;
  }

  if (! setALSIntegrationTime(VCNL4200_ALS_IT_50MS) ||
      ! setALSPersistence(VCNL4200_ALS_PERS_1) ||
      ! setALSthresholdLow(0) ||
      ! setALSthresholdHigh(0xFFFF) ||
      ! setInterrupt(false, false)) {
    return false;
  }

  return true;
}

bool Adafruit_VCNL4200::setALSIntegrationTime(vcnl4200_als_it_t it) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_it_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 6);
  return als_it_bits.write(it);
}

vcnl4200_als_it_t Adafruit_VCNL4200::getALSIntegrationTime() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_it_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 6);
  return (vcnl4200_als_it_t)als_it_bits.read();
}

bool Adafruit_VCNL4200::setInterrupt(bool enabled, bool whiteChan) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_int_en_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 1); // Bit 1: ALS_INT_EN
  Adafruit_BusIO_RegisterBits als_int_switch_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 5); // Bit 5: ALS_INT_SWITCH
  bool success = als_int_en_bit.write(enabled);
  success &= als_int_switch_bit.write(whiteChan);
  return success;
}

bool Adafruit_VCNL4200::getInterrupt() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_int_en_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 1); // Bit 1: ALS_INT_EN
  return als_int_en_bit.read();
}

bool Adafruit_VCNL4200::setALSPersistence(vcnl4200_als_pers_t pers) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_pers_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 2); // Bits 2-3: ALS_PERS
  return als_pers_bits.write(pers);
}

vcnl4200_als_pers_t Adafruit_VCNL4200::getALSPersistence() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_pers_bits = Adafruit_BusIO_RegisterBits(&als_conf_reg, 2, 2); // Bits 2-3: ALS_PERS
  return (vcnl4200_als_pers_t)als_pers_bits.read();
}

bool Adafruit_VCNL4200::setALSshutdown(bool shutdown) {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_sd_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 0); // Bit 0: ALS_SD
  return als_sd_bit.write(shutdown);
}

bool Adafruit_VCNL4200::getALSshutdown() {
  Adafruit_BusIO_Register als_conf_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_CONF, 2);
  Adafruit_BusIO_RegisterBits als_sd_bit = Adafruit_BusIO_RegisterBits(&als_conf_reg, 1, 0); // Bit 0: ALS_SD
  return als_sd_bit.read();
}

bool Adafruit_VCNL4200::setALSthresholdHigh(uint16_t threshold) {
  Adafruit_BusIO_Register als_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDH, 2);
  return als_thdh_reg.write(threshold);
}

uint16_t Adafruit_VCNL4200::getALSthresholdHigh() {
  Adafruit_BusIO_Register als_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDH, 2);
  return als_thdh_reg.read();
}

bool Adafruit_VCNL4200::setALSthresholdLow(uint16_t threshold) {
  Adafruit_BusIO_Register als_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDL, 2);
  return als_thdl_reg.write(threshold);
}

uint16_t Adafruit_VCNL4200::getALSthresholdLow() {
  Adafruit_BusIO_Register als_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_THDL, 2);
  return als_thdl_reg.read();
}

bool Adafruit_VCNL4200::setProxShutdown(bool shutdown) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_sd_bit = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 1, 0); // Bit 0: PS_SD
  return ps_sd_bit.write(shutdown);
}

bool Adafruit_VCNL4200::getProxShutdown() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_sd_bit = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 1, 0); // Bit 0: PS_SD
  return ps_sd_bit.read();
}

bool Adafruit_VCNL4200::setProxIntegrationTime(vcnl4200_ps_it_t it) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_it_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 3, 1); // Bits 1-3: PS_IT
  return ps_it_bits.write(it);
}

vcnl4200_ps_it_t Adafruit_VCNL4200::getProxIntegrationTime() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_it_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 3, 1); // Bits 1-3: PS_IT
  return (vcnl4200_ps_it_t)ps_it_bits.read();
}

bool Adafruit_VCNL4200::setProxPersistence(vcnl4200_ps_pers_t pers) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_pers_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 4); // Bits 4-5: PS_PERS
  return ps_pers_bits.write(pers);
}

vcnl4200_ps_pers_t Adafruit_VCNL4200::getProxPersistence() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_pers_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 4); // Bits 4-5: PS_PERS
  return (vcnl4200_ps_pers_t)ps_pers_bits.read();
}

bool Adafruit_VCNL4200::setProxDuty(vcnl4200_ps_duty_t duty) {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_duty_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 6); // Bits 6-7: PS_DUTY
  return ps_duty_bits.write(duty);
}

vcnl4200_ps_duty_t Adafruit_VCNL4200::getProxDuty() {
  Adafruit_BusIO_Register ps_conf1_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_duty_bits = Adafruit_BusIO_RegisterBits(&ps_conf1_reg, 2, 6); // Bits 6-7: PS_DUTY
  return (vcnl4200_ps_duty_t)ps_duty_bits.read();
}

bool Adafruit_VCNL4200::setProxInterrupt(vcnl4200_ps_int_t interrupt) {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_int_bits = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 2, 0+8); // Bits 0-1: PS_INT
  return ps_int_bits.write(interrupt);
}

vcnl4200_ps_int_t Adafruit_VCNL4200::getProxInterrupt() {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_int_bits = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 2, 0+8); // Bits 0-1: PS_INT
  return (vcnl4200_ps_int_t)ps_int_bits.read();
}


bool Adafruit_VCNL4200::setProxHD(bool high) {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_hd_bit = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 1, 3+8); // Bit 3: PS_HD
  return ps_hd_bit.write(high);
}

bool Adafruit_VCNL4200::getProxHD() {
  Adafruit_BusIO_Register ps_conf2_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF12, 2);
  Adafruit_BusIO_RegisterBits ps_hd_bit = Adafruit_BusIO_RegisterBits(&ps_conf2_reg, 1, 3+8); // Bit 3: PS_HD
  return ps_hd_bit.read();
}

bool Adafruit_VCNL4200::setProxSunCancelEnable(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_en_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 0); // Bit 0: PS_SC_EN
  return ps_sc_en_bit.write(enable);
}

bool Adafruit_VCNL4200::getProxSunCancelEnable() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_en_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 0); // Bit 0: PS_SC_EN
  return ps_sc_en_bit.read();
}

bool Adafruit_VCNL4200::setProxSunlightDoubleImmunity(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_adv_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 1); // Bit 1: PS_SC_ADV
  return ps_sc_adv_bit.write(enable);
}

bool Adafruit_VCNL4200::getProxSunlightDoubleImmunity() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sc_adv_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 1); // Bit 1: PS_SC_ADV
  return ps_sc_adv_bit.read();
}

bool Adafruit_VCNL4200::triggerProx() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_trig_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 2); // Bit 2: PS_TRIG
  return ps_trig_bit.write(true);
}

bool Adafruit_VCNL4200::setProxActiveForce(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_af_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 3); // Bit 3: PS_AF
  return ps_af_bit.write(enable);
}

bool Adafruit_VCNL4200::getProxActiveForce() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_af_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 3); // Bit 3: PS_AF
  return ps_af_bit.read();
}

bool Adafruit_VCNL4200::setProxSmartPersistence(bool enable) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_smart_pers_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 4); // Bit 4: PS_SMART_PERS
  return ps_smart_pers_bit.write(enable);
}

bool Adafruit_VCNL4200::getProxSmartPersistence() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_smart_pers_bit = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 1, 4); // Bit 4: PS_SMART_PERS
  return ps_smart_pers_bit.read();
}

bool Adafruit_VCNL4200::setProxMultiPulse(vcnl4200_ps_mps_t mps) {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_mps_bits = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 2, 5); // Bits 5-6: PS_MPS
  return ps_mps_bits.write(mps);
}

vcnl4200_ps_mps_t Adafruit_VCNL4200::getProxMultiPulse() {
  Adafruit_BusIO_Register ps_conf3_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_mps_bits = Adafruit_BusIO_RegisterBits(&ps_conf3_reg, 2, 5); // Bits 5-6: PS_MPS
  return (vcnl4200_ps_mps_t)ps_mps_bits.read();
}

bool Adafruit_VCNL4200::setProxLEDCurrent(vcnl4200_led_i_t current) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits led_i_bits = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 3, 0+8); // Bits 0-2: LED_I
  return led_i_bits.write(current);
}

vcnl4200_led_i_t Adafruit_VCNL4200::getProxLEDCurrent() {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits led_i_bits = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 3, 0+8); // Bits 0-2: LED_I
  return (vcnl4200_led_i_t)led_i_bits.read();
}

bool Adafruit_VCNL4200::setProxSunProtectPolarity(bool polarity) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_spo_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 3+8); // Bit 3: PS_SPO
  return ps_spo_bit.write(polarity);
}


bool Adafruit_VCNL4200::setProxBoostTypicalSunlightCapability(bool boost) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sp_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 4+8); // Bit 4: PS_SP
  return ps_sp_bit.write(boost);
}

bool Adafruit_VCNL4200::getProxBoostTypicalSunlightCapability() {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_sp_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 4+8); // Bit 4: PS_SP
  return ps_sp_bit.read();
}

bool Adafruit_VCNL4200::setProxIntLogicMode(bool mode) {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_int_logic_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 7+8); // Bit 7: PS_MS
  return ps_int_logic_bit.write(mode);
}

bool Adafruit_VCNL4200::getProxIntLogicMode() {
  Adafruit_BusIO_Register ps_ms_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CONF3MS, 2);
  Adafruit_BusIO_RegisterBits ps_int_logic_bit = Adafruit_BusIO_RegisterBits(&ps_ms_reg, 1, 7+8); // Bit 7: PS_MS
  return ps_int_logic_bit.read();
}

bool Adafruit_VCNL4200::setProxCancellationLevel(uint16_t level) {
  Adafruit_BusIO_Register ps_canc_level_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CANC_LVL, 2);
  return ps_canc_level_reg.write(level);
}

uint16_t Adafruit_VCNL4200::getProxCancellationLevel() {
  Adafruit_BusIO_Register ps_canc_level_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_CANC_LVL, 2);
  return ps_canc_level_reg.read();
}


bool Adafruit_VCNL4200::setProxIntThresholdLow(uint16_t threshold) {
  Adafruit_BusIO_Register ps_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDL, 2);
  return ps_thdl_reg.write(threshold);
}

uint16_t Adafruit_VCNL4200::getProxIntThresholdLow() {
  Adafruit_BusIO_Register ps_thdl_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDL, 2);
  return ps_thdl_reg.read();
}

bool Adafruit_VCNL4200::setProxIntThresholdHigh(uint16_t threshold) {
  Adafruit_BusIO_Register ps_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDH, 2);
  return ps_thdh_reg.write(threshold);
}

uint16_t Adafruit_VCNL4200::getProxIntThresholdHigh() {
  Adafruit_BusIO_Register ps_thdh_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_THDH, 2);
  return ps_thdh_reg.read();
}

uint16_t Adafruit_VCNL4200::readProxData() {
  Adafruit_BusIO_Register ps_data_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_PS_DATA, 2);
  return ps_data_reg.read();
}

uint16_t Adafruit_VCNL4200::readALSdata() {
  Adafruit_BusIO_Register als_data_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_ALS_DATA, 2);
  return als_data_reg.read();
}

uint16_t Adafruit_VCNL4200::readWhiteData() {
  Adafruit_BusIO_Register white_data_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_WHITE_DATA, 2);
  return white_data_reg.read();
}

uint8_t Adafruit_VCNL4200::getInterruptFlags() {
  Adafruit_BusIO_Register int_flag_reg = Adafruit_BusIO_Register(i2c_dev, VCNL4200_INT_FLAG, 2);
  return int_flag_reg.read() >> 8;
}
