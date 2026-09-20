/*
 *---------------------------------------------------------------------------------
 *
 * Copyright (c) 2022-2025, SparkFun Electronics Inc.
 *
 * SPDX-License-Identifier: MIT
 *
 *---------------------------------------------------------------------------------
 */

/*
 *
 *  flxDevMAX31865.cpp
 *
 *  Spark Device object for the MAX31865 RTD Temperature Sensor Amplifier (SPI).
 */

#include "Arduino.h"

#include "flxDevMAX31865.h"

//----------------------------------------------------------------------------------------------------------
/// @brief Constructor
///
flxDevMAX31865::flxDevMAX31865()
    : _thermo{nullptr}, _rtdType{1}, _wireConfig{MAX31865_3WIRE}, _filter50Hz{false}, _temperature{0.0f},
      _resistance{0.0f}, _rawRTD{0}, _fault{0}, _valid_data{false}, _in_setup{false}
{

    setName(getDeviceName(), "MAX31865 RTD Temperature Sensor");

    // Properties
    flxRegister(rtdType, "RTD Type", "RTD sensor type (PT100 or PT1000)");
    flxRegister(wireConfig, "Wire Configuration", "RTD wiring (2, 3, or 4 wire)");
    flxRegister(filter50Hz, "50Hz Filter", "Enable 50Hz noise filter (default 60Hz)");

    // Data parameters
    flxRegister(temperatureC, "Temperature (C)", "RTD temperature in degrees Celsius");
    flxRegister(resistanceOhms, "Resistance (Ohms)", "RTD resistance in Ohms");
    flxRegister(rawRTD, "Raw RTD", "Raw RTD ADC value");
    flxRegister(faultStatus, "Fault", "Fault status register (0 = no fault)");
}

//----------------------------------------------------------------------------------------------------------
///
/// @brief Called during the startup/initialization of the driver (after the constructor is called).
///
/// @param spiPort - The Arduino SPI port
///
/// @return true on success
///
bool flxDevMAX31865::onInitialize(SPIClass &spiPort)
{

    // Create the Adafruit driver with the CS pin from the framework
    _thermo = new Adafruit_MAX31865(chipSelect(), &spiPort);
    if (!_thermo)
    {
        flxLog_E(F("%s : Failed to allocate driver."), name());
        return false;
    }

    _in_setup = true;

    // Initialize with configured wire type
    if (!_thermo->begin((max31865_numwires_t)_wireConfig))
    {
        flxLog_E(F("%s : Failed to initialize sensor."), name());
        delete _thermo;
        _thermo = nullptr;
        _in_setup = false;
        return false;
    }

    // Apply 50Hz filter setting
    _thermo->enable50Hz(_filter50Hz);

    _in_setup = false;

    return true;
}

//---------------------------------------------------------------------------
// RTD type property - getter/setter
//---------------------------------------------------------------------------

uint8_t flxDevMAX31865::get_rtd_type(void)
{
    return _rtdType;
}

void flxDevMAX31865::set_rtd_type(uint8_t value)
{
    if (value == _rtdType && !_in_setup)
        return;

    _rtdType = value;
}

//---------------------------------------------------------------------------
// Wire configuration property - getter/setter
//---------------------------------------------------------------------------

uint8_t flxDevMAX31865::get_wire_config(void)
{
    return _wireConfig;
}

void flxDevMAX31865::set_wire_config(uint8_t value)
{
    if (value == _wireConfig && !_in_setup)
        return;

    _wireConfig = value;

    if (!_thermo || (!isInitialized() && !_in_setup))
        return;

    _thermo->setWires((max31865_numwires_t)_wireConfig);
}

//---------------------------------------------------------------------------
// 50Hz filter property - getter/setter
//---------------------------------------------------------------------------

bool flxDevMAX31865::get_filter_50hz(void)
{
    return _filter50Hz;
}

void flxDevMAX31865::set_filter_50hz(bool value)
{
    if (value == _filter50Hz && !_in_setup)
        return;

    _filter50Hz = value;

    if (!_thermo || (!isInitialized() && !_in_setup))
        return;

    _thermo->enable50Hz(_filter50Hz);
}

//---------------------------------------------------------------------------
///
/// @brief Called right before data parameters are read — triggers an RTD measurement
///
bool flxDevMAX31865::execute(void)
{
    if (!_thermo)
    {
        _valid_data = false;
        return false;
    }

    // Read raw RTD value (this triggers a one-shot conversion internally)
    _rawRTD = _thermo->readRTD();

    // Calculate resistance from raw value
    // ratio = rawRTD / 32768, resistance = ratio * refResistor
    float ratio = (float)_rawRTD / 32768.0f;
    _resistance = ratio * refResistor();

    // Calculate temperature using Callendar-Van Dusen equation (handled by library)
    _temperature = _thermo->calculateTemperature(_rawRTD, rtdNominal(), refResistor());

    // Check for faults
    _fault = _thermo->readFault();
    if (_fault)
    {
        if (_fault & MAX31865_FAULT_HIGHTHRESH)
            flxLog_W(F("%s : RTD High Threshold fault"), name());
        if (_fault & MAX31865_FAULT_LOWTHRESH)
            flxLog_W(F("%s : RTD Low Threshold fault"), name());
        if (_fault & MAX31865_FAULT_REFINLOW)
            flxLog_W(F("%s : REFIN- > 0.85 x Bias fault"), name());
        if (_fault & MAX31865_FAULT_REFINHIGH)
            flxLog_W(F("%s : REFIN- < 0.85 x Bias fault"), name());
        if (_fault & MAX31865_FAULT_RTDINLOW)
            flxLog_W(F("%s : RTDIN- < 0.85 x Bias fault"), name());
        if (_fault & MAX31865_FAULT_OVUV)
            flxLog_W(F("%s : Over/Under voltage fault"), name());

        _thermo->clearFault();
        _valid_data = false;
        return false;
    }

    _valid_data = true;
    return true;
}
