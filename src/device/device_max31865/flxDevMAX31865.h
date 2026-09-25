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
 *  flxDevMAX31865.h
 *
 *  Spark Device object for the MAX31865 RTD Temperature Sensor Amplifier (SPI).
 *  Supports PT100 and PT1000 RTD sensors.
 */

#pragma once
#include "Adafruit_MAX31865.h"
#include "flxCore.h"
#include "flxDevice.h"

#define kMAX31865DeviceName "MAX31865"

// Define our class — SPI device, no base class sharing needed
class flxDevMAX31865 : public flxDeviceSPIType<flxDevMAX31865>
{

  public:
    flxDevMAX31865();

    static const char *getDeviceName()
    {
        return kMAX31865DeviceName;
    };

    bool onInitialize(SPIClass &);

    bool execute(void);

  private:
    // The underlying Adafruit driver — constructed in onInitialize once we know the CS pin
    Adafruit_MAX31865 *_thermo;

    // RTD type property getter/setter
    uint8_t get_rtd_type(void);
    void set_rtd_type(uint8_t);
    uint8_t _rtdType; // 0 = PT100, 1 = PT1000

    // Wire config property getter/setter
    uint8_t get_wire_config(void);
    void set_wire_config(uint8_t);
    uint8_t _wireConfig;

    // Filter property getter/setter
    bool get_filter_50hz(void);
    void set_filter_50hz(bool);
    bool _filter50Hz;

    // Data accessors — values cached by execute(). After a failed read the temperature and resistance
    // are NAN, which the JSON log writes as null: missing, never zero.
    float get_temperature(void)
    {
        return _valid_data ? _temperature : NAN;
    }
    float get_resistance(void)
    {
        return _valid_data ? _resistance : NAN;
    }
    uint16_t get_raw_rtd(void)
    {
        return _valid_data ? _rawRTD : 0;
    }
    uint8_t get_fault(void)
    {
        return _fault;
    }

    float _temperature;
    float _resistance;
    uint16_t _rawRTD;
    uint8_t _fault;
    bool _valid_data;
    bool _in_setup;

    // RTD constants
    float rtdNominal(void)
    {
        return _rtdType == 0 ? 100.0f : 1000.0f;
    }
    float refResistor(void)
    {
        return _rtdType == 0 ? 430.0f : 4300.0f;
    }

  public:
    // Properties

    // RTD type: PT100 or PT1000
    flxPropertyRWUInt8<flxDevMAX31865, &flxDevMAX31865::get_rtd_type, &flxDevMAX31865::set_rtd_type> rtdType = {
        1, // default PT1000
        {{"PT100", 0}, {"PT1000", 1}}};

    // Wire configuration
    flxPropertyRWUInt8<flxDevMAX31865, &flxDevMAX31865::get_wire_config, &flxDevMAX31865::set_wire_config> wireConfig = {
        MAX31865_3WIRE,
        {{"2-Wire", MAX31865_2WIRE}, {"3-Wire", MAX31865_3WIRE}, {"4-Wire", MAX31865_4WIRE}}};

    // 50Hz filter (for mains noise rejection)
    flxPropertyRWBool<flxDevMAX31865, &flxDevMAX31865::get_filter_50hz, &flxDevMAX31865::set_filter_50hz> filter50Hz = {
        false};

    // Data parameters
    flxParameterOutFloat<flxDevMAX31865, &flxDevMAX31865::get_temperature> temperatureC;
    flxParameterOutFloat<flxDevMAX31865, &flxDevMAX31865::get_resistance> resistanceOhms;
    flxParameterOutUInt16<flxDevMAX31865, &flxDevMAX31865::get_raw_rtd> rawRTD;
    flxParameterOutUInt8<flxDevMAX31865, &flxDevMAX31865::get_fault> faultStatus;
};
