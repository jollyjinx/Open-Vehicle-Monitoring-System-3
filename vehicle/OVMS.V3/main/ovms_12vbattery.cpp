/*
;    Project:       Open Vehicle Monitor System
;    Date:          20th November 2022
;
;    (C) 2022       Patrick Stein @jollyjinx
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "12battery";

#include "ovms_boot.h"
extern boot_data_t boot_data;

#include "ovms_config.h"
#include "ovms_12vbattery.h"
#include "ovms_peripherals.h"

//MARK: Battery Measuring

float currentBatteryLevelAvg()
{
    #ifdef CONFIG_OVMS_COMP_ADC
      adc1_config_width(ADC_WIDTH_12Bit);
      adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);

      uint32_t min = 4294967295;
      uint32_t max = 0;
      uint32_t sum = 0;

      for (int i = 0; i < 7; i++)
      {
        uint32_t value = adc1_get_raw(ADC1_CHANNEL_0);

        min = value < min ? value : min;
        max = value > max ? value : max;

        sum += value;
      }
      sum -= min;
      sum -= max;

      float avg = ((float)sum) / 5.0f;
//      ESP_LOGI(TAG, "min:%d max:%d sum:%d avg:%f",min,max,sum,avg);
      return avg;
    #else
      ESP_LOGW(TAG, "ADC not available, cannot check 12V level");
      return 2544.0;
    #endif // CONFIG_OVMS_COMP_ADC
}


float currentBatteryVoltageAdjusted(float calibratedVoltage, float calibratedLevel)
{
    float   low11v  = 1967.0; float low15v  = 2787.5;
    float   high11v = 2200.5; float high15v = 3098.0;

    float   lowascending  = ( low15v  - low11v  ) / ( 15.0 - 11.0 );
    float   highascending = ( high15v - high11v ) / ( 15.0 - 11.0 );

    float   lowlevel  = low11v  + ( (calibratedVoltage - 11.0) * lowascending );
    float   highlevel = high11v + ( (calibratedVoltage - 11.0) * highascending );

    float   ratiolowhigh = ( calibratedLevel - lowlevel) /  ( highlevel - lowlevel );

    float   ascending = lowascending + ( (highascending - lowascending) * ratiolowhigh ) ;

    float   currentlevel = (float)currentBatteryLevelAvg();

    float currentVoltage = calibratedVoltage - ( (calibratedLevel - currentlevel) / ascending ) ;

    return currentVoltage;
}


bool voltageIsInAcceptableRange(float currentVoltage,float alertVoltage)
{
    if( currentVoltage > alertVoltage )
    {
        return true;
    }

    if( currentVoltage < 5.0 )
    {
        ESP_LOGI(TAG, "Assuming USB powered, proceeding with boot");
        return true;
    }
    return false;
}


bool isBatteryInAcceptableRange(uint32_t packedvalue)
{
    float   wakeVoltage         = ((float)(packedvalue >> 16)) / 1000.0;
    float   calibrationFactor   = ((float)(packedvalue & 0xFFFF)) / 10.0;

    float   currentVoltage = currentBatteryVoltageAdjusted(wakeVoltage,wakeVoltage * calibrationFactor);

    ESP_LOGI(TAG, "isBatteryInAcceptableRange: wakeVoltage:%f calibration:%f currentVoltage:%f",wakeVoltage,calibrationFactor,currentVoltage);

    return voltageIsInAcceptableRange(currentVoltage,wakeVoltage);
}

//MARK: Packing/Unpacking

void setPackedWakeupVoltage(uint32_t packedVoltage)
{
    boot_data.battery12vinfopacked = packedVoltage;
    boot_data.crc = boot_data.calc_crc();
}


uint32_t getPackedWakeupVoltage()
{
    return boot_data.battery12vinfopacked;
}


uint32_t packWakeupVoltageAndCalibrationFactor(float wakeVoltage, float calibrationFactor)
{
    uint32_t    voltage = (uint32_t)(wakeVoltage * 1000.0);
    uint32_t    factor  = (uint32_t)(calibrationFactor * 10.0);
    uint32_t    packed  = (voltage << 16) | (factor & 0xFFFF);

    return packed;
}


uint32_t packedValueFromConfiguration()
{
    uint32_t packedValue = 0;

    bool powermanagementIsEnabled = MyConfig.GetParamValueBool("power", "enabled", false);
    bool lowpowerSleepIsEnabled   = MyConfig.GetParamValueInt("power", "12v_shutdown_delay", 0 ) > 0 ? true : false;

    ESP_LOGI(TAG, "packedValueFromConfiguration: pwrmgnt: %d lowpowersleep:%d",powermanagementIsEnabled,lowpowerSleepIsEnabled);

    if (powermanagementIsEnabled && lowpowerSleepIsEnabled)
    {
        float normvoltage       = MyConfig.GetParamValueFloat("vehicle", "12v.ref", 12.6);
        float calibrationfactor = MyConfig.GetParamValueFloat("system.adc","factor12v", 195.7);

        ESP_LOGI(TAG, "packedValueFromConfiguration: myconfig %f %f",normvoltage,calibrationfactor);

        packedValue = packWakeupVoltageAndCalibrationFactor(normvoltage,calibrationfactor);
    }

    ESP_LOGI(TAG,"packedValueFromConfiguration: %d %d\n",(packedValue >>16),(packedValue & 0xFFFF) );
    return packedValue;
}

//MARK: Sleeping

void sleepImmediatelyIfNeeded()
{
    uint32_t packedValue = packedValueFromConfiguration();

    ESP_LOGI(TAG,"sleepImmediatelyIfNeeded: %d %d\n",(packedValue >>16),(packedValue & 0xFFFF) );

    if(packedValue == 0) packedValue = getPackedWakeupVoltage();

    ESP_LOGI(TAG,"sleepImmediatelyIfNeeded: %d %d\n",(packedValue >>16),(packedValue & 0xFFFF) );

    if( packedValue && !isBatteryInAcceptableRange(packedValue) )
    {
      ESP_LOGE(TAG, "sleepImmediatelyIfNeeded: 12V level insufficient, re-entering deep sleep");
      sleepImmediately(packedValue);
    }
}


void sleepImmediately()
{
    ESP_LOGI(TAG,"sleepImmediately():");
    uint32_t packedValue = packedValueFromConfiguration();
    sleepImmediately(packedValue);
}


void sleepImmediately(uint32_t packedValue,uint32_t time)
{
    ESP_LOGI(TAG,"sleepImmediately(packedValue,time): %d %d %d\n",(packedValue >>16),(packedValue & 0xFFFF),time );

    setPackedWakeupVoltage(packedValue);
    ESP_LOGI(TAG,"sleepImmediately: entering deep sleep");
    esp_deep_sleep(1000000LL * time);
}
