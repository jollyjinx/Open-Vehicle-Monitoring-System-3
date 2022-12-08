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

#ifndef __OVMS_12VBATTERY_H__
#define __OVMS_12VBATTERY_H__

float currentBatteryLevelAvg();
float currentBatteryVoltageAdjusted(float calibratedVoltage, float calibratedLevel);
bool voltageIsInAcceptableRange(float currentVoltage,float alertVoltage);
bool isBatteryInAcceptableRange(uint32_t packedvalue);

uint32_t packedValueFromConfiguration();

void sleepImmediatelyIfNeeded();
void sleepImmediately();
void sleepImmediately(uint32_t packedvalue,uint32_t howlong = 30);

#endif //#ifndef __OVMS_12VBATTERY_H__
