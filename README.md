# CEPHEI_SolarPanels

Arduino sketch for NI6001 and multiple peripheral devices interfacing 

## Protocol description

**Master** Request:

`@CMD PIN ARG#`, where

**PIN** = pin number (0 - 69);

|**CMD**|**ARG**|
|---|---|
|AI (analog read)|0/1 (DEFAULT or EXTERNAL analog reference)|
|DI (digital read)||
|DO (digital output)|0/1 (LOW or HIGH level)|
|PWM (PWM output)|0-255 (duty)|
|SERV (servo output)|0-180 (angle)|
|LUX (light sensor BH1750 read)|0-1 (0x23 or 0x5C address)|
|TEMP (temperature sensor DS18B20 read)|0-127 (address)|
|CFG (set config)|4X - according to PWM config table; <br>6 - enable I2C; <br>7 - set for temp. sensors|

**PWM config table** (PIN / X):
|44, 45, 46|4, 13|2, 3, 5, 6, 7, 8, 11, 12|9, 10|
|---|---|---|---|
|1 - 31372.55 Гц|1 - 62500.00 Гц|1 - 31372.55 Гц|1 - 31372.55 Гц|
|2 - 3921.16 Гц|2 - 7812.50 Гц|2 - 3921.16 Гц|2 - 3921.16 Гц|
|3 - 490.20 Гц|3 - 976.56 Гц|3 - 490.20 Гц|3 - 980.39 Гц|
|4 - 122.55 Гц|4 - 244.14 Гц|4 - 122.55 Гц|4 - 490.20 Гц|
||5 - 61.04 Гц|5 - 30.64 Гц|5 - 245.10 Гц|
||||6 - 122.55 Гц|
||||7 - 30.64 Гц|
