# CEPHEI_SolarPanels

Arduino sketch for NI6001 and multiple peripheral devices interfacing 

## Protocol description

### Master Request:

`@CMD PIN ARG#`, where

|**CMD**|**PIN**|**ARG**|
|---|---|---|
|AI (analog read)|54-70|0/1 (DEFAULT or EXTERNAL analog reference)|
|DI (digital read)|0-53||
|DO (digital output)|0-53|0/1 (LOW or HIGH level)|
|PWM (PWM output)|2-9, 10-13, 44-45|0-255 (duty)|
|DIM (lamp dimmer output)|0-1, 3-70|0-100 (power)|
|SERV (servo output)|2-9, 10-13, 44-45|0-180 (angle)|
|LUX (light sensor BH1750 read)|IGNORED|0-1 (0x23 or 0x5C address)|
|TEMP (temperature sensor DS18B20 read)|pin, configured as ONE_WIRE|0-127 (address)|
|CFG (set config)|any|4X - according to PWM config table; <br>6 - enable I2C; <br>7 - set for temp. sensors (ONE_WIRE); <br>97 - show config  pin value; <br>98 - show non-default pins; <br>99 - clear pin value|

**PWM config table** (PIN / X):
|44, 45, 46|4, 13|2, 3, 5, 6, 7, 8, 11, 12|9, 10|
|---|---|---|---|
|1 - 31372.55 Hz|1 - 62500.00 Hz|1 - 31372.55 Hz|1 - 31372.55 Hz|
|2 - 3921.16 Hz|2 - 7812.50 Hz|2 - 3921.16 Hz|2 - 3921.16 Hz|
|3 - 490.20 Hz|3 - 976.56 Hz|3 - 490.20 Hz|3 - 980.39 Hz|
|4 - 122.55 Hz|4 - 244.14 Hz|4 - 122.55 Hz|4 - 490.20 Hz|
||5 - 61.04 Hz|5 - 30.64 Hz|5 - 245.10 Hz|
||||6 - 122.55 Hz|
||||7 - 30.64 Hz|

### Slave Reply:

* without data:

`@OK<CR>`

* with data:

`@OK REPLY XXX.XXX<CR>`

* with error:

`@ER X<CR>`

### ERROR types:

|**CODE**|**DESCRIPTION**|**SOLUTION**|
|---|---|---|
|0|commands amount error|decrease commands count|
|1|pins error|try to use another pin: maybe, you set AO instead of PWM etc.|
|2|BH1750 address error|try to set another address of lux sensor|
|3|I2C error|try to use another pin instead of I2C pins|
