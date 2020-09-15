# CEPHEI_Arduino

Arduino sketch for multiple peripheral devices interfacing from PC by Serial communication.

It allows to:

* read values from analog sensors (e.g. IR-thermometers with 0-5V interface)
* control over 54 different devices by digital output signals (e.g. relays, buttons, diodes etc.)
* read digital TTL-signals (0-5V) from 54 sources
* control power of 14 different devices by PWM-signals (e.g. heater, cooler, diodes)
* change PWM duty of all the PWM-pins
* control bulb lump power with software-controlled dimmer by implementing zero-crossing handling (based on hardware interrupts of ATmega2560)
* interface with 1-Wire sensors (e.g. DS18B20 temperature sensor)
* interface with i2c sensors (e.g. BH1750 light sensor)

![alt text](https://github.com/andreamper220/CEPHEI_Arduino/blob/master/MEGA_wiring.png?raw=true)


## Protocol description

### Master Request:

`@CMD PIN ARG#<CR><LF>`, where

|**CMD**|**PIN**|**ARG**|
|---|---|---|
|AI (analog read)|54-70|0/1 (DEFAULT or EXTERNAL analog reference)|
|DI (digital read)|0-53||
|DO (digital output)|0-53|0/1 (LOW or HIGH level)|
|PWM (PWM output)|2-9, 10-13, 44-45|0-255 (duty)|
|DIM (lamp dimmer output)|0-1, 3-70|0-100 (power)|
|SERV (servo output / angle value)|2-9, 10-13, 44-45|0-180 (angle) / ? (for value getting)|
|LUX (light sensor BH1750 read)|IGNORED|0-1 (0x23 or 0x5C address)|
|TEMP (temperature sensor DS18B20 read)|pin, configured as ONE_WIRE|0-127 (address)|
|TIME (custom stamp for arduino reset checking)|IGNORED|CUSTOM_STRING / ?|
|CFG (set config)|any|4X - according to PWM config table; <br>6 - enable I2C; <br>7 - set for temp. sensors (ONE_WIRE); <br>8 - enable dimmer (lock 2nd pin)<br>97 - show config  pin value; <br>98 - show non-default pins; <br>99 - clear pin value|

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

* success without data:

`@OK<CR><LF>`

* success with data:

`@OK REPLY XXX.XXX<CR><LF>`

* with error:

`@ER X<CR><LF>`

### ERROR types:

|**CODE**|**DESCRIPTION**|**SOLUTION**|
|---|---|---|
|0|parameters amount error|decrease parameters count (divided by spaces)|
|1|pins error|try to use another pin: maybe, you set AO instead of PWM etc.|
|2|BH1750 address error|try to set another address of lux sensor|
|3|I2C error|try to use another pin instead of I2C pins|
|4|dimmer error|try not to use 2nd pin|
|5|not used pins error|this pin is marked as "non used" by CFG command|
|6|angle error|try to resend servo command|
|7|range error|enter correct argument range|
|8|serial error|incorrect command syntax|
