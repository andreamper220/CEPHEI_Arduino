void readConfig() {
  for (int pin = 0; pin < 70; pin++) {
    byte function = eeprom_read_byte(pin + 1);
    setConfig(pin, function);
  }
}

void setConfig(int pin, byte function)
{
//  if (function == 10 || function == 11) {
//    int analogPin = pin;
//    switch(function) {
//      case 10:
//        analogReference(DEFAULT);
//      case 11:
//        analogReference(EXTERNAL);
//    }
//    pinMode(analogPin, INPUT);
//    eeprom_write_byte(0, 1);
//    eeprom_write_byte(pin + 1, function);
//  } else if (function == 2) {
//    pinMode(pin, INPUT);
//    eeprom_write_byte(0, 1);
//    eeprom_write_byte(pin + 1, function);
//  } else if (function == 3) {
//    pinMode(pin, OUTPUT);
//    switch (function % 30) {
//      case 1:
//        digitalWrite(pin, HIGH);
//        break;
//      case 0:
//        digitalWrite(pin, LOW);
//        break;
//    }
//    eeprom_write_byte(0, 1);
//    eeprom_write_byte(pin + 1, function);
//  } else 
if (function > 40 && function <= 47) {
    byte frequency = function % 40;                                                                                                     
    pinMode(pin, OUTPUT);
    switch (pin) {
      case 4:
      case 13:
        TCCR0B = TCCR0B & B11111000  | frequency;
        break;
      case 11:
      case 12:
        TCCR1B = TCCR1B & B11111000  | frequency;
        break;
      case 9:
      case 10:
        TCCR2B = TCCR2B & B11111000  | frequency;
        break;
      case 2:
      case 3:
      case 5:
        TCCR3B = TCCR3B & B11111000  | frequency;
        break;
      case 6:
      case 7:
      case 8:
        TCCR4B = TCCR4B & B11111000  | frequency;
        break;
      case 44:
      case 45:
      case 46:
        TCCR5B = TCCR5B & B11111000  | frequency;
        break;
    }
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 5) {
    servo.attach(pin);
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 6) {
    isI2CEnabled = true;
  } else if (function == 7) {
    ONE_WIRE_BUS = pin;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 8) {
    DIMMER_PIN = pin;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(3, 200);
    eeprom_write_byte(pin + 1, function);
    isDimmerEnabled = true;  
  } else if (function == 97) {
    sendValue(String(eeprom_read_byte(pin + 1)));
  } else if (function == 98) {
    for (int pin = 0; pin < 70; pin++) {
      byte function = eeprom_read_byte(pin + 1);
      if (function != 255) {
        sendValue(String(pin));
      }
    }
  } else if (function == 99) {
    eeprom_write_byte(pin + 1, 255);
    readConfig();
  } else if (function == 200) {
    NOT_USED_PINS[pin] = true;
    eeprom_write_byte(pin + 1, function);
  }
}
