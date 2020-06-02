#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <avr/eeprom.h>

#define TEMP_SENSORS_COUNT    1
#define CRITICAL_TEMPERATURE  70

byte buff[2];
int ONE_WIRE_BUS = 2;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Servo servos[14];
byte servoIndex = 0;
byte index = 0;
byte dataIndex = 0;
bool isReadable = false;
bool isI2CEnabled = false;
int AI_PINS[16];
int DI_PINS[54];
int DO_PINS[54];
int PWM_PINS[14] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45};
int SERV_PINS[14] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45};
String dataString;
String datas[3][5];
String data[3];

unsigned long start;

void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  byte isConfigured = eeprom_read_byte(0);

  for (byte i = 0; i <= 13; i++) {
      SERV_PINS[i] = 255;
    }
  for (byte i = 54; i <= 70; i++) {
    AI_PINS[i - 54] = i;
  }
  for (byte i = 0; i <= 53; i++) {
    DI_PINS[i] = i;
    DO_PINS[i] = i;
  }
  
  if (isConfigured == 1) {
    readConfig();
  }
}

void loop() 
{
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  sensors.requestTemperatures();
  
  for (byte i = 0; i < TEMP_SENSORS_COUNT; i++) {
    float temperature = sensors.getTempCByIndex(i);
    if (temperature > CRITICAL_TEMPERATURE || temperature < 0.0) {
      for (byte j = 0; j < (sizeof(PWM_PINS) / sizeof(PWM_PINS[0])); j++) {
        setOutputPWM(PWM_PINS[j], 0, false);
      }
    }
  }
    
  if (Serial.available() > 0) {  //если есть доступные данные
    if (Serial.read() == 64 && !isReadable) {
      isReadable = true;
    }
    // считываем команду
    if (isReadable) {
      dataString = Serial.readStringUntil('#');
      if (dataString != "") {
        isReadable = false;
      }
      char* dataCharStar = dataString.c_str();
      char* dataChar = strtok(dataCharStar, " ");
      while (dataChar != NULL) {
        byte multiplier = (index / 3 > 1) ? index / 3 : 1;
        datas[index % (3 * multiplier)][index / 3] = dataChar;
        dataChar = strtok(NULL, " ");
        index++;
      }
      index = 0;
    }
  } else {
      for (byte i = 0; i < 5; i++) {
        for (byte j = 0; j < 3; j++) {
          data[j] = datas[j][i];  
        }
        if (data[0] == 0) {
          break;
        }
        
        int pin = data[1].toInt();
        int argument = data[2].toInt();
        String command = data[0];
        if (command == "AI") {
          getAnalogInput(pin, argument);
        } else if (command == "DI") {
          getDigitalInput(pin, argument);
        } else if (command == "DO") {
          setOutput(pin, argument);
        } else if (command == "PWM") {
          setOutputPWM(pin, argument, true);
        } else if (command == "SERV") {
          setOutputServo(pin, argument);
        } else if (command == "LUX") {
          getLux(pin, argument);
        } else if (command == "TEMP") {
          getTemp(pin, argument);
        } else if (command == "CFG") {
          byte function = lowByte(argument);
          setConfig(pin, function);
          Serial.println("@OK<CR>");
        }
        datas[0][i] = "";
        datas[1][i] = "";
        datas[2][i] = "";
      }
  }
}

/**
* Перезапись портов в соответствии с конфигом в EEPROM
* 55-70 - аналоговые/цифровые
*/
void readConfig() {
  for (int pin = 0; pin < 70; pin++) {
    byte function = eeprom_read_byte(pin + 1);
    setConfig(pin, function);
  }
}

void setConfig(int pin, byte function)
{
  if (function == 10 || function == 11) {
    int analogPin = pin;
//    AI_PINS[AIIndex] = analogPin;
    switch(function) {
      case 10:
        analogReference(DEFAULT);
      case 11:
        analogReference(EXTERNAL);
    }
    pinMode(analogPin, INPUT);
//    AIIndex++;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 2) {
//    DI_PINS[DIIndex] = pin;
    pinMode(pin, INPUT);
//    DIIndex++;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 3) {
//    DO_PINS[DOIndex] = pin;
    pinMode(pin, OUTPUT);
    switch (function % 30) {
      case 1:
        digitalWrite(pin, HIGH);
        break;
      case 0:
        digitalWrite(pin, LOW);
        break;
    }
//    DOIndex++;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function > 40 && function <= 47) {
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
//    PWM_PINS[PWMIndex] = pin;
//    PWMIndex++;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 5) {
    SERV_PINS[servoIndex] = pin;
    servos[servoIndex].attach(pin);
    servoIndex++;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 6) {
    isI2CEnabled = true;
  } else if (function == 7) {
    ONE_WIRE_BUS = pin;
    eeprom_write_byte(0, 1);
    eeprom_write_byte(pin + 1, function);
  } else if (function == 97) {
    Serial.println("@OK REPLY " + String(eeprom_read_byte(pin + 1)));  
  } else if (function == 98) {
    for (int pin = 0; pin < 70; pin++) {
      byte function = eeprom_read_byte(pin + 1);
      if (function != 255) {
        Serial.println(pin);
      }
    }
  } else if (function == 99) {
    eeprom_write_byte(pin + 1, 255);
    readConfig();
  }
}

/** commands */
void getAnalogInput(int pin, int argument) 
{
  if (in_array(pin, sizeof(AI_PINS) / sizeof(AI_PINS[0]), AI_PINS)) {
    if (argument == 1) {
      analogReference(EXTERNAL);
    } else {
      analogReference(DEFAULT);
    }
    pinMode(pin, INPUT);
    int value = analogRead(pin);
    Serial.println("@OK REPLY " + String(value) + "<CR>");
  } else {
    Serial.println("@ER 1<CR>");
  }
}

void getDigitalInput(int pin, int argument)
{
  if (in_array(pin, sizeof(DI_PINS) / sizeof(DI_PINS[0]), DI_PINS)) {
    if (isI2CEnabled && is_I2C_pin(pin)) {
      Serial.println("@ER 3<CR>");
    } else {
      pinMode(pin, INPUT);
      switch (digitalRead(pin)) {
        case HIGH:
          Serial.println("@OK REPLY 1<CR>");
          break;
        case LOW:
          Serial.println("@OK REPLY 0<CR>");
          break;
      }
    }
  } else {
    Serial.println("@ER 1<CR>");
  }
}

void setOutput(int pin, int argument)
{ 
  if (in_array(pin, sizeof(DO_PINS) / sizeof(DO_PINS[0]), DO_PINS)) {
    if (isI2CEnabled && is_I2C_pin(pin)) {
      Serial.println("@ER 3<CR>");
    } else {
      checkServo(pin);
      pinMode(pin, OUTPUT);
      switch (argument) {
        case 1:
          digitalWrite(pin, HIGH);
          break;
        case 0:
          digitalWrite(pin, LOW);
          break;
      }
      Serial.println("@OK<CR>");
    }
  } else {
    Serial.println("@ER 1<CR>");
  } 
}

void setOutputPWM(int pin, int argument, bool toShowReply)
{
  if (in_array(pin, sizeof(PWM_PINS) / sizeof(PWM_PINS[0]), PWM_PINS)) {
    if (isI2CEnabled && is_I2C_pin(pin)) {
      Serial.println("@ER 3<CR>");
    } else {
      checkServo(pin);
      pinMode(pin, OUTPUT);
      analogWrite(pin, argument);
      if (toShowReply) {
        Serial.println("@OK<CR>");
      }
    }
  } else {
    if (toShowReply) {
      Serial.println("@ER 1<CR>");
    }
  }
}

void setOutputServo(int pin, int argument) 
{
  if (in_array(pin, sizeof(PWM_PINS) / sizeof(PWM_PINS[0]), PWM_PINS)) {
    if (isI2CEnabled && is_I2C_pin(pin)) {
      Serial.println("@ER 3<CR>");
    } else {
      int index = find_key_by_value(pin, sizeof(SERV_PINS) / sizeof(SERV_PINS[0]), SERV_PINS);
      if (index != 255) {
        servos[index].write(argument);
      } else {
        SERV_PINS[servoIndex] = pin;
        servos[servoIndex].attach(pin);
        servos[servoIndex].write(argument);
        servoIndex++;
      }
      Serial.println("@OK<CR>");
    }
  } else {
    Serial.println("@ER 1<CR>");
  }
}

void getLux(int pin, int argument)
{
  int i;
  int BH1750address;
  uint16_t value=0;
  if (argument == 0) {
    BH1750address = 0x23; //GND
  } else {
    BH1750address = 0x5C; //3.3V
  }  
  BH1750_Init(BH1750address);
  delay(200);
 
  if(2==BH1750_Read(BH1750address))
  {
    value=((buff[0]<<8)|buff[1])/1.2;
    
    Serial.println("@OK REPLY " + String(value) + "<CR>");
  } else {
    Serial.println("@ER 2<CR>");
  }
}

void getTemp(int pin, int argument)
{
  if (isI2CEnabled && is_I2C_pin(pin)) {
      Serial.println("@ER 3<CR>");
  } else {
    OneWire oneWire(pin);
    DallasTemperature sensors(&oneWire);
  
    sensors.requestTemperatures();
    String sensor = String(sensors.getTempCByIndex(argument),DEC);
    Serial.println("@OK REPLY " + sensor + "<CR>");
  }
}
/** ----- */

/** LUX */
int BH1750_Read(int address) 
{
  int i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) 
  {
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();  
  return i;
}
 
void BH1750_Init(int address) 
{
  Wire.beginTransmission(address);
  Wire.write(0x10);
  Wire.endTransmission();
}
/** ----- */

void checkServo(int pin)
{
  if (in_array(pin, sizeof(SERV_PINS) / sizeof(SERV_PINS[0]), SERV_PINS)) {
    servoIndex = find_key_by_value(pin, sizeof(SERV_PINS) / sizeof(SERV_PINS[0]), SERV_PINS);
    SERV_PINS[servoIndex] = 255;
    servos[servoIndex].detach();
  }
}

/** functions */
bool is_I2C_pin(int pin) {
  int I2C_PINS[2] = {20, 21};
  if (in_array(pin, sizeof(I2C_PINS) / sizeof(I2C_PINS[0]), I2C_PINS)) {
    return true;
  } else {
    return false;
  }
}

int find_key_by_value(int val, int arr_size, int *arr) 
{
  for (int i = 0; i < arr_size; i++) {
    if (val == arr[i]) {
      return i;
    }
  }

  return 255;
}

bool in_array(int val, int arr_size, int *arr)
{
    byte i;
    for(i = 0; i < arr_size; i++)
    {
        if(arr[i] == val)
            return true;
    }
    return false;
}
/** ----- */
