#include <ServoTimer2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RBDdimmer.h>
#include <Wire.h>
#include <INA.h>
#include <avr/eeprom.h>

#define TEMP_SENSORS_COUNT    1
#define CRITICAL_TEMPERATURE  70

/** ERROR types */
#define COMMANDS_COUNT_ERROR  0
#define PINS_ERROR            1
#define BH1750_ADDRESS_ERROR  2
#define I2C_ERROR             3
#define DIMMER_ERROR          4
#define NOT_USED_PINS_ERROR   5
#define ANGLE_ERROR           6
#define RANGE_ERROR           7
#define SERIAL_ERROR          8

int errorCount = 0;

String customStamp = "";

byte buff[2];
int ONE_WIRE_BUS = 8;
int DIMMER_PIN = 6;
int IR_SENSOR_PIN = 62;
int SERVOH_PIN = 3;

dimmerLamp dimmer(DIMMER_PIN);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
INA_Class INA;

bool isCR = false;
bool isSerial = false;
bool isSuccess = false;
bool isFailure = false;

ServoTimer2 servoHold;
ServoTimer2 servo;
byte index = 0;
byte dataIndex = 0;
bool isReadable = false;
bool isDataFinished = false;
bool isI2CEnabled = false;
bool isDimmerEnabled = false;
bool NOT_USED_PINS[70];
int AI_PINS[16];
int DI_PINS[54];
int DO_PINS[70];
int PWM_PINS[10] = {3, 4, 5, 6, 7, 8, 9, 11, 12, 13};
int SERV_PINS[14] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45};
int COOLER_PINS[2] = {11, 12};
String dataString;
String datas[3][1];
String data[3];

double currentTemperature;
uint16_t currentLux;
int currentTemperatureIR;
int currentLampPower;
float currentVoltage_1;
float currentShuntVoltage_1;
float currentCurrent_1;
float currentPower_1;
float currentVoltage_2;
float currentShuntVoltage_2;
float currentCurrent_2;
float currentPower_2;

unsigned long start = 0;

void setup() 
{  
  Wire.begin();
  dimmer.begin(NORMAL_MODE, ON);
  Serial.begin(115200);
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  INA.begin(1,100000);
  INA.setAveraging(128);                                                   
  INA.setBusConversion(8500);                                                
  INA.setShuntConversion(8500);                                               
  INA.setMode(INA_MODE_CONTINUOUS_BOTH);
  
  byte isConfigured = eeprom_read_byte(100);
  
  if (isConfigured == 1) {
    readConfig();
  }
  // for Serial
  eeprom_write_byte(101, 200);
  eeprom_write_byte(102, 200);

  for (byte i = 0; i <= 13; i++) {
    SERV_PINS[i] = 255;
  }
  for (byte i = 54; i <= 70; i++) {
    AI_PINS[i - 54] = i;
    DO_PINS[i] = i;
  }
  for (byte i = 0; i <= 53; i++) {
    DO_PINS[i] = i;
    if (i != DIMMER_PIN) {
      DI_PINS[i] = i;
      setOutputPWM(i, 0, false);
    }
  }
  dimmer.setPower(2);
}

void loop() 
{
  isSerial = false;
  isSuccess = false;
  isFailure = false;

  /** WATCHDOG */
  if (millis() - start > 1000) {
    getLux(1);
    getAnalogInput(IR_SENSOR_PIN, 0, true);
    currentLampPower = dimmer.getPower();
    getCurrent(0);
    getCurrent(1);
    getShuntVoltage(0);
    getShuntVoltage(1);
    getVoltage(0);
    getVoltage(1);
    getPower(0);
    getPower(1);
    for (byte i = 0; i < TEMP_SENSORS_COUNT; i++) {
      getTemp(ONE_WIRE_BUS, i);
      if (currentTemperature > CRITICAL_TEMPERATURE || currentTemperature < 0.0) {
        for (byte j = 0; j < (sizeof(PWM_PINS) / sizeof(PWM_PINS[0])); j++) {
          if (!in_array(PWM_PINS[j], sizeof(COOLER_PINS) / sizeof(COOLER_PINS[0]), COOLER_PINS) && PWM_PINS[j] != DIMMER_PIN) {
            setOutputPWM(PWM_PINS[j], 0, false);
          }
        }
        dimmer.setPower(2);
      }
    }
    
    start = millis();
  }
  /** ------------- */

  if (Serial.available() > 0) {
    isSerial = true; 
    
    if (Serial.read() == 64 && !isReadable) {
      isReadable = true;
    } else {
      sendFailure(SERIAL_ERROR);
    }

    if (isReadable) {
      dataString = Serial.readStringUntil('#');
      if (dataString != "" && dataString.lastIndexOf(32) != dataString.length() - 1 && dataString.indexOf(32) != 0) {
        isReadable = false;
      } else {
        sendFailure(SERIAL_ERROR);
      }
      char* dataCharStar = dataString.c_str();
      char* dataChar = strtok(dataCharStar, " ");
      
      while (dataChar != NULL) {
        byte multiplier = (index / 3 > 1) ? index / 3 : 1;
        datas[index % (3 * multiplier)][index / 3] = dataChar;
        dataChar = strtok(NULL, " ");
        index++;
      }
        
      if (index != 3) {
        sendFailure(COMMANDS_COUNT_ERROR);
        isDataFinished = false;
      } else {
        isDataFinished = true;
      }
      
      index = 0;
    }

    if (isDataFinished && !isFailure) {
      if (Serial.read() == 13) {
        isCR = true;
      }
      if (Serial.read() == 10 && isCR) {
        datas[2][0].trim();
        for (byte j = 0; j < 3; j++) {
          data[j] = datas[j][0];  
        }
        if (data[0] != 0) {
          int pin = data[1].toInt();
          int argument = data[2].toInt();
          String command = data[0];
          if (NOT_USED_PINS[pin] == true) {
            sendFailure(NOT_USED_PINS_ERROR);
          } else {
            if (command == "AI") {
              getAnalogInput(pin, argument, false);
            } else if (command == "DI") {
              getDigitalInput(pin, argument);
            } else if (command == "DO") {
              setOutput(pin, argument, true);
            } else if (command == "PWM") {
              setOutputPWM(pin, argument, true); 
            } else if (command == "DIM") {
              setLampPower(pin, argument);  
            } else if (command == "SERV") {
              if (data[2] == "?") {
                getServoAngle(pin);
              } else {
                setOutputServo(pin, argument);
              }
            } else if (command == "SERVH") {
              setOutputServoHold(pin, argument);  
            } else if (command == "LUX") {
              sendValue(String(currentLux));
            } else if (command == "TEMP") {
              sendValue(String(currentTemperature, DEC));
            } else if (command == "CURR") {
              if (argument == 0) {
                sendValue(String(currentCurrent_1));
              } else if (argument == 1) {
                sendValue(String(currentCurrent_2));
              }
            } else if (command == "VOLT") {
              if (argument == 0) {
                sendValue(String(currentVoltage_1));
              } else if (argument == 1) {
                sendValue(String(currentVoltage_2));
              }
            } else if (command == "CFG") {
              byte function = lowByte(argument);
              setConfig(pin, function);
              sendSuccess();
            } else if (command == "TIME") {
              if (data[2] == "?") {
                getCustomStamp();
              } else {
                setCustomStamp(data[2]);
              }
            } else if (command == "WDOG") {
              Serial.println("@OK TEMP_" + String(currentTemperature, DEC) + " LUX_" + String(currentLux) +
                " IR_" + String(currentTemperatureIR) + " DIMMER_" + String(dimmer.getPower()) + 
                " VOLT1_" + String(currentVoltage_1) + " VOLTSNT1_" + String(currentShuntVoltage_1) + " CURR1_" + String(currentCurrent_1) + " POWR1_" + String(currentPower_1) +
                " VOLT2_" + String(currentVoltage_2) + " VOLTSNT2_" + String(currentShuntVoltage_2) + " CURR2_" + String(currentCurrent_2) + " POWR2_" + String(currentPower_2));
              isSuccess = true;
            }
            datas[0][0] = "";
            datas[1][0] = "";
            datas[2][0] = "";
            isCR = false;
            isDataFinished = false;
          }
        }
      }
    }

//    sendBufferSize();
    while (Serial.available()) { Serial.read(); delay(5); }
  }

  if (isSerial && !isSuccess) {
    sendFailure(SERIAL_ERROR);
  }
}
