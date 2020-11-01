/*
    Sketch is developed only for *Arduino MEGA 2560*!
    
    Arduino sketch for multiple peripheral devices interfacing from PC by Serial communication.
    
    It allows to:
    
    * read values from analog sensors (e.g. IR-thermometers with 0-5V interface)
    * control over 54 different devices by digital output signals (e.g. relays, buttons, diodes etc.)
    * read digital TTL-signals (0-5V) from 54 sources
    * control power of 14 different devices by PWM-signals (e.g. heater, cooler, diodes)
    * change PWM duty of all the PWM-pins
    * control bulb lump power with software-controlled dimmer by implementing **zero-crossing** handling (based on hardware interrupts of ATmega2560) with [rbdDimmer-library](https://github.com/RobotDynOfficial/RBDDimmer)
    * interface with 1-Wire sensors (e.g. DS18B20 temperature sensor)
    * interface with i2c sensors (e.g. BH1750 light sensor)
     
    Current-Voltage measurements are executed by 2xINA226 and [INA-library](https://github.com/SV-Zanshin/INA)

    Master Request:
    `@CMD PIN ARG#<CR><LF>`, where:
    
    |================================================================================================================|
    |CMD                                                |PIN              |ARG                                       |
    |===================================================|=================|==========================================|
    |AI (analog read)                                   |54-70            |0/1 (DEFAULT or EXTERNAL analog reference)|
    |---------------------------------------------------|-----------------|------------------------------------------|
    |DI (digital read)                                  |0-53             |IGNORED                                   |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |DO (digital output)                                |0-53             |0/1 (LOW or HIGH level)                   |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |PWM (PWM output)                                   |2-9, 10-13, 44-45|0-255 (duty)                              |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |DIM (lamp dimmer output)                           |0-1, 3-70        |0-100 (power)                             |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |SERV (servo output w/o angle holding / angle value)|2-9, 10-13, 44-45|0-180 (angle) / '?' (for value getting)   |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |SERVH (servo output with angle holding)            |2-9, 10-13, 44-45|0-180 (angle)                             |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |LUX (light sensor BH1750 read)                     |IGNORED          |0-1 (0x23 or 0x5C address)                |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |TEMP (temperature sensor DS18B20 read)             |ONE_WIRE pin     |0-127 (address)                           |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |TIME (custom stamp for arduino reset checking)     |IGNORED          |CUSTOM_STRING / '?'                       |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |CFG (set config)                                   |any              |4X - according to PWM config table;       |
    |                                                   |                 |6 - enable I2C;                           |
    |                                                   |                 |7 - set for temp. sensors (ONE_WIRE);     |
    |                                                   |                 |8 - enable dimmer (lock 2nd pin);         |
    |                                                   |                 |97 - show config  pin value;              |
    |                                                   |                 |98 - show non-default pins;               |
    |                                                   |                 |99 - clear pin value                      |
    |---------------------------------------------------|-----------------|------------------------------------------|
    |WDOG (shows all watchdog variables)                |IGNORED          |IGNORED                                   |
    |----------------------------------------------------------------------------------------------------------------|
    |RESET (soft Arduino reset)                         |IGNORED          |IGNORED                                   |
    |================================================================================================================|
 */

#include <ServoTimer2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RBDdimmer.h>
#include <Wire.h>
#include <INA.h>
#include <avr/eeprom.h>
#include <limits.h>

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

#define WDOG_FREQUENCY_MS     300        
#define CRIT_REPLY_FREQUENCY  300000     

int errorCount = 0;

String customStamp = "";

byte buff[2];
int ONE_WIRE_BUS = 8;
int DIMMER_PIN = 6;
int IR_SENSOR_PIN = 63;
int GLOBAL_ON_PIN = 64;
int SERVOH_PIN = 3;

dimmerLamp dimmer(DIMMER_PIN);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
INA_Class INA;

bool isCR = false;
bool isSerial = false;
bool isSuccess = false;
bool isFailure = false;
bool isWatchdogEnabled = false;

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
int PWM_PINS[6] = {6, 7, 8, 9, 11, 12};
int COOLER_PINS[2] = {11, 12};
String dataString;
String datas[3][1];
String data[3];

float currentTemperature;
uint16_t currentLux;
int currentTemperatureIR;
int globalOn;
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
unsigned long timeDifferenceWatchdog = 0;
unsigned long lastReplyTime = 0;
unsigned long timeDifferenceReply = 0;

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
  servoHold.attach(SERVOH_PIN);
  
  byte isConfigured = eeprom_read_byte(100);
  
  if (isConfigured == 1) {
    readConfig();
  }
  // for Serial
  eeprom_write_byte(101, 200);
  eeprom_write_byte(102, 200);

  for (byte i = 54; i <= 70; i++) {
    AI_PINS[i - 54] = i;
    DO_PINS[i] = i;
    if (i != IR_SENSOR_PIN) {
      setOutput(i, LOW, false);
    }
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

void(* resetFunc)(void) = 0;

void loop() 
{
  isSerial = false;
  isSuccess = false;
  isFailure = false;

  if (isWatchdogEnabled) {
    timeDifferenceReply = millis() - lastReplyTime;
    if (timeDifferenceReply > CRIT_REPLY_FREQUENCY || (timeDifferenceReply < 0 && (ULONG_MAX - lastReplyTime) + millis() > CRIT_REPLY_FREQUENCY)) {
      resetFunc();
    }
  }

  timeDifferenceWatchdog = millis() - start;
  /** WATCHDOG */
  if (timeDifferenceWatchdog > WDOG_FREQUENCY_MS || timeDifferenceWatchdog <= 0) {
    getLux(1);
    getAnalogInput(IR_SENSOR_PIN, 0);
    getAnalogInput(GLOBAL_ON_PIN, 0);
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
              getAnalogInput(pin, argument);
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
                sendValue(String(currentCurrent_1, DEC));
              } else if (argument == 1) {
                sendValue(String(currentCurrent_2, DEC));
              }
            } else if (command == "VOLT") {
              if (argument == 0) {
                sendValue(String(currentVoltage_1, DEC));
              } else if (argument == 1) {
                sendValue(String(currentVoltage_2, DEC));
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
                " VOLT1_" + String(currentVoltage_1, DEC) + " VOLTSNT1_" + String(currentShuntVoltage_1, DEC) + " CURR1_" + String(currentCurrent_1, DEC) + " POWR1_" + String(currentPower_1, DEC) +
                " VOLT2_" + String(currentVoltage_2, DEC) + " VOLTSNT2_" + String(currentShuntVoltage_2, DEC) + " CURR2_" + String(currentCurrent_2, DEC) + " POWR2_" + String(currentPower_2, DEC));
              isSuccess = true;
              setWatchdogEnabled();
            } else if (command == "RESET") {
              resetFunc();
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
