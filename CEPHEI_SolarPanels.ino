#include <ServoTimer2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RBDdimmer.h>
#include <Wire.h>
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
int ONE_WIRE_BUS = 12;
int DIMMER_PIN = 6;

dimmerLamp dimmer(DIMMER_PIN);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

bool isCR = false;

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
int PWM_PINS[9] = {3, 4, 5, 6, 7, 8, 11, 12, 13};
String dataString;
String datas[3][1];
String data[3];

unsigned long start = 0;

void setup() 
{  
  Wire.begin();
  dimmer.begin(NORMAL_MODE, ON);
  Serial.begin(115200);
  byte isConfigured = eeprom_read_byte(0);
  
  if (isConfigured == 1) {
    readConfig();
  }
  // for Serial
  eeprom_write_byte(1, 200);
  eeprom_write_byte(2, 200);

  for (byte i = 54; i <= 70; i++) {
    AI_PINS[i - 54] = i;
    DO_PINS[i] = i;
  }
  for (byte i = 0; i <= 53; i++) {
    if (i != DIMMER_PIN) {
      DI_PINS[i] = i;
      DO_PINS[i] = i;
      setOutputPWM(i, 0, false);
    }
  }
  dimmer.setPower(2);
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

  if (Serial.available() > 0) {  
    if (Serial.read() == 64 && !isReadable) {
      isReadable = true;
    } else {
      sendFailure(SERIAL_ERROR);
    }

    if (isReadable) {
      dataString = Serial.readStringUntil('#');
      if (dataString != "") {
        isReadable = false;
      }
      char* dataCharStar = dataString.c_str();
      char* dataChar = strtok(dataCharStar, " ");
      
      while (dataChar != NULL) {
        if (index > 3) {
          sendFailure(COMMANDS_COUNT_ERROR);
          isDataFinished = false;
        } else {
          isDataFinished = true;
        }
        byte multiplier = (index / 3 > 1) ? index / 3 : 1;
        datas[index % (3 * multiplier)][index / 3] = dataChar;
        dataChar = strtok(NULL, " ");
        index++;
      }
      if (index < 4) {
        sendFailure(SERIAL_ERROR);
      }
      index = 0;
    }

    if (isDataFinished) {
      if (Serial.read() == 13) {
        isCR = true;
      } else if (Serial.read() == 10) {
        if (isCR) {
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
              } else if (command == "LUX") {
                getLux(pin, argument);
              } else if (command == "TEMP") {
                getTemp(pin, argument);
              } else if (command == "TIME") {
                if (data[2] == "?") {
                  getCustomStamp();
                } else {
                  setCustomStamp(data[2]);
                }
              } else if (command == "CFG") {
                byte function = lowByte(argument);
                setConfig(pin, function);
                sendSuccess();
              }
              datas[0][0] = "";
              datas[1][0] = "";
              datas[2][0] = "";
              isCR = false;
              isDataFinished = false;
            }
          }
        } else {
          sendFailure(SERIAL_ERROR);           
        }
      } else {
        sendFailure(SERIAL_ERROR);
      }
    }

//    sendBufferSize();
  } 
}
