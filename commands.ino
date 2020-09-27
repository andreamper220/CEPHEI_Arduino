void getAnalogInput(int pin, int argument, bool isIRsensor) 
{
  if (in_array(pin, sizeof(AI_PINS) / sizeof(AI_PINS[0]), AI_PINS)) {
    if (argument == 1) {
      analogReference(EXTERNAL);
    } else {
      analogReference(DEFAULT);
    }
    pinMode(pin, INPUT);
    int value = analogRead(pin);
    if (isIRsensor) {
      currentTemperatureIR = value;
    } else {
      sendValue(String(value));
    }
  } else {
    sendFailure(PINS_ERROR);
  }
}

void getDigitalInput(int pin, int argument)
{
  if (in_array(pin, sizeof(DI_PINS) / sizeof(DI_PINS[0]), DI_PINS)) {
    if (isI2CEnabled && is_I2C_pin(pin)) {
      sendFailure(I2C_ERROR);
    } else {
      pinMode(pin, INPUT);
      switch (digitalRead(pin)) {
        case HIGH:
          sendValue("1");
          break;
        case LOW:
          sendValue("0");
          break;
      }
    }
  } else {
    sendFailure(PINS_ERROR);
  }
}

void setOutput(int pin, int argument, bool toShowReply)
{ 
  if (in_array(pin, sizeof(DO_PINS) / sizeof(DO_PINS[0]), DO_PINS)) {
    if (argument == 0 || argument == 1) {
      if (isI2CEnabled && is_I2C_pin(pin)) {
        sendFailure(I2C_ERROR);
      } else {
        pinMode(pin, OUTPUT);
        switch (argument) {
          case 1:
            digitalWrite(pin, HIGH);
            break;
          case 0:
            digitalWrite(pin, LOW);
            break;
        }
        if (toShowReply) {
          sendSuccess();
        }
      }
    } else {
      sendFailure(RANGE_ERROR);
    }
  } else {
    sendFailure(PINS_ERROR);
  } 
}

void setOutputPWM(int pin, int argument, bool toShowReply)
{
  if (in_array(pin, sizeof(PWM_PINS) / sizeof(PWM_PINS[0]), PWM_PINS)) {
    if (argument >= 0 && argument <= 255) {
      if (isI2CEnabled && is_I2C_pin(pin)) {
        sendFailure(I2C_ERROR);
      } else {
        pinMode(pin, OUTPUT);
        analogWrite(pin, argument);
        if (toShowReply) {
          sendSuccess();
        }
      }
    } else {
      sendFailure(RANGE_ERROR);
    }
  } else {
    if (toShowReply) {
      sendFailure(PINS_ERROR);
    }
  }
}

void setLampPower(int pin, int argument) 
{
  if (argument >= 0 && argument <= 99) {
    if (!isDimmerEnabled || DIMMER_PIN != pin) {
      sendFailure(DIMMER_ERROR);
    } else {
      dimmer.setPower(argument);
      sendSuccess();
    }
  } else {
    sendFailure(RANGE_ERROR);
  }
}

void getServoAngle(int pin) 
{
  if (in_array(pin, sizeof(PWM_PINS) / sizeof(PWM_PINS[0]), PWM_PINS)) {
    servo.attach(pin);
    sendValue(String(map(servo.read(), 750, 2250, 0, 180)));
    servo.detach();
  } else {
    sendFailure(PINS_ERROR);
  }
}

void setOutputServo(int pin, int argument) 
{
  int angle = map(argument, 0, 180, 750, 2250);
  if (in_array(pin, sizeof(PWM_PINS) / sizeof(PWM_PINS[0]), PWM_PINS)) {
    if (argument >= 0 && argument <= 180) {
      if (isI2CEnabled && is_I2C_pin(pin)) {
        sendFailure(I2C_ERROR);
      } else {
        servo.attach(pin);
        servo.write(angle);
        delay(2000);
        if (round(servo.read()) == angle) {
          sendSuccess();
        } else {
          sendFailure(ANGLE_ERROR);
        }
        servo.detach();
      }
    } else {
      sendFailure(RANGE_ERROR);
    }
  } else {
    sendFailure(PINS_ERROR);
  }
}

void getLux(int argument)
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
    currentLux = value;
  } else {
    sendFailure(BH1750_ADDRESS_ERROR);
  }
}

void getTemp(int pin, int argument)
{
  if (isI2CEnabled && is_I2C_pin(pin)) {
    sendFailure(I2C_ERROR);
  } else {
    sensors.requestTemperatures();
    currentTemperature = sensors.getTempCByIndex(argument);
  }
}

void getCurrent(int argument)
{
  if (argument == 0) {
    currentCurrent_1 = INA.getBusMicroAmps(0) / 1000.0;
  } else if (argument == 1) {
    currentCurrent_2 = INA.getBusMicroAmps(1) / 1000.0;
  }
}

void getVoltage(int argument)
{
  if (argument == 0) {
    currentVoltage_1 = INA.getBusMilliVolts(0) / 1000.0;
  } else if (argument == 1) {
    currentVoltage_2 = INA.getBusMilliVolts(1) / 1000.0;
  }
}

void setCustomStamp(String argument)
{
  customStamp = argument;
  sendSuccess(); 
}

void getCustomStamp()
{
  sendValue(String(customStamp));
}
