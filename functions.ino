/** Serial functions */
bool isValidRequest(String serialData)
{
  char dataLength = serialData.length();
  if (serialData.charAt(0) != 64 
    || serialData.lastIndexOf(10) != dataLength - 1
    || serialData.lastIndexOf(13) != dataLength - 2
    || serialData.lastIndexOf('#') != dataLength - 3
  ) {
    return false;
  } else {
    return true;
  }
}

void sendSuccess()
{
  Serial.println("@OK");
  isSuccess = true;
}

void sendFailure(int errorType)
{
  if (!isFailure) {
    errorCount++;
    Serial.print("@ER ");
    Serial.println(String(errorType));
  }
  isFailure = true;
}

void sendValue(String value)
{
  Serial.print("@OK REPLY ");
  Serial.println(value);
  isSuccess = true;
}

void sendBufferSize()
{
  Serial.print("buffer size = ");
  Serial.println(String(Serial.available()));
}

/** functions */
bool is_I2C_pin(int pin) 
{
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

/** SERV */
void checkServo(int pin)
{
  if (in_array(pin, sizeof(SERV_PINS) / sizeof(SERV_PINS[0]), SERV_PINS)) {
    servoIndex = find_key_by_value(pin, sizeof(SERV_PINS) / sizeof(SERV_PINS[0]), SERV_PINS);
    SERV_PINS[servoIndex] = 255;
    servos[servoIndex].detach();
  }
}

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
