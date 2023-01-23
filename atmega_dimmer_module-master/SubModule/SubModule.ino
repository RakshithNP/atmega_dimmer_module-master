#include <Wire.h>
#include <EEPROM.h>

#define TRIAC_PIN         6         // triac trigger pin 
#define ZCD_DETECT_PIN    0         // zcd input as interrupt
#define CURRENT_PIN       A1        // current sensor input
#define RELAY_PIN         3         // relay pin

#define Slave_Addres      0x30      // i2c slave address

#define MAX_DIMVALUE      125       // absolute max limit of dim level
#define MIN_DIMVALUE      10        // absolute min limit of dim level

#define DEVICE_ADDRESS    10        // eeprom address of device control
#define DIM_ADDRESS       11        // eeprom address of dim level
#define DIM_MAX_ADDRESS   12        // eeprom address of dim max 
#define DIM_MIN_ADDRESS   13        // eeprom address of dim min


volatile int dimTime = 0;                                 // dim time calculated by (75 * (dim_level))
volatile byte dimValue = 0, dimValue_temp = 0;            // dim level var
volatile byte dimMax = 0, dimMax_temp = 0;                // max dim level var
volatile byte dimMin = 0, dimMin_temp = 0;                // min dim level var
volatile byte deviceControl = 0, deviceControl_temp = 0;  // device control state var
volatile byte change = 0;                                 // change in any of above vars

int read_current_count = 0;         // current read cycle count
int min_analog = 1024;              // minimum analog read
int max_analog = 0;                 // max analog read
int16_t load_amp = 0;               // current  ampere



//  -----   read current sensor ----  //

void currentSens()
{
  int readValue = analogRead(CURRENT_PIN);                                    // analog read current sensor pin
  if (readValue > max_analog)
  {
    max_analog = readValue;
  }
  if (readValue < min_analog)
  {
    min_analog = readValue;
  }
  if (read_current_count > 1000) {
    load_amp = (max_analog - min_analog);
    read_current_count = 0;
    min_analog = 1024;
    max_analog = 0;
  }
  read_current_count++;
}


//  -----   i2c current value request event by master ----  //

void requestEvent() {
  Wire.write((load_amp >> 8) & 0xFF);
  Wire.write(load_amp & 0xFF);
}


// -----  i2c recieve event by master ----- //

void receiveEvent(int nBytes)
{
  if (nBytes)
  {
    dimValue_temp = Wire.read();
    deviceControl_temp = Wire.read();
    dimMax_temp = Wire.read();
    dimMin_temp = Wire.read();
    change = 1;
  }
}


//  ----- zero cross interrupt ISR  ----- //

void zero_crosss_INT()
{
  delayMicroseconds(dimTime);      // waite for time period of dimTime in microsends then trigger triac
  digitalWrite(TRIAC_PIN, HIGH);   // trigger triac high
  delayMicroseconds(10);           // minimum time to trigger triac high (rise time)
  digitalWrite(TRIAC_PIN, LOW);    //trigger triac low

}


//  ----- setup ----- //

void setup()
{
  pinMode(TRIAC_PIN, OUTPUT);       // triac trigger pin as outout
  pinMode(ZCD_DETECT_PIN, INPUT);   // zcd pin as input
  pinMode(RELAY_PIN, OUTPUT);       // relay pin as output

  digitalWrite(TRIAC_PIN, LOW);     // turn off triac on startup
  digitalWrite(RELAY_PIN, LOW);     // turn off relay on startup

  deviceControl = deviceControl_temp = (bool)EEPROM.read(DEVICE_ADDRESS);     // previous state of device control
  dimValue = dimValue_temp = EEPROM.read(DIM_ADDRESS);                        // previous stored value of dim level
  dimMax = dimMax_temp = EEPROM.read(DIM_MAX_ADDRESS);                        // previous stored value of  dim max
  dimMin = dimMin_temp = EEPROM.read(DIM_MIN_ADDRESS);                        // previous stored value of dim min

  if (dimMin < MIN_DIMVALUE)                                                  // check if dim min under absolute min and max
  {
    dimMin = dimMin_temp = MIN_DIMVALUE;
    EEPROM.write(DIM_MIN_ADDRESS, MIN_DIMVALUE);
  }
  if (dimMax > MAX_DIMVALUE)                                                  // check if dim max under absolute min and max
  {
    dimMax = dimMax_temp = MAX_DIMVALUE;
    EEPROM.write(DIM_MAX_ADDRESS, MAX_DIMVALUE);
  }
  
  if ((dimValue < dimMin) || (dimValue > dimMax))                             // check if dim value is under dim min and dim max
  {
    dimValue  = dimValue_temp = MAX_DIMVALUE;
    EEPROM.write(DIM_ADDRESS, dimValue);
  }

  Wire.begin(Slave_Addres);                                                   // initialize i2c commnication as a slave at given slave address
  Wire.onRequest(requestEvent);                                               // callback for i2c request event by master  which is requestEvent()
  Wire.onReceive(receiveEvent);                                               // callback for i2c recieve event by master which is receiveEvent()


  delay(2000);                                                                // power on delay

  dimTime = (75 * dimValue);
  digitalWrite(RELAY_PIN, deviceControl);
  
  attachInterrupt(ZCD_DETECT_PIN, &zero_crosss_INT, RISING);                  // register zero cross isr callback
}


//  ----- loop  ----- //

void loop()
{
  if (deviceControl && (dimValue < (dimMin + 5)))                             // if dim level is less then (dim_min + 5) than trigger triac always high
  {
    digitalWrite(TRIAC_PIN, HIGH);   // trigger triac high
  }
  else
  {
    digitalWrite(TRIAC_PIN, LOW);    //trigger triac low
  }

  if (change) {
    noInterrupts();
    if (deviceControl_temp != deviceControl)                                  // check any change in device control state
    {
      if ((deviceControl_temp == 0) || (deviceControl_temp == 1))
      {
        deviceControl = deviceControl_temp;
        EEPROM.write(DEVICE_ADDRESS, deviceControl);
      }
      else
      {
        deviceControl_temp = deviceControl;
      }
    }
    if (dimValue_temp != dimValue)                                            // check any change in dim level and update memory
    {
      if ((dimValue_temp >= dimMin) && (dimValue_temp <= dimMax))
      {
        dimValue =  dimValue_temp;
        EEPROM.write(DIM_ADDRESS, dimValue);
        dimTime = (75 * dimValue);
      }
      else
      {
        dimValue_temp = dimValue;
      }
    }
    if (dimMax_temp != dimMax)                                                // check any change in dim max value and update memory
    {
      if ((dimMax_temp >= MIN_DIMVALUE) && (dimMax_temp <= MAX_DIMVALUE))
      {
        dimMax = dimMax_temp;
        EEPROM.write(DIM_MAX_ADDRESS, dimMax);
      }
      else
      {
        dimMax_temp = dimMax;
      }
    }
    if (dimMin_temp != dimMin)                                                // check any change in dim min value and update memory
    {
      if ((dimMin_temp >= MIN_DIMVALUE) && (dimMin_temp <= MAX_DIMVALUE))
      {
        dimMin = dimMin_temp;
        EEPROM.write(DIM_MIN_ADDRESS, dimMin);
      }
      else
      {
        dimMin_temp = dimMin;
      }
    }
    change = 0;
    digitalWrite(RELAY_PIN, deviceControl);
    interrupts();
  }

  currentSens();                                                              // read current sensor values

}
