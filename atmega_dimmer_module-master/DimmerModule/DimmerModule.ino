#include "src/modlib/I2C.h"
#include "src/modlib/energy_monitor.h"
#include "Seeed_QTouch.h"
#include "SoftwareWire.h"
#include <EEPROM.h>

#define FIRMWARE_V_C 10 // firmware version

#define SOFT_I2C_SDA 12      // secondary i2c sda
#define SOFT_I2C_SCL 4       // secondary i2c scl
#define SUB_MODULE_ADDR 0x30 // sub module i2c address

#define MEM_CONTROL_ON_OFF_ADDRESS 0x01        // eeprom memory address for constrol on off
#define MEM_DIM_ADDRESS 0x04                   // eeprom memory address for dimming value
#define MEM_DIM_MAX_ADDRESS 0x07               // eeprom memory address for dim max value
#define MEM_DIM_MIN_ADDRESS 0x0A               // eeprom memory address for dim min value
#define MEM_BUTTON_LED_BRIGHTNESS_ADDRESS 0x0D // eeprom memory address for button led brightness
#define MEM_STATUS_LED_BRIGHTNESS_ADDRESS 0x10 // eeprom memory address for status led brightness
#define MEM_BUTTON_LOCK_ADDRESS 0x13           // eeprom memory address for button lock
#define MEM_TOUCH_LOCK_ADDRESS 0x16            // eeprom memory address for touch lock

#define SHIFT_DATA_PIN 7   // shift register data pin
#define SHIFT_CLCOK_PIN 11 // shift register clock
#define SHIFT_LATCH_PIN 10 // shift register latch
#define SHIFT_ENABLE_PIN 9 // shift register enable
#define BUTTON_LED_1_PIN 5 // push button led 1 white
#define BUTTON_LED_2_PIN 6 // push button led 2 blue
#define PUSH_BUTTON_PIN 8  // push button pin
#define TOUCH_RESET_PIN 13 // reset pin of touch ic
#define TOUCH_CHANGE_PIN 3 // change interrupt pin of touch ic
#define VSENS_PIN A0       // pairing pin

#define MAX_DIM_MAX_DEFAULT 5 // default max dim value
#define MAX_DIM_MIN_DEFAULT 1 // defaul min dim value
#define DIM_DEFAULT 1         // default dim value

#define CURRENT_PIN A7      // ac currrent sensor pin
#define RELAY_1_PIN A2      // low value capacitor relay
#define RELAY_2_PIN A1      // mid value capacitor relay
#define RELAY_3_PIN A3      // high value capacitor relay
#define FULL_ON_RELAY_PIN 2 // full speed realy

#define TOUCH_I2C_ADDRESS 0x1B // i2c address of touch ic

#define LEAST_LED_BRIGHTNESS 1    // least led brightness of button led
#define DEFAULT_LED_BRIGHTNESS 25 // Default led brightness of button led

#define MAX_TOUCHPAD_VALUE 5
#define MIN_TOUCHPAD_VALUE 1

uint16_t current_sens = 0; // current sensor var
uint16_t avg_power = 0;
uint8_t on_off = 0, on_off_temp = 0;                                                                         // control on-off var
uint8_t dim_value = 1, dim_value_temp = 1;                                                                   // dimming var
uint8_t dim_max = 5, dim_max_temp = 5;                                                                       // dimming max var
uint8_t dim_min = 1, dim_min_temp = 1;                                                                       // dimming min var
uint8_t button_led_brightness = DEFAULT_LED_BRIGHTNESS, button_led_brightness_temp = DEFAULT_LED_BRIGHTNESS; // button led brightness level var
uint8_t status_led_brightness = 245, status_led_brightness_temp = 245;                                       // status led brightness level var
uint8_t is_button_lock = 0, is_button_lock_temp = 0;                                                         // lock button var
uint8_t is_touch_lock = 0, is_touch_lock_temp = 0;                                                           // lock touch var
uint8_t is_motion = 0;                                                                                       // motion
uint8_t logs[70] = {0};                                                                                      // module log
uint8_t lastTouch = 0;                                                                                       // last touch for logs

int blink_led = 0; // status leds blinking count

uint8_t led[7] = {128, 130, 134, 142, 174, 190}; // status led sequence
uint8_t single_led[7] = {128, 2, 4, 8, 32, 16};  // single led sequence

int countx = 0;

int i2c_count = 0;
unsigned long i2c_timeout = 0;

energy_handle_t load_energy_handle;

// -------callback functions for power measurement ----//
uint16_t read_voltage(void)
{
  return analogRead(VSENS_PIN);
}
uint16_t read_current(void)
{
  return analogRead(CURRENT_PIN);
}

//  ----- update eeprom memory  ----- //

void updateMemory(int address, byte data) // store value in eerom memory
{
  EEPROM.write(address, data);
}

//  ----- update status leds value ----- //

void updateStatusLed(byte ledData)
{
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLCOK_PIN, MSBFIRST, ledData); // shift data in to status led shift-register
  digitalWrite(SHIFT_LATCH_PIN, HIGH);
}

void resetTouch()
{
  digitalWrite(TOUCH_RESET_PIN, HIGH); // pull low reset once of touch ic on startup
  delay(50);
  digitalWrite(TOUCH_RESET_PIN, LOW);
  delay(50);
  digitalWrite(TOUCH_RESET_PIN, HIGH);

  QTouch.reset();
}

//  ----- update status leds value ----- //

void touchCheck()
{
  static unsigned long touchTime = 0;
  if ((millis() - touchTime) > 300000)
  {
    touchTime = millis();
    softWire.beginTransmission(TOUCH_I2C_ADDRESS);
    uint8_t error = softWire.endTransmission();
    if (error != 0)
    {
      resetTouch();
    }
    QTouch.calibrate();
  }
}

//  ----- delay of 250msec ---- //

void delay250ms()
{
  uint8_t countx = 0;
  while (countx < 251)
  {
    delayMicroseconds(1000);
    countx++;
  }
}

// -----  motion event ----- //

void isMotion()
{
 /* This flag will be set after setting led */
  static bool motion_led_set=false;
  /* time stamp of last led set*/
  static unsigned long last_motion_led_ts=millis();
  /* Motion led is not set*/
  if (is_motion == 1&&!motion_led_set)
  { 
    if (!on_off)
    {
      analogWrite(BUTTON_LED_1_PIN, LEAST_LED_BRIGHTNESS);
      analogWrite(BUTTON_LED_2_PIN, LEAST_LED_BRIGHTNESS);
      /* Set motion led set flag */
      motion_led_set=true;
      /* Get the time stamp*/
      last_motion_led_ts=millis();
    }
    else
    {
      is_motion = 0;
      motion_led_set=false;
    }
  }
  else if(is_motion == 1&&motion_led_set) /** Motion Led already set */
  {
    /* On time out reset the flag and udate led */
    if(millis()-last_motion_led_ts>35000)
    {
      is_motion = 0;
      motion_led_set=false;
      int a = map(dim_value, dim_min, dim_max, 1, 5);
      (on_off) ? analogWrite(BUTTON_LED_1_PIN, button_led_brightness) : analogWrite(BUTTON_LED_1_PIN, 0);
      (on_off) ? analogWrite(BUTTON_LED_2_PIN, button_led_brightness) : analogWrite(BUTTON_LED_2_PIN, 0);
      (on_off) ? updateStatusLed(led[a]) : updateStatusLed(led[0]);
    }
  } /** if other condition occurs*/
  else
  {
    motion_led_set=false;
  }
}

// ---- update logs ---- //

void updateLog()
{
  static unsigned long time_log = 0;
  static unsigned long dummy_count = 0;

  if ((millis() - time_log) > 60000)
  {
    time_log = millis();
    softWire.beginTransmission(0x1B);
    uint8_t err1 = softWire.endTransmission();
    (err1 == 0) ? err1 = 1 : err1 = 0;
    sprintf(logs, "{\"AC_V\":%d,\"control\": %d,\"dummyLoad\": %d}", load_energy_handle.v_avg, on_off, dummy_count);
    dummy_count++;
  }
}

//  ----- change sensivity of touch pads  ----- //

void changTouchSens()
{
  softWire.beginTransmission(0x1B);
  softWire.write(32);
  softWire.write(2);
  softWire.endTransmission();
  delay(50);
  softWire.beginTransmission(0x1B);
  softWire.write(33);
  softWire.write(2);
  softWire.endTransmission();
  delay(50);
  softWire.beginTransmission(0x1B);
  softWire.write(34);
  softWire.write(2);
  softWire.endTransmission();
  delay(50);
  softWire.beginTransmission(0x1B);
  softWire.write(35);
  softWire.write(2);
  softWire.endTransmission();
  delay(50);
  softWire.beginTransmission(0x1B);
  softWire.write(36);
  softWire.write(2);
  softWire.endTransmission();
  delay(50);
  softWire.beginTransmission(0x1B);
  softWire.write(37);
  softWire.write(2);
  softWire.endTransmission();
}

//  ----- status led blinking ---- //

void isLedBlink()
{
  if (blink_led && on_off)
  {
    int a = map(dim_value, dim_min, dim_max, 1, 5);
    if ((blink_led > 400 && blink_led < 800) || (blink_led > 1200 && blink_led < 1600) || (blink_led > 2000 && blink_led < 2400) || (blink_led > 2800 && blink_led < 3200))
    {
      updateStatusLed(led[a - 1]);
      delayMicroseconds(350);
    }
    else
    {
      updateStatusLed(led[a]);
      delayMicroseconds(350);
    }
    blink_led++;
    if (blink_led >= 3600)
    {
      blink_led = 0;
    }
  }
}

//  ----- delay before turing relay on  ----- //

void dimTime()
{
  if (countx)
  {
    delayMicroseconds(100);
    countx++;
    if (countx >= 4500)
    {
      relayChange();
      blink_led = 1;
      countx = 0;
    }
  }
}

//  ----- read top current sensor value ----- //

void getCurrent()
{
  static int read_current_count = 0, max_analog = 0, min_analog = 0;

  int readValue = analogRead(CURRENT_PIN);
  if (readValue > max_analog)
  {
    max_analog = readValue;
  }
  if (readValue < min_analog)
  {
    min_analog = readValue;
  }
  if (read_current_count > 10000)
  {
    read_current_count = 0;
    current_sens = (max_analog - min_analog);
    max_analog = 0;
    min_analog = 1024;
  }
  read_current_count++;
}

// ---------------Get power -------------//
void getPower(void)
{
  calculate_active_power(&load_energy_handle);
  calculate_voltage_avg(&load_energy_handle);
  calculate_current_avg(&load_energy_handle);
  avg_power = load_energy_handle.p_avg;
}

//  ----- read button pin ----- //

void getButton()
{
  static bool last_state = 0;
  static unsigned long start_pressed = 0;
  static unsigned long timeLongHold = 0;

  bool button_state = digitalRead(PUSH_BUTTON_PIN);
  if (button_state != last_state)
  {
    if (button_state == HIGH)
    {
      start_pressed = millis();
    }
    else
    {
      if (!is_button_lock)
      {
        if (((millis() - start_pressed) >= 10) && ((millis() - start_pressed) < 1000))
        {
          on_off_temp = !on_off_temp;
        }
      }
      else
      {
        delay(500);
        analogWrite(BUTTON_LED_1_PIN, button_led_brightness);
        analogWrite(BUTTON_LED_2_PIN, button_led_brightness);
        delay(500);
        analogWrite(BUTTON_LED_1_PIN, 0);
        analogWrite(BUTTON_LED_2_PIN, 0);
        delay(500);
        analogWrite(BUTTON_LED_1_PIN, button_led_brightness);
        analogWrite(BUTTON_LED_2_PIN, button_led_brightness);
        delay(500);
        analogWrite(BUTTON_LED_1_PIN, 0);
        analogWrite(BUTTON_LED_2_PIN, 0);
        delay(500);
        analogWrite(BUTTON_LED_1_PIN, button_led_brightness);
        analogWrite(BUTTON_LED_2_PIN, button_led_brightness);
        delay(500);
        analogWrite(BUTTON_LED_1_PIN, 0);
        analogWrite(BUTTON_LED_2_PIN, 0);
        delay(500);
        (on_off) ? analogWrite(BUTTON_LED_1_PIN, button_led_brightness) : analogWrite(BUTTON_LED_1_PIN, 0);
        (on_off) ? analogWrite(BUTTON_LED_2_PIN, button_led_brightness) : analogWrite(BUTTON_LED_2_PIN, 0);
      }
    }
  }
  last_state = button_state;
}

//  ----- read touch pads ----- //

void getTouch()
{
  static int curr = 0, last = 0;
  if (digitalRead(TOUCH_CHANGE_PIN) == LOW)
  {
    int readTouch = QTouch.touchNum();
    if (!is_touch_lock)
    {
      if ((readTouch >= MIN_TOUCHPAD_VALUE && readTouch <= MAX_TOUCHPAD_VALUE) && (on_off))
      {
        curr = readTouch;
        if (curr != last && curr >= 0)
        {
          last = curr;
          dim_value_temp = map(curr, MIN_TOUCHPAD_VALUE, MAX_TOUCHPAD_VALUE, dim_min, dim_max);
          countx = 1;
          blink_led = 0;
        }
        lastTouch = curr;
      }
    }
    else
    {
      delay(500);
      analogWrite(SHIFT_ENABLE_PIN, status_led_brightness);
      delay(500);
      analogWrite(SHIFT_ENABLE_PIN, 255);
      delay(500);
      analogWrite(SHIFT_ENABLE_PIN, status_led_brightness);
      delay(500);
      analogWrite(SHIFT_ENABLE_PIN, 255);
      delay(500);
      analogWrite(SHIFT_ENABLE_PIN, status_led_brightness);
    }
  }
}

//  ----- check if on_off state is changed  ----- //

void isOnOffChange()
{
  if (on_off != on_off_temp)
  {
    if (on_off_temp == 0 || on_off_temp == 1)
    {
      on_off = on_off_temp;
      (on_off) ? analogWrite(BUTTON_LED_1_PIN, button_led_brightness) : analogWrite(BUTTON_LED_1_PIN, 0);
      (on_off) ? analogWrite(BUTTON_LED_2_PIN, button_led_brightness) : analogWrite(BUTTON_LED_2_PIN, 0);
      int a = map(dim_value, dim_min, dim_max, 1, 5);
      (on_off) ? updateStatusLed(led[a]) : updateStatusLed(0);
      if (on_off)
      {
        countx = 4000;
      }
      else
      {
        digitalWrite(FULL_ON_RELAY_PIN, LOW);
        delay250ms();
        digitalWrite(RELAY_2_PIN, LOW);
        delay250ms();
        digitalWrite(RELAY_3_PIN, LOW);
        delay250ms();
        digitalWrite(RELAY_1_PIN, LOW);
        delay250ms();
      }
      updateMemory(MEM_CONTROL_ON_OFF_ADDRESS, on_off);
    }
    else
    {
      on_off_temp = on_off;
    }
  }
}

//  ----  change in capacitor relays according to dim vlaue ----  //

void relayChange()
{
  if (on_off)
  {
    uint8_t a = map(dim_value, dim_min, dim_max, 1, 5);
    switch (a)
    {
    case 0:
      digitalWrite(FULL_ON_RELAY_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_2_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_3_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_1_PIN, HIGH);
      delay250ms();
      break;
    case 1:
      digitalWrite(FULL_ON_RELAY_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_1_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_3_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_2_PIN, HIGH);
      delay250ms();
      break;
    case 2:
      digitalWrite(FULL_ON_RELAY_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_1_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_2_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_3_PIN, HIGH);
      delay250ms();
      break;
    case 3:
      digitalWrite(FULL_ON_RELAY_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_2_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_1_PIN, HIGH);
      delay250ms();
      digitalWrite(RELAY_3_PIN, HIGH);
      delay250ms();
      break;
    case 4:
      digitalWrite(FULL_ON_RELAY_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_1_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_2_PIN, HIGH);
      delay250ms();
      digitalWrite(RELAY_3_PIN, HIGH);
      delay250ms();
      break;
    case 5:
      digitalWrite(RELAY_1_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_2_PIN, LOW);
      delay250ms();
      digitalWrite(RELAY_3_PIN, LOW);
      delay250ms();
      digitalWrite(FULL_ON_RELAY_PIN, HIGH);
      delay250ms();
      break;
    }
  }
  else
  {
    digitalWrite(FULL_ON_RELAY_PIN, LOW);
    delay250ms();
    digitalWrite(RELAY_2_PIN, LOW);
    delay250ms();
    digitalWrite(RELAY_3_PIN, LOW);
    delay250ms();
    digitalWrite(RELAY_1_PIN, LOW);
    delay250ms();
  }
}

//  ----- check if dim value is change  ----- //

void isDimChange()
{
  if (dim_value != dim_value_temp)
  {
    if ((dim_value_temp >= dim_min) && (dim_value_temp <= dim_max))
    {
      dim_value = dim_value_temp;
      updateMemory(MEM_DIM_ADDRESS, dim_value);
      if (on_off)
      {
        int a = map(dim_value, dim_min, dim_max, 1, 5);
        updateStatusLed(led[a]);
        countx = 1;
      }
    }
    else
    {
      dim_value_temp = dim_value;
    }
  }
}

//  ----- check if dim min value is change ----- ///

void isDimMinChange()
{
  if (dim_min != dim_min_temp)
  {
    if (dim_min_temp >= MAX_DIM_MIN_DEFAULT && dim_min_temp < MAX_DIM_MAX_DEFAULT)
    {
      dim_min = dim_min_temp;
      updateMemory(MEM_DIM_MIN_ADDRESS, dim_min);
      dim_value_temp = map(dim_value_temp, dim_min, dim_max, dim_min, dim_min);
    }
    else
    {
      dim_min_temp = dim_min;
    }
  }
}

// ----- check if dim max value is change ----- //

void isDimMaxChange()
{
  if (dim_max != dim_max_temp)
  {
    if (dim_max_temp <= MAX_DIM_MAX_DEFAULT && dim_max_temp > MAX_DIM_MIN_DEFAULT)
    {
      dim_max = dim_max_temp;
      updateMemory(MEM_DIM_MAX_ADDRESS, dim_max);
      dim_value_temp = map(dim_value_temp, dim_min, dim_max, dim_min, dim_min);
    }
    else
    {
      dim_max_temp = dim_max;
    }
  }
}

// ----- check if status led brightness value is change ----- //

void isStatusLedChange()
{
  if (status_led_brightness != status_led_brightness_temp)
  {
    status_led_brightness = status_led_brightness_temp;
    (on_off) ? analogWrite(SHIFT_ENABLE_PIN, status_led_brightness) : analogWrite(SHIFT_ENABLE_PIN, 0);
    updateMemory(MEM_STATUS_LED_BRIGHTNESS_ADDRESS, status_led_brightness);
  }
}

// ----- check if buttton led brightness value is cahnge ----- //

void isButtonLedChange()
{
  if (button_led_brightness != button_led_brightness_temp)
  {
    button_led_brightness = button_led_brightness_temp;
    (on_off) ? analogWrite(BUTTON_LED_1_PIN, button_led_brightness) : analogWrite(BUTTON_LED_1_PIN, 0);
    (on_off) ? analogWrite(BUTTON_LED_2_PIN, button_led_brightness) : analogWrite(BUTTON_LED_2_PIN, 0);
    updateMemory(MEM_BUTTON_LED_BRIGHTNESS_ADDRESS, button_led_brightness);
  }
}

// ----- check if change in button lock ----- //

void isButtonLock()
{
  if (is_button_lock_temp != is_button_lock)
  {
    if (is_button_lock_temp == 1 || is_button_lock_temp == 0)
    {
      is_button_lock = is_button_lock_temp;
      updateMemory(MEM_BUTTON_LOCK_ADDRESS, is_button_lock);
    }
    else
    {
      is_button_lock_temp = is_button_lock;
    }
  }
}

// ----- check if change in touch lock ----- //

void isTouchLock()
{
  if (is_touch_lock_temp != is_touch_lock)
  {
    if (is_touch_lock_temp == 1 || is_touch_lock_temp == 0)
    {
      is_touch_lock = is_touch_lock_temp;
      updateMemory(MEM_TOUCH_LOCK_ADDRESS, is_touch_lock);
    }
    else
    {
      is_touch_lock_temp = is_touch_lock;
    }
  }
}

// ----- boot up led notification ----- //

void startNotification()
{
  if (eeprom_read_byte(MEM_SLAVE_FLAG) == SLAVE_ADDRESS_FLAG)
  {
    for (uint8_t i = 0; i < 2; i++)
    {
      for (uint8_t k = 1; k < 25; k++)
      {
        analogWrite(BUTTON_LED_1_PIN, k);
        analogWrite(BUTTON_LED_2_PIN, k);
        delay(50);
      }
      delay(50);
      for (uint8_t k = 25; k > 0; k--)
      {
        analogWrite(BUTTON_LED_1_PIN, k);
        analogWrite(BUTTON_LED_2_PIN, k);
        delay(50);
      }
      delay(50);
    }
  }
  else
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      analogWrite(BUTTON_LED_1_PIN, 255);
      analogWrite(BUTTON_LED_2_PIN, 255);
      delay(250);
      analogWrite(BUTTON_LED_1_PIN, 0);
      analogWrite(BUTTON_LED_2_PIN, 0);
      delay(250);
    }
  }
}

//  ----- check i2c pair on vsense pin ----- //

void isI2CPair()
{
  unsigned long time_now = millis();
  unsigned long pressed = 0;
  bool state = digitalRead(VSENS_PIN);
  uint8_t count = 0;
  while ((millis() - time_now) < 500) // read pulse train on VENS_PIN
  {
    bool change = digitalRead(VSENS_PIN);
    if (change != state && ((millis() - pressed) >= 20 && (millis() - pressed) <= 100))
    {
      pressed = millis();
      state = change;
      count++;
      if (count >= 5)
      {
        break;
      }
    }
  }
  if (count >= 5) // if VSENS_PIN pulse count is more than 5
  {
    eeprom_write_byte(MEM_SLAVE_FLAG, 0xFF);
    I2C.pairingWait(); // intialize i2c pairing port at address 10
    unsigned long time_now = millis();
    while ((millis() - time_now) < 1000)
    {
      if (eeprom_read_byte(MEM_SLAVE_FLAG) == SLAVE_ADDRESS_FLAG) // if eeprom slave flag is similar to slave address flag means its paired and exit while loop
      {
        break;
      }
      delay(100);
    }
    I2C.stop(); // disable i2c
  }
}
//  ----- setup  ----- //

void setup()
{
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  pinMode(SHIFT_CLCOK_PIN, OUTPUT);
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  pinMode(SHIFT_CLCOK_PIN, OUTPUT);
  pinMode(SHIFT_ENABLE_PIN, OUTPUT);
  pinMode(TOUCH_RESET_PIN, OUTPUT);
  pinMode(BUTTON_LED_1_PIN, OUTPUT);
  pinMode(BUTTON_LED_2_PIN, OUTPUT);
  pinMode(PUSH_BUTTON_PIN, INPUT);
  pinMode(VSENS_PIN, INPUT);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(RELAY_3_PIN, OUTPUT);
  pinMode(FULL_ON_RELAY_PIN, OUTPUT);
  pinMode(TOUCH_CHANGE_PIN, INPUT);

  digitalWrite(FULL_ON_RELAY_PIN, LOW);
  digitalWrite(RELAY_2_PIN, LOW);
  digitalWrite(RELAY_3_PIN, LOW);
  digitalWrite(RELAY_1_PIN, LOW);

  analogWrite(BUTTON_LED_1_PIN, 0);   // turn off button led 1 on startup
  analogWrite(BUTTON_LED_2_PIN, 0);   // turn off button led 2 on startup
  analogWrite(SHIFT_ENABLE_PIN, 255); // dim low staus led on startup
  updateStatusLed(0);                 // turn off all staus led on startup

  I2C.stop();

  isI2CPair();

  softWire.begin(); // initialise software i2c

  resetTouch(); // reset touch ic

  load_energy_handle.get_current_raw_value = read_current;
  load_energy_handle.get_voltage_raw_value = read_voltage;
  load_energy_handle.is_inverse = 1;

  energy_handle_init(&load_energy_handle);

  on_off = on_off_temp = (bool)EEPROM.read(MEM_CONTROL_ON_OFF_ADDRESS);                                // previous state of device control
  dim_value = dim_value_temp = EEPROM.read(MEM_DIM_ADDRESS);                                           // previous stored value of dim level
  dim_max = dim_max_temp = EEPROM.read(MEM_DIM_MAX_ADDRESS);                                           // previous stored value of  dim max
  dim_min = dim_min_temp = EEPROM.read(MEM_DIM_MIN_ADDRESS);                                           // previous stored value of dim min
  button_led_brightness = button_led_brightness_temp = EEPROM.read(MEM_BUTTON_LED_BRIGHTNESS_ADDRESS); // previous stored value of button led brightness
  status_led_brightness = status_led_brightness_temp =200; //EEPROM.read(MEM_STATUS_LED_BRIGHTNESS_ADDRESS); // previous stored value of status leds brightness
  is_button_lock = is_button_lock_temp = (bool)EEPROM.read(MEM_BUTTON_LOCK_ADDRESS);                   // previous stored value of button lock
  is_touch_lock = is_touch_lock_temp = (bool)EEPROM.read(MEM_TOUCH_LOCK_ADDRESS);                      // previous stored value of touch lock

  if ((dim_max > MAX_DIM_MAX_DEFAULT) || (dim_max <= MAX_DIM_MIN_DEFAULT)) // check if dim max under absolute min and max
  {
    dim_max = dim_max_temp = MAX_DIM_MAX_DEFAULT;
  }
  if ((dim_min < MAX_DIM_MIN_DEFAULT) || (dim_min >= MAX_DIM_MAX_DEFAULT)) // check if dim min under absolute min and max
  {
    dim_min = dim_min_temp = MAX_DIM_MIN_DEFAULT;
  }
  if ((dim_value < dim_min) || (dim_value > dim_max)) // check if dim value is under dim min and dim max
  {
    dim_value = dim_value_temp = DIM_DEFAULT;
  }

  if (is_button_lock != 0 || is_button_lock != 1)
  {
    is_button_lock = is_button_lock_temp = 0;
    updateMemory(MEM_BUTTON_LOCK_ADDRESS, is_button_lock);
  }

  if (is_touch_lock != 0 || is_touch_lock != 1)
  {
    is_touch_lock = is_touch_lock_temp = 0;
    updateMemory(MEM_TOUCH_LOCK_ADDRESS, is_touch_lock);
  }

  //changTouchSens();                                                                                       // change touch sensitivity

  sprintf(logs, "{\"start\": 1,\"control\": %d,\"isTouch\": 0, \"lastTouch\": 0, \"dummyLoad\": 0}", on_off);

  /* ---- NOTE: first assign all i2c events than initialize i2c ----*/
  I2C.firmwareVC = FIRMWARE_V_C; // assign firmeare version, if not assign it will be 0 default
  I2C.attachEvent(32, 64, FEATURE_BOOL, &on_off_temp, 0);
  I2C.attachEvent(33, 0, FEATURE_INTEGER, &current_sens, 0);
  I2C.attachEvent(34, 65, FEATURE_BYTE, &dim_value_temp, 1);
  I2C.attachEvent(35, 66, FEATURE_BYTE, &status_led_brightness_temp, 254);
  I2C.attachEvent(36, 0, FEATURE_INTEGER, &avg_power, 0);
  I2C.attachEvent(37, 67, FEATURE_BOOL, &is_button_lock_temp, 0);
  I2C.attachEvent(38, 68, FEATURE_BOOL, &is_touch_lock_temp, 0);
  I2C.attachLog((uint8_t)sizeof(logs), logs);
  I2C.attachLedBrightnessData(&button_led_brightness_temp);
  I2C.attachMotionData(&is_motion);

  I2C.init(); // initialize i2c

  startNotification(); // power on led sequence
  startNotification(); // power on led sequence

  delay(1000);

  analogWrite(SHIFT_ENABLE_PIN, status_led_brightness);

  if (on_off)
  {
    analogWrite(BUTTON_LED_1_PIN, button_led_brightness);
    analogWrite(BUTTON_LED_2_PIN, button_led_brightness);
    int a = map(dim_value, dim_min, dim_max, 1, 5);
    updateStatusLed(led[a]);
    countx = 1;
  }
  else
  {
    analogWrite(BUTTON_LED_1_PIN, 0);
    analogWrite(BUTTON_LED_2_PIN, 0);
    updateStatusLed(0);
  }
}

//  ------ loop ----- //

void loop()
{
  getButton();

  getTouch();

  isOnOffChange();

  isDimMaxChange();

  isDimMinChange();

  isDimChange();

  isButtonLedChange();

  isStatusLedChange();

  isButtonLock();

  isTouchLock();

  isLedBlink();

  getCurrent();

  dimTime();

  touchCheck();

  updateLog();

  isMotion();

  getPower();

  if (i2c_count != I2C.countx)
  {
    i2c_count = I2C.countx;
    i2c_timeout = millis();
  }
  if ((millis() - i2c_timeout) > 10000)
  {
    I2C.stop();
    delay(100);
    I2C.init();
    i2c_timeout = millis();
  }
}
