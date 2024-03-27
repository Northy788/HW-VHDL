#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <Wire.h>
#include "driver/timer.h"

#define OK 0
#define BUSY 1
#define WAIT 2
#define ERROR 4
#define TIMEOUT -1

#define TRUE 1
#define FALSE 0

Adafruit_MPU6050 mpu;
VL53L0X sensor;

timer_config_t t_conf = {
  TIMER_ALARM_DIS,
  TIMER_PAUSE,
  TIMER_INTR_LEVEL,
  TIMER_COUNT_UP,
  TIMER_AUTORELOAD_DIS,
  16000
};
uint16_t sit_range = 0;
uint8_t clock_status = WAIT;
uint16_t range = 0;
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  Wire.begin();
  MPU6050_INIT();
  VL53L0X_INIT();
  timer_init(TIMER_GROUP_1, TIMER_0 , &t_conf);
  //while (get_distance() == TIMEOUT) {};
  sit_range = sensor.readRangeContinuousMillimeters();
  clock_status = OK;
  Serial.print(sit_range);
  Serial.println(" -> OK!");
}

void loop() {
  clock_position_motion_dectect();
  range = sensor.readRangeContinuousMillimeters();
  //talk with FPGA
  //query time
  //
  // put your main code here, to run repeatedly:
  //delay(10);
}

void MPU6050_INIT()
{
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  Serial.println("");
  delay(100);
}

uint8_t detected_motion()
{
  if(mpu.getMotionInterruptStatus()) {
    return TRUE;
  }
  return FALSE;
}

void VL53L0X_INIT()
{
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

uint16_t get_distance()
{ 
  uint16_t range = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT");}
  return range;
}

void clock_position_motion_dectect(void)
{
  if (detected_motion() )
  {
    clock_status = BUSY;
    Serial.println("DETECT CLOCK MOTION!");
  }
  while (clock_status == BUSY)
  {
    if (!detected_motion())
    {
      //clock_status = WAIT; then
      clock_status = WAIT;
      Serial.println("");
      Serial.println("NOT DETECT CLOCK MOTION!");
      timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
      timer_start(TIMER_GROUP_1, TIMER_0);
      Serial.println("CHECKING");
    }
    Serial.print(".");
  }
  while (clock_status == WAIT)
  {
    double val;
    timer_get_counter_time_sec(TIMER_GROUP_1, TIMER_0, &val);
    if (detected_motion())
    {
      clock_status = BUSY;
      Serial.println("");
      Serial.println("DETECT CLOCK MOTION AGAIN!");
      timer_pause(TIMER_GROUP_1, TIMER_0);
      timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
    }
    /* check value with couter if 5 sec it not moving save new sit_range*/
    if (val >= 5.0){
      sit_range = range; 
      clock_status = OK;
      timer_pause(TIMER_GROUP_1, TIMER_0);
      Serial.printf("GET NEW CLOCK POSITION! -> %d", sit_range);
      timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
    }
    Serial.println(val);
  }
}
