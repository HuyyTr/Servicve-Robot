#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sr_msgs/WheelFeedback.h>
#include <sr_msgs/WheelVelocity.h>
#include <sr_msgs/MinimizeImuMessage.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
#define CONTROL_SERIAL_USART3
#define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
// #define SERIAL_BAUD         230400      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
// SoftwareSerial HoverSerial(2,3);        // RX, TX
#define HoverSerial Serial2

ros::NodeHandle nh;

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################

sr_msgs::WheelFeedback wheel_feedback_msgs;
ros::Publisher wheel_speed_feedback_pub("wheel_speed_feedback", &wheel_feedback_msgs);

void Receive()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
  }
  else {
      return;
  }
  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        // Serial.print(incomingByte);
        String incomingByte_string = "IncomingByte: "+ String(incomingByte);
        nh.loginfo(incomingByte_string.c_str());
        // return;
  #endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
      p       = (byte *)&NewFeedback;
      *p++    = incomingBytePrev;
      *p++    = incomingByte;
      idx     = 2;	
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
      *p++    = incomingByte; 
      idx++;
  }	
  
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
      uint16_t checksum;
      checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

      // Check validity of the new data
      if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
          // Copy the new data
          memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
          String log = "Right Wheel Speed: " + String(Feedback.speedR_meas) + " - Left Wheel Speed: " + String(Feedback.speedL_meas);
          nh.loginfo(log.c_str());
      }
        else {
          // nh.logwarn("start or checksum error...");
        // Serial.println("Non-valid data skipped");
      }
      idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  else{
    String log_idx = "idx: " + String(idx) + " Size of(SerialFeedback): " + String(sizeof(SerialFeedback));
    nh.logwarn(log_idx.c_str());
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

// void serialEvent2()
// {
//   Receive();
// }

MPU6050 mpu;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ########################## SETUP ##########################


void wheelVelocityMessageCallBack(const sr_msgs::WheelVelocity& wheel_velocity_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  Send(wheel_velocity_msg.steer, wheel_velocity_msg.speed);
}

ros::Subscriber<sr_msgs::WheelVelocity> wheel_velocity_sub("wheel_velocity", &wheelVelocityMessageCallBack);

sr_msgs::MinimizeImuMessage minimize_imu_msg;
ros::Publisher imu_pub("minimize_imu_data", &minimize_imu_msg);

void setup() 
{
  //setup for imu sensor
  Wire.begin();
  Wire.setClock(400000); 

  //setup hoverboard
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(13, OUTPUT);
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(wheel_velocity_sub);
  nh.advertise(imu_pub);
  nh.advertise(wheel_speed_feedback_pub);
}

// ########################## LOOP ##########################

unsigned long time_prv;
unsigned long delay_time_now;

bool imu_enable = false;

void loop(void)
{ 
  if(!imu_enable)
  {
    mpu.initialize();

    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150);
    mpu.setYAccelOffset(-50);
    mpu.setZAccelOffset(1060);

    if (mpu.dmpInitialize() == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    }

    imu_enable = true;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGyro(&gy, fifoBuffer);
  }

  Receive();

  if((millis()- time_prv)>80)
  {
    wheel_feedback_msgs.left_wheel_speed_feedback = Feedback.speedL_meas;
    wheel_feedback_msgs.right_wheel_speed_feedback = Feedback.speedR_meas;
    wheel_speed_feedback_pub.publish(&wheel_feedback_msgs);

    minimize_imu_msg.yaw = ypr[0];
    minimize_imu_msg.linear_acceleration[0] = (double(aa.x)/16384)*9.81;
    minimize_imu_msg.linear_acceleration[1] = (double(aa.y)/16384)*9.81;
    minimize_imu_msg.linear_acceleration[2] = (double(aa.z)/16384)*9.81;

    minimize_imu_msg.angular_velocity[0] = (gy.x)/131;
    minimize_imu_msg.angular_velocity[1] = (gy.y)/131;
    minimize_imu_msg.angular_velocity[2] = (gy.z)/131;
    imu_pub.publish(&minimize_imu_msg);
    time_prv = millis(); 
  }  
  nh.spinOnce();
  
  //delay 10 millis 
  delay_time_now = millis();
  while ((millis() < delay_time_now +10))
  {
  }
  
}

// ########################## END ##########################
