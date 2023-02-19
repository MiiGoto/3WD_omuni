/*
  < Intended use environment >
  Host machine OS : Ubuntu 20.04.LST
  ROS distribution : ROS2 foxy
  Microcontroller board: ESP32-S3-WROOM-1 on devkitC
  I2C pins(SDA, SCL) : 10, 11
  MD address : 0x0a, 0x0b, 0x0c

  Subscribe '/cmd_vel' (geometry_msgs/Twist) topic via micro-ROS agent
  for Move real robot of 3w 
  Auther : Kawai
*/


// INCLUDE FILES ///////////////////////////////////////////////////////////
#include <stdio.h>
#include <math.h>
#include <vector>
#include <Wire.h>
// OnBoardLed
#include <Adafruit_NeoPixel.h>

// micro-ROS
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

// DECLARATIONS ////////////////////////////////////////////////////////
// OnBoardLed
Adafruit_NeoPixel OnBoardLed(1, 48, NEO_GRB + NEO_KHZ800);

// micro-ROS
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// 3w
const float control_mat[3][3] = {{0.6667, 0.0000, 0.3333},
                                 {-0.3333, -0.5774, 0.3333},
                                 {-0.3333, 0.5774, 0.3333}};
const int wheel_address[3] = {0x0a, 0x0b, 0x0c};  //足回り用ＭＤのアドレス
const int n_wheels = sizeof(wheel_address) / sizeof(int);
static unsigned long loopTime = 0, lastLoop = 0;

// FUNCTIONS ///////////////////////////////////////////////////////////////
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// error_loop for RCCHECK
void error_loop(){
  while(1){
    OnBoardLed.setPixelColor(0, 2, 0, 0);
    OnBoardLed.show();
    delay(500);
    OnBoardLed.setPixelColor(0, 0, 0, 0);
    OnBoardLed.show();
    delay(500);
  }
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * cmd_vel = (const geometry_msgs__msg__Twist *)msgin;
  
  // glows blue when it work
  OnBoardLed.clear();
  OnBoardLed.setPixelColor(0, 0, 2, 20);
  OnBoardLed.show();
}

// ARDUINO SETUP ///////////////////////////////////////////////////////////
void setup() {
  // OnBoardLed setup
  OnBoardLed.begin();
  OnBoardLed.clear();
  OnBoardLed.setPixelColor(0, 5, 5, 5);
  OnBoardLed.show();

  // I2C setup
  Wire.begin(10, 11);

  // All the following are micro-ROS setup
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "real_3w_operater", "", &support));
  // create subscription
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel, &subscription_callback, ON_NEW_DATA));

  // initialize cmd_vel(Twist)
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;
}

// ARDUINO LOOP ////////////////////////////////////////////////////////////
void loop() {
  // glows a little green when in the loop
  OnBoardLed.setPixelColor(0, 0, 0, 2);
  OnBoardLed.show();
  float wheel_speed[3] = {0, 0, 0};  // 足回り用の三つのモーターへの速度指令
  float speed_command[3] = {0, 0, 0};  // 機体のx, y, Θの速度(角速度)

  // measure the loop time
  loopTime = micros() - lastLoop;
  lastLoop = micros();

  // micro-ROS spin
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  // HACK: convert cmd_vel to speed_command 
  // 変換なしで代入するだけにしたいよね
  speed_command[0] = cmd_vel.linear.x / 0.25; // HACK: linear.x(m/s) / 0.25 = joy_raw_data
  speed_command[1] = cmd_vel.linear.y / 0.25; // HACK: linear.y(m/s) / 0.25 = joy_raw_data
  speed_command[2] = cmd_vel.angular.z / 3; // HACK: angular.z(rad/s) / 3 = joy_raw_data

  // TODO: IMUの値を使って速度のPID制御を行う

  for (int i = 0; i < n_wheels; i++) {
    for (int j = 0; j < n_wheels; j++) {
      wheel_speed[i] += control_mat[i][j] * speed_command[j];
    }
    
    // TODO: MD側のプログラムも書き換えてここの無駄な127を消して-1~1でやり取りをする(多分intでやり取りしてた時の名残)
    wheel_speed[i] = wheel_speed[i] * 127; //今までは-1~1を-127~127に変換して使ってた
    wheel_speed[i] = constrain(wheel_speed[i], -127, 127); 

    // I2C 送信処理
    Wire.beginTransmission(wheel_address[i]);
    Wire.write((char)wheel_speed[i]);
    Wire.endTransmission();
  }
}

