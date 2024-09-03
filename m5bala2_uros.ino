#define M5STACK_MPU6886 

#include <M5Stack.h>
#include "freertos/FreeRTOS.h"
#include "imu_filter.h"
#include "MadgwickAHRS.h"
#include "bala.h"
#include "pid.h"
#include "calibration.h"

// ROS
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

//#define CLIENT_IP "xxxxx"
#define WIFINAME "BSL"
#define WIFIPASS "b10Inspired"
#define AGENT_IP "172.17.39.171"
#define AGENT_PORT 9003
#define NODE_NAME "m5bala2_node"
#define FREQ 40 //Hz


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

//#define LED_PIN 

extern uint8_t bala_img[41056];
static void PIDTask(void *arg);
static void draw_waveform();
int cnt_display = 0;

static float angle_point = -1.5;
int32_t w_left, w_right;
int16_t left_out, right_out;
int16_t forward_offset = 0;
int16_t left_offset = 0;
int16_t right_offset = 0;

// hyper parameters
float incli_angle = 0.0f;
float kp = 24.0f, ki = 0.0f, kd = 90.0f; // default
float s_kp = 15.0f, s_ki = 0.075f, s_kd = 0.0f;

bool calibration_mode = false;

// ------ ROS ------
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_publisher_t publisher;
rcl_subscription_t incli_subscriber;
rcl_subscription_t steer_subscriber;
rclc_executor_t pub_executor;
rclc_executor_t sub_executor;
rcl_timer_t timer;
// ros msgs
std_msgs__msg__Float32 incli_msg;
std_msgs__msg__Int16 steer_msg;


std_msgs__msg__String msg;
// ip address
IPAddress client_ip;
IPAddress agent_ip;
// topic values
//float incli_angle = 10.5f;
int steer_angle = 0;
float last_incli_angle = 0.0f;
int last_steer_angle = 0;

int16_t error_id = 0;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// ------------

Bala bala;

PID pid(angle_point, kp, ki, kd);
PID speed_pid(0, s_kp, s_ki, s_kd);

void error_loop() {
  while (1) {
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    M5.Lcd.setCursor(10, 40);
    M5.Lcd.printf("========= ERROR LOOP : IDX %d ==========", error_id); 
    delay(100);
  }
}


// the setup routine runs once when M5Stack starts up
void setup(){
  // declaration of freertos tasks
  portBASE_TYPE pub_task, sub_task;
  // set ip address
  agent_ip.fromString(AGENT_IP);

  // Initialize the M5Stack object
  M5.begin(true, false, false, false);

  Serial.begin(115200);
  M5.IMU.Init();

  int16_t x_offset, y_offset, z_offset;
  float angle_center;
  calibrationInit();

  // Button B reaction
  if (M5.BtnB.isPressed()) {
    calibrationGryo();
    calibration_mode = true;
  }

  // Button C reaction
  if (M5.BtnC.isPressed()) {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Charge mode");
    while (1) {
        if (M5.Power.isCharging()) {
            M5.Lcd.println("Start charging...");
            while(1) {
                if (M5.Power.isChargeFull())
                    M5.Lcd.println("Charge completed!");
                delay(5000);
            }
        }
        delay(500);
    }
  }

  // calibration of IMU
  calibrationGet(&x_offset, &y_offset, &z_offset, &angle_center);
  Serial.printf("x: %d, y: %d, z:%d, angle: %.2f", x_offset, y_offset, z_offset, angle_center);

  // get inclination angle of IMU and the main body
  angle_point = angle_center;
  pid.SetPoint(angle_point);

  // ------ micro-ros setup -------
  //set_microros_native_ethernet_udp_transports(mac, client_ip, agent_ip,
                                              //AGENT_PORT);
  set_microros_wifi_transports(WIFINAME, WIFIPASS, AGENT_IP, AGENT_PORT);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  error_id = 0;
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  error_id = 1;
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create publisher
  error_id = 2;
  RCCHECK(rclc_publisher_init_default(
      &publisher, 
      &node, 
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/m5bala2/data"));

  // create subscriber
  error_id = 3;
  RCCHECK(rclc_subscription_init_default(
      &incli_subscriber, 
      &node, 
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/m5bala2/incli_angle"));
  RCCHECK(rclc_subscription_init_default(
      &steer_subscriber, 
      &node, 
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "/m5bala2/steer_angle"));
  

  // create timer
  const unsigned int timer_timeout = 1000/FREQ; //[msec]
  error_id = 4;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  // publisher executor
  error_id = 5;
  RCCHECK(rclc_executor_init(&pub_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &timer)); 

  // subscriber executor
  error_id = 6;
  RCCHECK(rclc_executor_init(&sub_executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &incli_subscriber, &incli_msg,
                                         &incli_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &steer_subscriber, &steer_msg,
                                         &steer_subscription_callback, ON_NEW_DATA));
  // -----------------------------------------

  // --- FreeRTOS executors ---
  // PID task creator on FreeRTOS
  SemaphoreHandle_t i2c_mutex;;
  i2c_mutex = xSemaphoreCreateMutex();
  bala.SetMutex(&i2c_mutex);
  ImuTaskStart(x_offset, y_offset, z_offset, &i2c_mutex);
  // multi task executer of FreeRTOS
  xTaskCreatePinnedToCore(PIDTask, "pid_task", 4 * 1024, NULL, 4, NULL, 1);
  
  
  // create publisher and subscriber tasks
  pub_task = xTaskCreatePinnedToCore(
      pub_spin_task, "publisher_task",
      4*1024, NULL, 3, NULL, 1);

  // calibration message
  if (calibration_mode) {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("calibration mode");
  }
}

// the loop routine runs over and over again forever
void loop() {
  static uint32_t next_show_time = 0;
  //int32_t w_left, w_right;
  int16_t x_offset, y_offset, z_offset;
  float angle_center;

  vTaskDelay(pdMS_TO_TICKS(5));

  // update angles
  calibrationGet(&x_offset, &y_offset, &z_offset, &angle_center);
  angle_point = angle_center + incli_angle;
  pid.SetPoint(angle_point);
  //}

  
  // Display angle information
  if(millis() > next_show_time) {
    draw_waveform();
    next_show_time = millis() + 10;
  }

  M5.update();

  // print imu information
  M5.Lcd.setCursor(10, 30);
  M5.Lcd.printf("--- M5 stack ---");
  M5.Lcd.setCursor(10, 60);
  M5.Lcd.printf("left and right encoder : %d, %d", w_left, w_right);
  M5.Lcd.setCursor(10, 80);
  M5.Lcd.printf("left and right output : %d, %d", left_out, right_out);
  M5.Lcd.setCursor(10, 120);
  M5.Lcd.printf("cnt_display : %d", cnt_display);
  M5.Lcd.setCursor(10, 140);
  M5.Lcd.printf("incli_angle : %f, steer_angle : %d", incli_angle, steer_angle);
  M5.Lcd.setCursor(10, 160);
  M5.Lcd.printf("angle_point : %.2f", angle_point);
  M5.Lcd.setCursor(10, 190);
  M5.Lcd.printf("kp : %.2f, ki : %.2f, kd : %.2f", kp, ki, kd);

  // move forward
  if (M5.BtnA.wasPressed()) {
    angle_point += 1.0;
    pid.SetPoint(angle_point);  // set target inclination angle of main body
    M5.Lcd.setCursor(10, 100);
    M5.Lcd.println("Angle point : ");
    M5.Lcd.println(angle_point);
  }
  
  // stop
  if (M5.BtnB.wasPressed()) {
    
    calibrationGet(&x_offset, &y_offset, &z_offset, &angle_center);
    angle_point = angle_center;
    /*
    if (calibration_mode) {
      calibrationSaveCenterAngle(angle_point);
    }
    M5.Lcd.println("Calibration save : ");
    M5.Lcd.println(angle_point);
    */
    pid.SetPoint(angle_point);  // set target inclination angle of main body
    M5.Lcd.setCursor(10, 100);
    M5.Lcd.println("Angle point : ");
    M5.Lcd.println(angle_point);
  }

  // move backward
  if (M5.BtnC.wasPressed()) {
    angle_point -= 1.0;
    pid.SetPoint(angle_point);  // set target inclination angle of main body
    M5.Lcd.setCursor(10, 100);
    M5.Lcd.println("Angle point : ");
    M5.Lcd.println(angle_point);
  }

  last_incli_angle = incli_angle;
  last_steer_angle = steer_angle;
} 

static void PIDTask(void *arg) {
  float bala_angle;
  float motor_speed = 0;

  int16_t pwm_speed;
  int16_t pwm_output;
  int16_t pwm_angle;

  // additional offset
  int16_t speed;
  //int16_t forward_offset;
  //int16_t left_offset = 0;
  //int16_t right_offset = 0;
  //int16_t left_out, right_out;

  int32_t encoder = 0;
  int32_t last_encoder = 0;
  uint32_t last_ticks = 0;

  pid.SetOutputLimits(1023, -1023);
  pid.SetDirection(-1);
  
  speed_pid.SetIntegralLimits(40, -40);
  speed_pid.SetOutputLimits(1023, -1023);
  speed_pid.SetDirection(1);

  for(;;) { // infinite loop
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(5));

    // update pid param
    pid.UpdateParam(kp, ki, kd);
    speed_pid.UpdateParam(s_kp, s_ki, s_kd);

    // in imu task update, update freq is 200HZ
    bala_angle = getAngle();
    
    // Get motor encoder value
    //bala.UpdateEncoder();
    bala.GetEncoder(&w_left, &w_right); // deg

    encoder = bala.wheel_left_encoder + bala.wheel_right_encoder;
    
    // motor_speed filter
    motor_speed = 0.8 * motor_speed + 0.2 * (encoder - last_encoder);
    last_encoder = encoder;

    if(fabs(bala_angle) < 70) {
      pwm_angle = (int16_t)pid.Update(bala_angle);
      pwm_speed = (int16_t)speed_pid.Update(motor_speed);
      speed = 200;
      forward_offset = 0;
      /*
      if(){
        left_offset = speed;
        right_offset = -speed;
      }*/
      left_offset = speed * steer_angle/90;
      right_offset = speed * (-steer_angle/90);
      //pwm_output = pwm_speed + pwm_angle;
      left_out = pwm_speed + pwm_angle + left_offset;
      right_out = pwm_speed + pwm_angle + right_offset;

      if(pwm_output > 1023) { pwm_output = 1023; }
      if(pwm_output < -1023) { pwm_output = -1023; }
      
      // set speed of left and right motor
      bala.SetSpeed(left_out, right_out); // set speed of the left and right wheel
    } else {
      pwm_angle = 0;
      bala.SetSpeed(0, 0);
      bala.SetEncoder(0, 0);
      speed_pid.SetIntegral(0);
    }
  }
}

static void draw_waveform() {
	#define MAX_LEN 120
	#define X_OFFSET 100
	#define Y_OFFSET 95
	#define X_SCALE 3
	static int16_t val_buf[MAX_LEN] = {0};
	static int16_t pt = MAX_LEN - 1;
	val_buf[pt] = constrain((int16_t)(getAngle() * X_SCALE), -50, 50);

  if (--pt < 0) {
		pt = MAX_LEN - 1;
	}

	for (int i = 1; i < (MAX_LEN); i++) {
		uint16_t now_pt = (pt + i) % (MAX_LEN);
		M5.Lcd.drawLine(i + X_OFFSET, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, i + 1 + X_OFFSET, val_buf[(now_pt + 2) % MAX_LEN] + Y_OFFSET, TFT_BLACK);
		if (i < MAX_LEN - 1) {
			M5.Lcd.drawLine(i + X_OFFSET, val_buf[now_pt] + Y_OFFSET, i + 1 + X_OFFSET, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, TFT_GREEN);
    }
	}
  //M5.Lcd.println("HOGE BALA2");
}


// ros timer callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if(timer != NULL){
    cnt_display ++;
    char buffer[50];
    static int cnt = 0;
    sprintf(buffer, "hello world: %d, sys_clk: %d", cnt++, xTaskGetTickCount());
    
    //msg.data = micro_ros_string_utilities_set(msg.data, buffer);
    msg.data.size = 50;
    msg.data.data = buffer;

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL))
  }
}

// ros subscription
void incli_subscription_callback(const void *msgin){
  //*steer_msg = (const std_msgs__msg__Int16 *)msgin;  
  const std_msgs__msg__Float32 * incli_msg = (const std_msgs__msg__Float32 *)msgin;  
  incli_angle = incli_msg->data; // ! ERROR POINT
}

void steer_subscription_callback(const void *msgin){
  //*steer_msg = (const std_msgs__msg__Int16 *)msgin;  
  const std_msgs__msg__Int16 * steer_msg = (const std_msgs__msg__Int16 *)msgin;  
  steer_angle = steer_msg->data; // ! ERROR POINT
}

// node executor
static void pub_spin_task(void *p){
  //UNUSED(p);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    /*
    RCCHECK(rclc_executor_spin(&executor));
    vTaskDelayUntil(&xLastWakeTime, 10);
    */
    RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(5)));
    RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(5)));
  }
}

static void sub_spin_task(void *p){
  //UNUSED(p);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(5)));
  }
}

