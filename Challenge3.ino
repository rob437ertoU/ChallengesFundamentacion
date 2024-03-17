#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_publisher_t publisher_pot;
rcl_publisher_t publisher_voltage;
rcl_subscription_t subscription_pwm;
std_msgs_msg_Float32 msg_pot;
std_msgs_msg_Float32 msg_pwm;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;  // Changed timer name to timer_1
rcl_timer_t timer_2;  // Added timer_2

#define LED_PIN 13
#define POT_PIN 36
//#define PWM_PIN 15
#define LED2_PIN 15

#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 5000



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback_1(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int32_t pot_value = analogRead(POT_PIN);
    msg_pot.data = pot_value;
    RCSOFTCHECK(rcl_publish(&publisher_pot, &msg_pot, NULL));
  }
}

void timer_callback_2(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int32_t pot_value = analogRead(POT_PIN);
    msg_pot.data = pot_value;
    RCSOFTCHECK(rcl_publish(&publisher_pot, &msg_pot, NULL));

    float voltage = pot_value * (3.3 / 4095.0);
    msg_pot.data = static_cast<float_t>(voltage);
    RCSOFTCHECK(rcl_publish(&publisher_voltage, &msg_pot, NULL));
  }
}

void subscription_callback(const void *msgin){
  const std_msgs_msgFloat32 * msg = static_cast<const std_msgsmsg_Float32 *>(msgin);
  int pwm_value = (int)(msg->data*255.0f/100.0f);                                               //Para hacer cast manera rapida
  ledcWrite(PWM_CHANNEL, pwm_value);
  /*if (pwm_value <= 0 || pwm_value > 100){
    ledcWrite(PWM_CHANNEL, 0);
  //} else {
    ledcWrite(PWM_CHANNEL, static_cast<int>(pwm_value * 2.55));  // Map PWM value to 0-255 range
  } */
} 

void setup() {
  ledcSetup(PWM_CHANNEL,PWM_FREQUENCY, PWM_RESOLUTION); //CONFIGURE LED PWM FUNCT
  ledcAttachPin(LED2_PIN, PWM_CHANNEL);                //ATACH THE CHANNERL TO THE GPIO 
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  //pinMode(LED2_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);  
  //digitalWrite(LED2_PIN, HIGH); 
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher_pot,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_esp32/pot"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_voltage,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_esp32/voltage"));

  RCCHECK(rclc_subscription_init_default(
    &subscription_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/micro_ros_esp32/pwm_duty_cycle"));

  const unsigned int timer_1_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_1,
    &support,
    RCL_MS_TO_NS(timer_1_timeout),
    timer_callback_1));

  const unsigned int timer_2_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_2,
    &support,
    RCL_MS_TO_NS(timer_2_timeout),
    timer_callback_2));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription_pwm, &msg_pwm, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));

  msg_pot.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}