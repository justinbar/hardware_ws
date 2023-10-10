#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED_PIN 13       // the led is already equip on to the board on pin 13
#define MOTOR_1_PIN_1 2  // Replace with the appropriate GPIO pin for motor 1 control
#define MOTOR_1_PIN_2 3  // Replace with the appropriate GPIO pin for motor 1 control
#define MOTOR_2_PIN_1 4  // Replace with the appropriate GPIO pin for motor 2 control
#define MOTOR_2_PIN_2 5  // Replace with the appropriate GPIO pin for motor 2 control
#define MOTOR_1_PWM_PIN 6 // Replace with the appropriate PWM pin for motor 1 speed control
#define MOTOR_2_PWM_PIN 7 // Replace with the appropriate PWM pin for motor 2 speed control

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Twist message callback
// Twist message callback
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  // Control motors for forward and backward movement
  if (linear_x > 0 )
  {
    // Move motors forward
    digitalWrite(MOTOR_1_PIN_1, HIGH);
    digitalWrite(MOTOR_1_PIN_2, LOW);
    digitalWrite(MOTOR_2_PIN_1, HIGH);
    digitalWrite(MOTOR_2_PIN_2, LOW);
  }
  else if (linear_x < 0)
  {
    // Move motors backward
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_1_PIN_2, HIGH);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, HIGH);
  }
  else
  {
    // Stop motors
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_1_PIN_2, LOW);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, LOW);
  }

  // Control motors for left and right movement
  if (angular_z > 0)
  {
    // Move left
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_1_PIN_2, HIGH);
    digitalWrite(MOTOR_2_PIN_1, HIGH);
    digitalWrite(MOTOR_2_PIN_2, LOW);
  }
  else if (angular_z < 0)
  {
    // Move right
    digitalWrite(MOTOR_1_PIN_1, HIGH);
    digitalWrite(MOTOR_1_PIN_2, LOW);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, HIGH);
  }
  else if (linear_x == 0 && angular_z == 0)
  {
    // Stop motors if there is no movement
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_1_PIN_2, LOW);
    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, LOW);
  }

  // If velocity in x direction is 0, turn off LED; if 1, turn on LED
  digitalWrite(LED_PIN, (linear_x != 0 || angular_z != 0) ? HIGH : LOW);
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
  pinMode(MOTOR_2_PIN_1, OUTPUT);
  pinMode(MOTOR_2_PIN_2, OUTPUT);
  pinMode(MOTOR_1_PWM_PIN, OUTPUT);
  pinMode(MOTOR_2_PWM_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(1000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel")); 

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  analogWrite(MOTOR_1_PWM_PIN, 210); // Set motor 1 
  analogWrite(MOTOR_2_PWM_PIN, 210); // Set motor 2
}

void loop()
{
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); //100 millisecond
}
