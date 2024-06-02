#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32_multi_array.h>

#include "PID.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      error_loop();                                                                                                    \
    }                                                                                                                  \
  }

#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK)) {}                                                                                    \
  }

enum Motors
{
  Left,
  Right,
  MOTORS_COUNT
};

#define MOTORS_COUNT 2

enum MicrosAgentState
{
  CONNECTING,
  CONNECTED,
  DISCONNECTING,
};

void error_loop();
void fill_wheels_msg(std_msgs__msg__Float32MultiArray* msg);
void create_microros_entities();
void destroy_microros_entities();
void wait_for_microros_agent();
bool is_microros_agent_connected();
void microros_loop_tick(uint32_t main_counter);
void wheels_command_callback(const void* message);

uint32_t main_counter = 0;

constexpr int LEFT_ENCODER_A = 4;
constexpr int LEFT_ENCODER_B = 34;

constexpr int RIGHT_ENCODER_A = 21;
constexpr int RIGHT_ENCODER_B = 19;

// Pololu
constexpr int RIGHT_MOTOR_B = 35;
constexpr int RIGHT_MOTOR_A = 32;

constexpr int LEFT_MOTOR_DIR = 32;
constexpr int LEFT_MOTOR_PWM = 27;
constexpr int LEFT_MOTOR_PWM_CHANNEL = 0;

constexpr int RIGHT_MOTOR_DIR = 0;
constexpr int RIGHT_MOTOR_PWM = 0;
constexpr int RIGHT_MOTOR_PWM_CHANNEL = 1;

volatile static int32_t left_encoder_ticks = 0;
volatile static int32_t last_left_encoder_ticks = 0;

volatile static int32_t right_encoder_ticks = 0;
volatile static int32_t last_right_encoder_ticks = 0;

volatile static bool right_motor_direction = false;
volatile static bool left_motor_direction = false;

rcl_publisher_t wheels_position_state_pub;
rcl_publisher_t wheels_velocity_state_pub;
rcl_subscription_t wheels_command_sub;

// Units are rad/s
static std_msgs__msg__Float32MultiArray wheels_position_state_msg;
static std_msgs__msg__Float32MultiArray wheels_velocity_state_msg;
static std_msgs__msg__Float32MultiArray wheels_command_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

MicrosAgentState microros_agent_state;

TickType_t main_last_time_wake;
const TickType_t main_loop_period = 20;
const TickType_t control_loop_period = 20;

PID controller_left(1.0, 0.6, 0.0, control_loop_period * 0.001);
PID controller_right(1.0, 0.6, 0.0, control_loop_period * 0.001);

QueueHandle_t motor_commands_queue = NULL;
QueueHandle_t motor_velocity_states_queue = NULL;
volatile float motor_commands[MOTORS_COUNT];

void IRAM_ATTR encoder_a_left_interrupt()
{
  const uint8_t A = digitalRead(LEFT_ENCODER_A);
  const uint8_t B = digitalRead(LEFT_ENCODER_B);
  if ((A == 1 && B == 0) || (A == 0 && B == 1))
  {
    --left_encoder_ticks;
  }
  else
  {
    ++left_encoder_ticks;
  }
}

void IRAM_ATTR encoder_b_left_interrupt()
{
  const uint8_t A = digitalRead(LEFT_ENCODER_A);
  const uint8_t B = digitalRead(LEFT_ENCODER_B);
  if ((A == 1 && B == 1) || (A == 0 && B == 0))
  {
    --left_encoder_ticks;
  }
  else
  {
    ++left_encoder_ticks;
  }
}

void IRAM_ATTR encoder_right_interrupt()
{
  if (digitalRead(RIGHT_ENCODER_B))
  {
    --right_encoder_ticks;
  }
  else
  {
    ++right_encoder_ticks;
  }
}

void control_tick(TimerHandle_t xTimer)
{
  const float dt_s = control_loop_period * 0.001;
  float motor_commands_received_values[MOTORS_COUNT];

  int32_t right_encoder_tick_diff = right_encoder_ticks - last_right_encoder_ticks;
  int32_t left_encoder_tick_diff = left_encoder_ticks - last_left_encoder_ticks;

  float left_encoder_tick_per_sec = (float)(left_encoder_tick_diff) / dt_s;
  float left_encoder_tick_per_sec_for_pid = left_encoder_tick_per_sec;
  float right_encoder_tick_per_sec = (float)(right_encoder_tick_diff) / dt_s;
  float right_encoder_tick_per_sec_for_pid = right_encoder_tick_per_sec;

  if (left_encoder_tick_per_sec_for_pid < 0.0)
  {
    left_encoder_tick_per_sec_for_pid *= -1;
  }

  if (right_encoder_tick_per_sec_for_pid < 0.0)
  {
    right_encoder_tick_per_sec_for_pid *= -1;
  }
  controller_left.setProcessValue(left_encoder_tick_per_sec_for_pid);
  controller_right.setProcessValue(right_encoder_tick_per_sec_for_pid);

  // Update Set Value
  if (xQueueReceive(motor_commands_queue, &(motor_commands_received_values), (TickType_t)1) == pdPASS)
  {
    if (motor_commands_received_values[0] < 0)
    {
      left_motor_direction = 1;
      motor_commands_received_values[0] *= -1;
    }
    else if (motor_commands_received_values[0] > 0)
    {
      left_motor_direction = 0;
    }

    if (motor_commands_received_values[1] < 0)
    {
      right_motor_direction = 1;
      motor_commands_received_values[1] *= -1;
    }
    else if (motor_commands_received_values[1] > 0)
    {
      right_motor_direction = 0;
    }

    controller_left.setSetPoint(motor_commands_received_values[0]);
    controller_right.setSetPoint(motor_commands_received_values[1]);
  }

  digitalWrite(LEFT_MOTOR_DIR, left_motor_direction);
  digitalWrite(RIGHT_MOTOR_DIR, left_motor_direction);

  float left_computed_value = controller_left.compute();
  float right_computed_value = controller_right.compute();
  digitalWrite(2, !digitalRead(2));

  ledcWrite(LEFT_MOTOR_PWM_CHANNEL, left_computed_value * 4095);
  ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, right_computed_value * 4095);

  last_left_encoder_ticks = left_encoder_ticks;
  last_right_encoder_ticks = right_encoder_ticks;

  float motor_velocity_states[MOTORS_COUNT];
  motor_velocity_states[0] = left_encoder_tick_per_sec;
  motor_velocity_states[1] = 0.0;

  xQueueSend(motor_velocity_states_queue, (void*)motor_velocity_states, (TickType_t)1);
}

void pid_setup()
{
  controller_left.setInputLimits(0, 100000);
  controller_left.setOutputLimits(0.0, 1.0);
  controller_left.setMode(1);
  controller_left.setSetPoint(0);

  controller_right.setInputLimits(0, 1000);
  controller_right.setOutputLimits(0.0, 1.0);
  controller_right.setMode(1);
  controller_right.setSetPoint(0);
}

void setup()
{
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);

  pid_setup();

  // pinMode(LEFT_ENCODER_A, INPUT);
  // pinMode(RIGHT_ENCODER_A, INPUT);

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  // pinMode(LEFT_MOTOR_PWM_BACKWARD, OUTPUT);
  // pinMode(RIGHT_MOTOR_PWM_FORWARD, OUTPUT);
  // pinMode(RIGHT_MOTOR_PWM_BACKWARD, OUTPUT);

  ledcSetup(LEFT_MOTOR_PWM_CHANNEL, 20000, 12);
  // ledcSetup(LEFT_MOTOR_PWM_BACKWARD_CHANNEL, 20000, 16);

  ledcAttachPin(LEFT_MOTOR_PWM, LEFT_MOTOR_PWM_CHANNEL);
  // ledcAttachPin(RIGHT_MOTOR_PWM_BACKWARD, RIGHT_MOTOR_PWM_BACKWARD_CHANNEL);

  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  // pinMode(RIGHT_MOTOR_DIR , OUTPUT);

  attachInterrupt(LEFT_ENCODER_A, encoder_a_left_interrupt, CHANGE);
  attachInterrupt(LEFT_ENCODER_B, encoder_b_left_interrupt, CHANGE);
  // attachInterrupt(RIGHT_ENCODER_A, encoder_right_interrupt, RISING);

  TimerHandle_t control_timer;
  control_timer = xTimerCreate("control_tick", pdMS_TO_TICKS(control_loop_period), pdTRUE, 0, control_tick);
  xTimerStart(control_timer, 0);

  motor_commands_queue = xQueueCreate(5, sizeof(float) * MOTORS_COUNT);
  motor_velocity_states_queue = xQueueCreate(5, sizeof(float) * MOTORS_COUNT);

  fill_wheels_msg(&wheels_command_msg);
  fill_wheels_msg(&wheels_position_state_msg);
  fill_wheels_msg(&wheels_velocity_state_msg);
  set_microros_serial_transports(Serial);
}

void loop()
{
  for (;;)
  {
    vTaskDelayUntil(&main_last_time_wake, main_loop_period);

    switch (microros_agent_state)
    {
      case CONNECTING:
        wait_for_microros_agent();
        microros_agent_state = CONNECTED;
        create_microros_entities();
        break;
      case CONNECTED:
        microros_loop_tick(main_counter);
        break;
      case DISCONNECTING:
        destroy_microros_entities();
        microros_agent_state = CONNECTING;
        break;
      default:
        break;
    }

    ++main_counter;
  }
}

void error_loop()
{
  while (1)
  {
    delay(100);
    // digitalWrite(2, !digitalRead(2));
  }
}

void fill_wheels_msg(std_msgs__msg__Float32MultiArray* msg)
{
  msg->data.capacity = MOTORS_COUNT;
  msg->data.size = MOTORS_COUNT;
  msg->data.data = (float*)malloc(msg->data.capacity * sizeof(float));
}

void wheels_command_callback(const void* message)
{
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)message;

  if (msg != NULL and msg->data.size == MOTORS_COUNT)
  {
    uint8_t bad_command_flag = 0;
    for (int i = 0; i < MOTORS_COUNT; ++i)
    {
      if (isnan(msg->data.data[i]))
      {
        bad_command_flag = 1;
        break;
      }
      wheels_command_msg.data.data[i] = msg->data.data[i];
      motor_commands[i] = wheels_command_msg.data.data[i];
    }

    if (bad_command_flag)
    {
      for (int i = 0; i < MOTORS_COUNT; ++i)
      {
        motor_commands[i] = 0.0;
      }
    }

    xQueueSend(motor_commands_queue, (void*)motor_commands, (TickType_t)1);
  }
}

void create_microros_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_diffdrive_robot_firmware", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&wheels_position_state_pub, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                      "wheel_position_state"));

  RCCHECK(rclc_publisher_init_default(&wheels_velocity_state_pub, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                      "wheel_velocity_state"));

  RCCHECK(rclc_subscription_init_default(&wheels_command_sub, &node,
                                         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                         "wheel_velocity_command"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &wheels_command_sub, &wheels_command_msg, &wheels_command_callback,
                                         ON_NEW_DATA));
}

void destroy_microros_entities()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&wheels_position_state_pub, &node);
  rcl_publisher_fini(&wheels_velocity_state_pub, &node);
  rcl_subscription_fini(&wheels_command_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void wait_for_microros_agent()
{
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1))
  {
    delay(500);
    // digitalWrite(2, HIGH);
    delay(500);
    // digitalWrite(2, LOW);
  }
}

bool is_microros_agent_connected()
{
  return RMW_RET_OK == rmw_uros_ping_agent(100, 1);
}

void microros_loop_tick(uint32_t main_counter)
{
  // 100 * 1ms is 10Hz
  if (main_counter % 10)
  {
    if (!is_microros_agent_connected())
    {
      microros_agent_state = DISCONNECTING;

      motor_commands[0] = 0.0;
      motor_commands[1] = 0.0;
      xQueueSend(motor_commands_queue, (void*)motor_commands, (TickType_t)1);
      return;
    }
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  // 10 * 1ms is 100Hz
  if (main_counter % 100)
  {
    wheels_position_state_msg.data.data[0] = left_encoder_ticks;
    wheels_position_state_msg.data.data[1] = right_encoder_ticks;

    RCSOFTCHECK(rcl_publish(&wheels_position_state_pub, &wheels_position_state_msg, NULL));

    float motor_velocity_state_received_values[MOTORS_COUNT];

    if (xQueueReceive(motor_velocity_states_queue, &(motor_velocity_state_received_values), (TickType_t)1) == pdPASS)
    {
      wheels_velocity_state_msg.data.data[0] = motor_velocity_state_received_values[0];
      wheels_velocity_state_msg.data.data[1] = motor_velocity_state_received_values[1];
      RCSOFTCHECK(rcl_publish(&wheels_velocity_state_pub, &wheels_velocity_state_msg, NULL));
    }
  }
}
