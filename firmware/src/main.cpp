#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32_multi_array.h>

rcl_publisher_t wheels_state_pub;
rcl_subscription_t wheels_command_sub;

// Units are rad/s
static std_msgs__msg__Float32MultiArray wheels_state_msg;
static std_msgs__msg__Float32MultiArray wheels_command_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

uint32_t main_counter = 0;

enum MicrosAgentState
{
  CONNECTING,
  CONNECTED,
  DISCONNECTING,
} microros_agent_state;

enum Motors
{
  Left,
  Right,
  MOTORS_COUNT
};

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
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      digitalWrite(2, HIGH);                                                                                           \
    }                                                                                                                  \
  }

void error_loop()
{
  while (1)
  {
    delay(100);
    digitalWrite(2, !digitalRead(2));
  }
}

void fill_wheels_msg(std_msgs__msg__Float32MultiArray* msg)
{
  static float data[MOTORS_COUNT] = {
    0,
    0,
  };
  msg->data.capacity = MOTORS_COUNT;
  msg->data.size = MOTORS_COUNT;
  msg->data.data = (float*)data;
}

void wheels_command_callback(const void* message)
{
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)message;

  digitalWrite(2, !digitalRead(2));

  if (msg != NULL and msg->data.size == MOTORS_COUNT)
  {
    for (auto i = 0u; i < MOTORS_COUNT; ++i)
    {
      // Check if values are correct
      if (isnan(msg->data.data[i]))
      {
        wheels_command_msg.data.data[i] = msg->data.data[i];
        // Take velocities
      }
      else
      {
        // Stop motors
      }
    }

    // Update motor driver velocities

    // Update motor watchdog
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
  RCCHECK(rclc_publisher_init_default(&wheels_state_pub, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "wheel_state"));

  RCCHECK(rclc_subscription_init_default(
      &wheels_command_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "wheel_command"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &wheels_command_sub, &wheels_command_msg, &wheels_command_callback,
                                         ON_NEW_DATA));
}

void destroy_microros_entities()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&wheels_state_pub, &node);
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
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
  }
}

bool is_microros_agent_connected()
{
  return RMW_RET_OK == rmw_uros_ping_agent(100, 1);
}

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  fill_wheels_msg(&wheels_state_msg);
  fill_wheels_msg(&wheels_command_msg);
}

void control_loop_one_tick()
{
  // 100 * 1ms is 10Hz
  if (main_counter % 10)
  {
    if (!is_microros_agent_connected())
    {
      microros_agent_state = DISCONNECTING;

      // STOP MOTORS

      return;
    }
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  // 10 * 1ms is 100Hz control
  if (main_counter % 10)
  {
    fill_wheels_msg(&wheels_state_msg);
    // Take encoder readings in rad/s

    // Simple motor simulation
    const float Kp = 0.0000001;
    const float error_0 = wheels_state_msg.data.data[0] - wheels_command_msg.data.data[0];
    wheels_state_msg.data.data[0] += Kp * error_0;

    const float error_1 = wheels_state_msg.data.data[1] - wheels_command_msg.data.data[1];
    wheels_state_msg.data.data[1] += Kp * error_1;

    if (isnan(wheels_state_msg.data.data[0]) || isnan(wheels_state_msg.data.data[1]))
    {
      wheels_state_msg.data.data[0] = 0.0;
      wheels_state_msg.data.data[1] = 0.0;
    }

    RCSOFTCHECK(rcl_publish(&wheels_state_pub, &wheels_state_msg, NULL));
  }
}

void loop()
{
  switch (microros_agent_state)
  {
    case CONNECTING:
      wait_for_microros_agent();
      microros_agent_state = CONNECTED;
      create_microros_entities();
      break;
    case CONNECTED:
      control_loop_one_tick();
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