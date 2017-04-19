#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/common_msgs/String64.hpp>
#include <core/sensor_msgs/RPY_f32.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/sensor_publisher/Publisher.hpp>
#include <core/led/Publisher.hpp>
#include <core/led/Subscriber.hpp>
#include <core/madgwick/Madgwick.hpp>
#include <core/balancing_robot/BalanceControlNode.hpp>
#include <core/balancing_robot/VelocityControlNode.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/L3GD20H_driver/L3GD20H.hpp>
#include <core/LSM303D_driver/LSM303D.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using Vector3_i16_Publisher = core::sensor_publisher::Publisher<core::common_msgs::Vector3_i16>;

// --- NODES ------------------------------------------------------------------
core::led::Publisher led_publisher("led_publisher", core::os::Thread::PriorityEnum::LOWEST);
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
Vector3_i16_Publisher gyro_publisher("gyro_publisher", module.gyro, core::os::Thread::PriorityEnum::NORMAL + 1);
Vector3_i16_Publisher acc_publisher("acc_publisher", module.acc, core::os::Thread::PriorityEnum::NORMAL + 1);
Vector3_i16_Publisher mag_publisher("mag_publisher", module.mag);
core::madgwick::Madgwick madgwick_filter("madgwick");
core::balancing_robot::BalanceControlNode  balance_node("balance");
core::balancing_robot::VelocityControlNode velocity_node("velocity");

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::PublisherConfiguration led_publisher_configuration;
core::led::SubscriberConfiguration led_subscriber_configuration;
core::sensor_publisher::Configuration gyro_publisher_configuration;
core::sensor_publisher::Configuration acc_publisher_configuration;
core::sensor_publisher::Configuration mag_publisher_configuration;
core::madgwick::MadgwickConfiguration madgwick_filter_configuration;
core::balancing_robot::BalanceControlNodeConfiguration balance_control_configuration;
core::balancing_robot::VelocityControlNodeConfiguration velocity_control_configuration;

// --- MAIN -------------------------------------------------------------------
extern "C" {
   int
   main()
   {
      module.initialize();

      // Led publisher node
      led_publisher_configuration.topic = "led";
      led_publisher_configuration.led   = 1;
      led_publisher.setConfiguration(led_publisher_configuration);
      module.add(led_publisher);

      // Led subscriber node
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);
      module.add(led_subscriber);

      // Sensor nodes
      gyro_publisher_configuration.topic = "gyro";
      gyro_publisher.setConfiguration(gyro_publisher_configuration);
      module.add(gyro_publisher);

      acc_publisher_configuration.topic  = "acc";
      acc_publisher.setConfiguration(acc_publisher_configuration);
      module.add(acc_publisher);

      mag_publisher_configuration.topic  = "mag";
      mag_publisher.setConfiguration(mag_publisher_configuration);
      module.add(mag_publisher);

      // IMU Filter
      madgwick_filter_configuration.topicGyro = gyro_publisher_configuration.topic;
      madgwick_filter_configuration.topicAcc  = acc_publisher_configuration.topic;
      madgwick_filter_configuration.topicMag  = mag_publisher_configuration.topic;
      madgwick_filter_configuration.topic     = "imu";
      madgwick_filter_configuration.frequency = 50.0f;
      madgwick_filter.setConfiguration(madgwick_filter_configuration);
      module.add(madgwick_filter);

      // Balance control
      balance_control_configuration.left_pwm_topic   = "pwm_left";
      balance_control_configuration.right_pwm_topic  = "pwm_right";
      balance_control_configuration.attitude_topic   = "imu";
      balance_control_configuration.setpoint_topic   = "tilt";
      balance_control_configuration.parameters_topic = "pid_params";
      balance_control_configuration.period = 20;
      balance_control_configuration.offset = 0.0f;
      balance_control_configuration.kp     = -450.0f / 4096.0f;
      balance_control_configuration.ti     = 0.2f;
      balance_control_configuration.td     = 0.02f;
      balance_node.setConfiguration(balance_control_configuration);
      module.add(balance_node);

      // Velocity control
      velocity_control_configuration.left_encoder_topic  = "encoder_left";
      velocity_control_configuration.right_encoder_topic = "encoder_right";
      velocity_control_configuration.setpoint_topic      = "velocity_sp";
      velocity_control_configuration.output_topic        = "tilt";
      velocity_control_configuration.wheel_radius        = 0.155f / 2.0f;
      velocity_control_configuration.period = 50;
      velocity_control_configuration.kp     = 2.0f;
      velocity_control_configuration.ti     = 0.0f;
      velocity_control_configuration.td     = -0.1f;
      velocity_node.setConfiguration(velocity_control_configuration);
      module.add(velocity_node);

      // ... and let's play!
      module.setup();
      module.run();

      // Is everything going well?
      for (;;) {
         if (!module.isOk()) {
            module.halt("This must not happen!");
         }

         core::os::Thread::sleep(core::os::Time::ms(500));
      }

      return core::os::Thread::OK;
   }   // main
}
