#include <Configuration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <common_msgs/Led.hpp>
#include <common_msgs/String64.hpp>
#include <sensor_msgs/RPY_f32.hpp>
//@@MESSAGE_HEADERS@@//

// --- NODES ------------------------------------------------------------------
#include <sensors/Publisher.hpp>
#include <led/Subscriber.hpp>
#include <madgwick/Madgwick.hpp>
#include <balancing_robot/BalanceControlNode.hpp>
#include <balancing_robot/VelocityControlNode.hpp>
//@@NODE_HEADERS@@//

// --- BOARD IMPL -------------------------------------------------------------
#include <sensors/L3GD20H.hpp>
#include <sensors/LSM303D.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using Vector3_i16_Publisher = sensors::Publisher<common_msgs::Vector3_i16>;

// --- NODES ------------------------------------------------------------------
Vector3_i16_Publisher gyro_publisher("gyro_publisher", module.gyro, Core::MW::Thread::PriorityEnum::HIGH);
Vector3_i16_Publisher acc_publisher("acc_publisher", module.acc, Core::MW::Thread::PriorityEnum::HIGH);
Vector3_i16_Publisher mag_publisher("mag_publisher", module.mag);

led::Subscriber led_subscriber("led_subscriber", Core::MW::Thread::PriorityEnum::LOWEST);
madgwick::Madgwick   madgwick_filter("madgwick");
balancing_robot::BalanceControlNode  balance_node("balance");
balancing_robot::VelocityControlNode velocity_node("velocity");
//@@NODES@@//

// --- MAIN -------------------------------------------------------------------
extern "C" {
	int
	main()
	{
		module.initialize();

		// Nodes configuration
		led_subscriber.configuration["topic"] = "led";

		gyro_publisher.configuration["topic"] = "gyro";
		acc_publisher.configuration["topic"]  = "acc";
		mag_publisher.configuration["topic"]  = "mag";

		madgwick_filter.configuration["topicGyro"] = gyro_publisher.configuration["topic"];
		madgwick_filter.configuration["topicAcc"]  = acc_publisher.configuration["topic"];
		madgwick_filter.configuration["topicMag"]  = mag_publisher.configuration["topic"];
		madgwick_filter.configuration["topic"]     = "imu";
		madgwick_filter.configuration["frequency"] = 50.0f;

		balance_node.configuration["left_pwm_topic"]  = "pwm_left";
		balance_node.configuration["right_pwm_topic"] = "pwm_right";
		balance_node.configuration["attitude_topic"]  = "imu";
		balance_node.configuration["setpoint_topic"]  = "tilt";
		balance_node.configuration["parameters_topic"]  = "pid_params";
		balance_node.configuration.period = 20;
		balance_node.configuration.offset = 0.4;

		velocity_node.configuration["left_encoder_topic"]  = "encoder_left";
		velocity_node.configuration["right_encoder_topic"] = "encoder_right";
		velocity_node.configuration["setpoint_topic"]      = "velocity_sp";
		velocity_node.configuration["output_topic"]        = "tilt";
		velocity_node.configuration["wheel_radius"]        = 0.155f / 2;
		velocity_node.configuration.period = 50;

		//@@NODE_CONFIGURATIONS@@//

		// Add nodes to the module...
		module.add(led_subscriber);
		module.add(gyro_publisher);
		module.add(acc_publisher);
		module.add(mag_publisher);
		module.add(madgwick_filter);
		module.add(balance_node);
		module.add(velocity_node);
		//@@NODE_MANAGEMENT@@//

		// ... and let's play!
		module.setup();
		module.run();

		// Is everything going well?
		for (;;) {
			if (!module.isOk()) {
				module.halt("This must not happen!");
			}

			Core::MW::Thread::sleep(Core::MW::Time::ms(500));
		}

		return Core::MW::Thread::OK;
	} // main
}
