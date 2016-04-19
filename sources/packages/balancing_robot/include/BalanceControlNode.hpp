#pragma once

#include <Core/MW/Publisher.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreNode.hpp>
#include <Core/MW/Callback.hpp>

#include <Configuration.hpp>

#include <common_msgs/Float32.hpp>
#include <sensor_msgs/RPY_f32.hpp>
#include <sensor_msgs/Delta_f32.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>
#include <pid_msgs/PIDParameters.hpp>
#include <balancing_robot/BalanceControlNodeConfiguration.hpp>
#include <pid/PID.hpp>


namespace balancing_robot {
	class BalanceControlNode:
		public Core::MW::CoreNode
	{
public:
		BalanceControlNode(
				const char*                    name,
				Core::MW::Thread::PriorityEnum priority = Core::MW::Thread::PriorityEnum::NORMAL
		);
		virtual
		~BalanceControlNode();

public:
		BalanceControlNodeConfiguration configuration;

private:
		float _tilt;
		float _setpoint;
		PID   _pid;
		pid_msgs::PIDParameters _pidParameters;

private:
		bool
		onPrepareMW();

		bool
		onLoop();

		static bool
		attitude_callback(
				const sensor_msgs::RPY_f32& msg,
				Core::MW::Node*             node
		);

		static bool
		setpoint_callback(
				const common_msgs::Float32& msg,
				Core::MW::Node*             node
		);

		static bool
		parameters_callback(
				const pid_msgs::PIDParameters& msg,
				Core::MW::Node*             node
		);

private:
		Core::MW::Subscriber<sensor_msgs::RPY_f32, 5>    _attitude_subscriber;
		Core::MW::Subscriber<common_msgs::Float32, 5>    _setpoint_subscriber;
		Core::MW::Subscriber<pid_msgs::PIDParameters, 5>    _pid_parameters_subscriber;

		Core::MW::Publisher<actuator_msgs::Setpoint_f32> _left_pwm_publisher;
		Core::MW::Publisher<actuator_msgs::Setpoint_f32> _right_pwm_publisher;
	};
}
