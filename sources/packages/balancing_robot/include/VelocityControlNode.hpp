#pragma once

#include <Core/MW/Publisher.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreNode.hpp>
#include <Core/MW/Callback.hpp>

#include <Configuration.hpp>

#include <common_msgs/Float32.hpp>
#include <differential_drive_msgs/Velocity.hpp>
#include <sensor_msgs/Delta_f32.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>
#include <balancing_robot/VelocityControlNodeConfiguration.hpp>
#include <pid/PID.hpp>

namespace balancing_robot {
	class VelocityControlNode:
		public Core::MW::CoreNode
	{
public:
		VelocityControlNode(
				const char*                    name,
				Core::MW::Thread::PriorityEnum priority = Core::MW::Thread::PriorityEnum::NORMAL
		);
		virtual
		~VelocityControlNode();

public:
		VelocityControlNodeConfiguration configuration;

private:
		struct {
			float linear;
			float angular;
		}
		_setpoint;

		float _left_encoder;
		float _right_encoder;
		PID   _pid;

private:
		bool
		onPrepareMW();

		bool
		onLoop();

		static bool
		setpoint_callback(
				const differential_drive_msgs::Velocity& msg,
				Core::MW::Node*                          node
		);

		static bool
		left_encoder_callback(
				const sensor_msgs::Delta_f32& msg,
				Core::MW::Node*               node
		);

		static bool
		right_encoder_callback(
				const sensor_msgs::Delta_f32& msg,
				Core::MW::Node*               node
		);


private:
		Core::MW::Subscriber<differential_drive_msgs::Velocity, 5> _setpoint_subscriber;
		Core::MW::Subscriber<sensor_msgs::Delta_f32, 5>            _left_encoder_subscriber;
		Core::MW::Subscriber<sensor_msgs::Delta_f32, 5>            _right_encoder_subscriber;
		Core::MW::Publisher<common_msgs::Float32> _output_publisher;
	};
}
