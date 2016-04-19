#include <Module.hpp>

#include <balancing_robot/BalanceControlNode.hpp>
#include <Core/Utils/Math/Constants.hpp>
#include <Core/Utils/Math/Conversions.hpp>
#include <sensor_msgs/RPY_f32.hpp>

namespace balancing_robot {
	BalanceControlNode::BalanceControlNode(
			const char*                    name,
			Core::MW::Thread::PriorityEnum priority
	) :
		CoreNode::CoreNode(name, priority),
		_tilt(0.0f),
		_setpoint(0.0f)
	{
		_workingAreaSize = 512;
	}

	BalanceControlNode::~BalanceControlNode()
	{
		teardown();
	}

	bool
	BalanceControlNode::onPrepareMW()
	{
		_attitude_subscriber.set_callback(BalanceControlNode::attitude_callback);
		_setpoint_subscriber.set_callback(BalanceControlNode::setpoint_callback);
		_pid_parameters_subscriber.set_callback(BalanceControlNode::parameters_callback);

		this->subscribe(_attitude_subscriber, configuration.attitude_topic);
		this->subscribe(_setpoint_subscriber, configuration.setpoint_topic);

		this->advertise(_left_pwm_publisher, configuration.left_pwm_topic);
		this->advertise(_right_pwm_publisher, configuration.right_pwm_topic);

		_pid.config(-250.0 / 4096.0, 0.2, 0.02, 0.02, -1.0, 1.0);
		_pid.set(0);

		_pidParameters.kp = -250.0 / 4096.0;
		_pidParameters.ti = 0.2;
		_pidParameters.td = 0.02;

		return true;
	}

	bool
	BalanceControlNode::onLoop()
	{
		// Core::MW::Time t = Core::MW::Time::now();

		if (Core::MW::Thread::should_terminate()) {
			teardown();
		} else {
			if (!this->spin(Configuration::SUBSCRIBER_SPIN_TIME)) {
				Module::led.toggle();
			} else {
				actuator_msgs::Setpoint_f32* msgp;
				float pwm = 0.0;

				if ((_tilt < -20.0) || (_tilt > 20.0)) {
					pwm = 0;
				} else {
					pwm = _pid.update((_tilt)); // tilty offset
				}

				if (_left_pwm_publisher.alloc(msgp)) {
					msgp->value = pwm;
					_left_pwm_publisher.publish(msgp);
				}

				if (_right_pwm_publisher.alloc(msgp)) {
					msgp->value = -pwm;
					_right_pwm_publisher.publish(msgp);
				}

				Core::MW::Thread::sleep((uint16_t)configuration.period);
			}
		}

		return true;
	} // BalanceControlNode::onLoop

	bool
	BalanceControlNode::attitude_callback(
			const sensor_msgs::RPY_f32& msg,
			Core::MW::Node*             node
	)
	{
		BalanceControlNode* _this = static_cast<BalanceControlNode*>(node);

		_this->_tilt = msg.pitch - _this->configuration.offset;

		return true;
	}

	bool
	BalanceControlNode::setpoint_callback(
			const common_msgs::Float32& msg,
			Node*                       node
	)
	{
		BalanceControlNode* _this = static_cast<BalanceControlNode*>(node);

		_this->_setpoint = msg.value;
		_this->_pid.set(_this->_setpoint);

		return true;
	}

	bool
	BalanceControlNode::parameters_callback(
			const pid_msgs::PIDParameters& msg,
			Core::MW::Node*             node
	) {
		BalanceControlNode* _this = static_cast<BalanceControlNode*>(node);

		_this->_pidParameters.kp = msg.kp;
		_this->_pidParameters.ti = msg.ti;
		_this->_pidParameters.td = msg.td;

		_this->_pid.config(_this->_pidParameters.kp, _this->_pidParameters.ti, _this->_pidParameters.td, 0.02, -1.0, 1.0);

		return true;
	}
}
