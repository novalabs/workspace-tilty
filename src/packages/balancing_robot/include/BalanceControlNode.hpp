#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/mw/CoreConfiguration.hpp>

#include <core/common_msgs/Float32.hpp>
#include <core/sensor_msgs/RPY_f32.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>
#include <core/pid_msgs/PIDParameters.hpp>
#include <core/balancing_robot/BalanceControlNodeConfiguration.hpp>
#include <core/pid/PID.hpp>

namespace core {
namespace balancing_robot {
class BalanceControlNode:
   public core::mw::CoreNode,
   public core::mw::CoreConfigurable<core::balancing_robot::BalanceControlNodeConfiguration>::CoreConfigurable
{
public:
   BalanceControlNode(
      const char*                    name,
      core::os::Thread::PriorityEnum priority = core::os::Thread::PriorityEnum::NORMAL
   );
   virtual
   ~BalanceControlNode();

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
      void*                       context
   );

   static bool
   setpoint_callback(
      const common_msgs::Float32& msg,
      void*                       context
   );

   static bool
   parameters_callback(
      const pid_msgs::PIDParameters& msg,
      void*                          context
   );


private:
   core::mw::Subscriber<sensor_msgs::RPY_f32, 5>    _attitude_subscriber;
   core::mw::Subscriber<common_msgs::Float32, 5>    _setpoint_subscriber;
   core::mw::Subscriber<pid_msgs::PIDParameters, 5> _pid_parameters_subscriber;

   core::mw::Publisher<actuator_msgs::Setpoint_f32> _left_pwm_publisher;
   core::mw::Publisher<actuator_msgs::Setpoint_f32> _right_pwm_publisher;
};
}
}
