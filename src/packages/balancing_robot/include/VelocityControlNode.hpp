#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>

#include <core/common_msgs/Float32.hpp>
#include <core/differential_drive_msgs/Velocity.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>
#include <core/balancing_robot/VelocityControlNodeConfiguration.hpp>
#include <core/pid/PID.hpp>

namespace core {
namespace balancing_robot {
class VelocityControlNode:
   public core::mw::CoreNode,
   public core::mw::CoreConfigurable<core::balancing_robot::VelocityControlNodeConfiguration>::CoreConfigurable
{
public:
   VelocityControlNode(
      const char*                    name,
      core::os::Thread::PriorityEnum priority = core::os::Thread::PriorityEnum::NORMAL
   );
   virtual
   ~VelocityControlNode();

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
      void*                                    context
   );

   static bool
   left_encoder_callback(
      const sensor_msgs::Delta_f32& msg,
      void*                         context
   );

   static bool
   right_encoder_callback(
      const sensor_msgs::Delta_f32& msg,
      void*                         context
   );


private:
   core::mw::Subscriber<differential_drive_msgs::Velocity, 5> _setpoint_subscriber;
   core::mw::Subscriber<sensor_msgs::Delta_f32, 5> _left_encoder_subscriber;
   core::mw::Subscriber<sensor_msgs::Delta_f32, 5> _right_encoder_subscriber;
   core::mw::Publisher<common_msgs::Float32>       _output_publisher;
};
}
}
