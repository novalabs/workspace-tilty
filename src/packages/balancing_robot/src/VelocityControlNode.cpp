#include <Module.hpp>

#include <core/balancing_robot/VelocityControlNode.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>
#include <core/sensor_msgs/RPY_f32.hpp>

namespace core {
namespace balancing_robot {
VelocityControlNode::VelocityControlNode(
   const char*                    name,
   core::os::Thread::PriorityEnum priority
) :
   CoreNode::CoreNode(name, priority),
   CoreConfigurable<core::balancing_robot::VelocityControlNodeConfiguration>::CoreConfigurable(name),
   _left_encoder(0.0f),
   _right_encoder(0.0f)
{
   _workingAreaSize = 512;
}

VelocityControlNode::~VelocityControlNode()
{
   teardown();
}

bool
VelocityControlNode::onPrepareMW()
{
   _setpoint_subscriber.set_callback(VelocityControlNode::setpoint_callback);
   _left_encoder_subscriber.set_callback(VelocityControlNode::left_encoder_callback);
   _right_encoder_subscriber.set_callback(VelocityControlNode::right_encoder_callback);

   this->subscribe(_setpoint_subscriber, configuration().setpoint_topic);
   this->subscribe(_left_encoder_subscriber, configuration().left_encoder_topic);
   this->subscribe(_right_encoder_subscriber, configuration().right_encoder_topic);

   this->advertise(_output_publisher, configuration().output_topic);

   _pid.config(configuration().kp, configuration().ti, configuration().td, configuration().period / 1000.0, -10.0, 10.0);
   _pid.set(0);

   return true;
}   // VelocityControlNode::onPrepareMW

bool
VelocityControlNode::onLoop()
{
   // Core::MW::Time t = Core::MW::Time::now();

   if (!this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {
      Module::led.toggle();
   } else {
      common_msgs::Float32* msgp;
      float v = (_left_encoder + _right_encoder) / 2 * configuration().wheel_radius;
      float tilt_setpoint = _pid.update(v);

      if (_output_publisher.alloc(msgp)) {
         msgp->value = tilt_setpoint;
         _output_publisher.publish(msgp);
      }

      core::os::Thread::sleep(core::os::Time::ms(configuration().period));
   }

   return true;
}   // VelocityControlNode::onLoop

bool
VelocityControlNode::setpoint_callback(
   const differential_drive_msgs::Velocity& msg,
   void*                                    context
)
{
   VelocityControlNode* _this = static_cast<VelocityControlNode*>(context);

   _this->_setpoint.linear  = msg.linear;
   _this->_setpoint.angular = msg.angular;

   return true;
}

bool
VelocityControlNode::left_encoder_callback(
   const sensor_msgs::Delta_f32& msg,
   void*                         context
)
{
   VelocityControlNode* _this = static_cast<VelocityControlNode*>(context);

   _this->_left_encoder = -msg.value * (uint16_t)_this->configuration().period;

   return true;
}

bool
VelocityControlNode::right_encoder_callback(
   const sensor_msgs::Delta_f32& msg,
   void*                         context
)
{
   VelocityControlNode* _this = static_cast<VelocityControlNode*>(context);

   _this->_right_encoder = msg.value * (uint16_t)_this->configuration().period;

   return true;
}
}
}
