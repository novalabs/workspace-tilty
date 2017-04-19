#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/common_msgs/String64.hpp>
#include <core/common_msgs/Vector3_i16.hpp>
#include <core/sensor_msgs/RPY_f32.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Subscriber.hpp>
#include <core/led/Publisher.hpp>

// --- BOARD IMPL -------------------------------------------------------------

// --- MISC -------------------------------------------------------------------
#include "chprintf.h"

// *** DO NOT MOVE ***
Module module;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

static const ShellCommand commands[] = {
		{NULL, NULL}
};

/*
 * IMU subscriber node.
 */
void
imu_sub_node(
   void* arg
)
{
   core::mw::Node node("imu_sub");

   core::mw::Subscriber<core::sensor_msgs::RPY_f32, 2> imu_sub;
   core::mw::Subscriber<core::common_msgs::Vector3_i16, 2> acc_sub;
   core::mw::Subscriber<core::common_msgs::Vector3_i16, 2> gyro_sub;

   core::sensor_msgs::RPY_f32* imu_msgp;
   core::common_msgs::Vector3_i16* acc_msgp;
   core::common_msgs::Vector3_i16* gyro_msgp;


   (void)arg;
   chRegSetThreadName("test_sub");

   node.subscribe(imu_sub, "imu");
   node.subscribe(acc_sub, "acc");
   node.subscribe(gyro_sub, "gyro");

   for (;;) {
      node.spin(core::os::Time::ms(1000));

      if (imu_sub.fetch(imu_msgp)) {
         module.stream.printf("IMU %f %f %f\r\n", imu_msgp->roll, imu_msgp->pitch, imu_msgp->yaw);
         imu_sub.release(*imu_msgp);
      }

      if (acc_sub.fetch(acc_msgp)) {
         module.stream.printf("ACC %d %d %d\r\n", acc_msgp->x, acc_msgp->y, acc_msgp->z);
         acc_sub.release(*acc_msgp);
      }

      if (gyro_sub.fetch(gyro_msgp)) {
         module.stream.printf("GYRO %d %d %d\r\n", gyro_msgp->x, gyro_msgp->y, gyro_msgp->z);
         gyro_sub.release(*gyro_msgp);
      }

//		core::mw::Thread::sleep(core::mw::Time::ms(100));
   }
} // test_sub_node

void
imu_sub_node_status(
   void* arg
)
{
   core::mw::Node node("imu_sub");

   core::mw::Subscriber<core::sensor_msgs::RPY_f32, 2>        imu_sub;
   core::mw::Subscriber<core::sensor_msgs::Delta_f32, 2>      left_encoder_sub;
   core::mw::Subscriber<core::sensor_msgs::Delta_f32, 2>      right_encoder_sub;
   core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 2> left_pwm_sub;
   core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 2> right_pwm_sub;

   core::sensor_msgs::RPY_f32*        imu_msgp;
   core::sensor_msgs::Delta_f32*      left_encoder_msgp;
   core::sensor_msgs::Delta_f32*      right_encoder_msgp;
   core::actuator_msgs::Setpoint_f32* left_pwm_msgp;
   core::actuator_msgs::Setpoint_f32* right_pwm_msgp;

   (void)arg;
   chRegSetThreadName("status");

   node.subscribe(imu_sub, "imu");
   node.subscribe(left_encoder_sub, "encoder_left");
   node.subscribe(right_encoder_sub, "encoder_right");
   node.subscribe(left_pwm_sub, "pwm_left");
   node.subscribe(right_pwm_sub, "pwm_right");

   for (;;) {
      node.spin(core::os::Time::ms(1000));

      if (imu_sub.fetch(imu_msgp)) {
         module.stream.printf("IMU %f %f %f\r\n", imu_msgp->roll, imu_msgp->pitch, imu_msgp->yaw);
         imu_sub.release(*imu_msgp);
      }

      if (left_encoder_sub.fetch(left_encoder_msgp)) {
         module.stream.printf("LE %f\r\n", left_encoder_msgp->value);
         left_encoder_sub.release(*left_encoder_msgp);
      }

      if (right_encoder_sub.fetch(right_encoder_msgp)) {
         module.stream.printf("RE %f\r\n", right_encoder_msgp->value);
         right_encoder_sub.release(*right_encoder_msgp);
      }

      if (left_pwm_sub.fetch(left_pwm_msgp)) {
         module.stream.printf("LP %f\r\n", left_pwm_msgp->value);
         left_pwm_sub.release(*left_pwm_msgp);
      }

      if (right_pwm_sub.fetch(right_pwm_msgp)) {
         module.stream.printf("RP %f\r\n", right_pwm_msgp->value);
         right_pwm_sub.release(*right_pwm_msgp);
      }

      //		core::mw::Thread::sleep(core::mw::Time::ms(100));
   }
} // test_sub_node

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::SubscriberConfiguration led_subscriber_configuration;

// --- MAIN -------------------------------------------------------------------
extern "C" {
   int
   main(
      void
   )
   {
      module.initialize();

      // Led subscriber node
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);
      module.add(led_subscriber);

      // Custom nodes
      core::os::Thread::create_heap(NULL, 1024, core::os::Thread::PriorityEnum::NORMAL, imu_sub_node_status, nullptr);

      // ... and let's play!
      module.setup();
      module.run();

      // Is everything going well?
      for (;;) {
         if (!module.isOk()) {
            module.halt("This must not happen!");
         }

         module.shell(commands); // (Re)Spawn shell as needed...

         core::os::Thread::sleep(core::os::Time::ms(500));
      }

      return core::os::Thread::OK;
   } // main
}
