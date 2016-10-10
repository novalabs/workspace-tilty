#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/sensor_publisher/Publisher.hpp>
#include <core/actuator_subscriber/Subscriber.hpp>
#include <core/actuator_subscriber/Speed.hpp>
#include <core/led/Subscriber.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/QEI_driver/QEI.hpp>
#include <core/MC33926_driver/MC33926.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using QEI_Publisher  = core::sensor_publisher::Publisher<ModuleConfiguration::QEI_DELTA_DATATYPE>;
using PWM_Subscriber = core::actuator_subscriber::Subscriber<float, core::actuator_msgs::Setpoint_f32>;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
QEI_Publisher         encoder_node("encoder", module.qei, core::os::Thread::PriorityEnum::NORMAL);
PWM_Subscriber        pwm_node("actuator_sub", module.pwm, core::os::Thread::PriorityEnum::NORMAL);

// --- CONFIGURATIONS ---------------------------------------------------------
core::QEI_driver::QEI_DeltaConfiguration qei_configuration;
core::led::SubscriberConfiguration       led_subscriber_configuration;
core::sensor_publisher::Configuration    encoder_configuration;
core::actuator_subscriber::Configuration pwm_subscriber_configuration;

// --- MAIN -------------------------------------------------------------------
extern "C" {
   int
   main()
   {
      module.initialize();

      // Module configuration
      qei_configuration.period = 50;
      qei_configuration.ticks  = 30.0f * 48.0f;
      module.qei.setConfiguration(qei_configuration);

      // Led subscriber node
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);
      module.add(led_subscriber);

      // Encoder node
      encoder_configuration.topic = "encoder_left";
      encoder_node.setConfiguration(encoder_configuration);
      module.add(encoder_node);

      pwm_subscriber_configuration.topic = "pwm_left";
      module.add(pwm_node);

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
   } // main
}
