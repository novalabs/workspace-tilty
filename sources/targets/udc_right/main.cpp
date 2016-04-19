#include <Configuration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <common_msgs/Led.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>
//@@MESSAGE_HEADERS@@//

// --- NODES ------------------------------------------------------------------
#include <sensors/Publisher.hpp>
#include <led/Subscriber.hpp>
#include <pid/PIDNode.hpp>
#include <actuators/Subscriber.hpp>
//@@NODE_HEADERS@@//

// --- BOARD IMPL -------------------------------------------------------------
#include <sensors/QEI.hpp>
#include <actuators/MC33926.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using QEI_Publisher  = sensors::Publisher<Configuration::QEI_DELTA_DATATYPE>;
using PWM_Subscriber = actuators::Subscriber<float, actuator_msgs::Setpoint_f32>;

// --- NODES ------------------------------------------------------------------
QEI_Publisher   encoder("encoder", module.qei, Core::MW::Thread::PriorityEnum::NORMAL);
led::Subscriber led_subscriber("led_subscriber", Core::MW::Thread::PriorityEnum::LOWEST);
PWM_Subscriber  motor("actuator_sub", module.pwm, Core::MW::Thread::PriorityEnum::NORMAL);
//@@NODES@@//

// --- MAIN -------------------------------------------------------------------
extern "C" {
	int
	main()
	{
		module.initialize();

		// Module configuration
		module.qei.configuration["period"] = 50;
		module.qei.configuration["ticks"]  = 30.0f * 48.0f;

		// Nodes configuration
		led_subscriber.configuration["topic"] = "led";
		encoder.configuration["topic"]        = "encoder_right";
		motor.configuration["topic"]          = "pwm_right";

		// Add nodes to the node manager (== board)...
		module.add(led_subscriber);
		module.add(encoder);
		module.add(motor);

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
