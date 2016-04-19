#include <Configuration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <common_msgs/Led.hpp>
#include <common_msgs/String64.hpp>
#include <common_msgs/Vector3_i16.hpp>
#include <sensor_msgs/RPY_f32.hpp>
#include <sensor_msgs/Delta_f32.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>
//@@MESSAGE_HEADERS@@//

// --- NODES ------------------------------------------------------------------
#include <led/Subscriber.hpp>
#include <led/Publisher.hpp>
//@@NODE_HEADERS@@//

// --- BOARD IMPL -------------------------------------------------------------

// --- MISC -------------------------------------------------------------------
#include "chprintf.h"

// *** DO NOT MOVE ***
Module module;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)

static void
cmd_mem(
		BaseSequentialStream* chp,
		int                   argc,
		char*                 argv[]
)
{
	size_t n, size;

	(void)argv;

	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}

	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void
cmd_threads(
		BaseSequentialStream* chp,
		int                   argc,
		char*                 argv[]
)
{
	static const char* states[] = {
		CH_STATE_NAMES
	};
	thread_t*          tp;

	(void)argv;

	if (argc > 0) {
		chprintf(chp, "Usage: threads\r\n");
		return;
	}

	chprintf(chp, "    addr    stack prio refs     state time\r\n");
	tp = chRegFirstThread();

	do {
		chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
				(uint32_t)tp,
				(uint32_t)tp->p_ctx.r13,
				(uint32_t)tp->p_prio,
				(uint32_t)(tp->p_refs - 1),
				states[tp->p_state],
				(uint32_t)tp->p_time);
		tp = chRegNextThread(tp);
	} while (tp != NULL);
} // cmd_threads

static void
cmd_balance_pid_set(
		BaseSequentialStream* chp,
		int                   argc,
		char*                 argv[]
)
{
	size_t n, size;

	(void)argv;

	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}

	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static const ShellCommand commands[]
   = {{"mem", cmd_mem}, {"threads", cmd_threads}, {"bpid", cmd_balance_pid_set},
		{NULL,  NULL   }};


/*
 * Test node.
 */
void
test_sub_node(
		void* arg
)
{
	Core::MW::Node node("test_sub");
	Core::MW::Subscriber<common_msgs::String64, 5> sub;
	common_msgs::String64* msgp;

	(void)arg;
	chRegSetThreadName("test_sub");

	node.subscribe(sub, "test");

	for (;;) {
		node.spin(Core::MW::Time::ms(1000));

		if (sub.fetch(msgp)) {
			module.stream.printf("%s\r\n", msgp->data);
			sub.release(*msgp);
		}
	}
} // test_sub_node

/*
 * IMU subscriber node.
 */
void
imu_sub_node(
		void* arg
)
{
	Core::MW::Node node("imu_sub");

	Core::MW::Subscriber<sensor_msgs::RPY_f32, 2>     imu_sub;
	Core::MW::Subscriber<common_msgs::Vector3_i16, 2> acc_sub;
	Core::MW::Subscriber<common_msgs::Vector3_i16, 2> gyro_sub;

	sensor_msgs::RPY_f32*     imu_msgp;
	common_msgs::Vector3_i16* acc_msgp;
	common_msgs::Vector3_i16* gyro_msgp;


	(void)arg;
	chRegSetThreadName("test_sub");

	node.subscribe(imu_sub, "imu");
	node.subscribe(acc_sub, "acc");
	node.subscribe(gyro_sub, "gyro");

	for (;;) {
		node.spin(Core::MW::Time::ms(1000));

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

//		Core::MW::Thread::sleep(Core::MW::Time::ms(100));
	}
} // test_sub_node

void
imu_sub_node_status(
		void* arg
)
{
	Core::MW::Node node("imu_sub");

	Core::MW::Subscriber<sensor_msgs::RPY_f32, 2>     imu_sub;
	Core::MW::Subscriber<sensor_msgs::Delta_f32, 2> left_encoder_sub;
	Core::MW::Subscriber<sensor_msgs::Delta_f32, 2> right_encoder_sub;
	Core::MW::Subscriber<actuator_msgs::Setpoint_f32, 2> left_pwm_sub;
	Core::MW::Subscriber<actuator_msgs::Setpoint_f32, 2> right_pwm_sub;

	sensor_msgs::RPY_f32*     imu_msgp;
	sensor_msgs::Delta_f32*   left_encoder_msgp;
	sensor_msgs::Delta_f32*   right_encoder_msgp;
	actuator_msgs::Setpoint_f32* left_pwm_msgp;
	actuator_msgs::Setpoint_f32* right_pwm_msgp;

	(void)arg;
	chRegSetThreadName("status");

	node.subscribe(imu_sub, "imu");
	node.subscribe(left_encoder_sub, "encoder_left");
	node.subscribe(right_encoder_sub, "encoder_right");
	node.subscribe(left_pwm_sub, "pwm_left");
	node.subscribe(right_pwm_sub, "pwm_right");

	for (;;) {
		node.spin(Core::MW::Time::ms(1000));

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

		//		Core::MW::Thread::sleep(Core::MW::Time::ms(100));
	}
} // test_sub_node

// --- NODES ------------------------------------------------------------------
led::Subscriber led_subscriber("led_subscriber", Core::MW::Thread::PriorityEnum::LOWEST);
led::Publisher  led_publisher("led_publisher");

/*
 * Application entry point.
 */
extern "C" {
	int
	main(
			void
	)
	{
		module.initialize();

		led_publisher.configuration["topic"] = "led";
		led_publisher.configuration["led"]   = (uint32_t)1;

		led_subscriber.configuration["topic"] = led_publisher.configuration["topic"];

		// Add nodes to the node manager (== board)...
		module.add(led_subscriber);
		module.add(led_publisher);
		//@@NODE_MANAGEMENT@@//

		// Custom nodes
		Core::MW::Thread::create_heap(NULL, THD_WORKING_AREA_SIZE(1024), NORMALPRIO, imu_sub_node_status, nullptr);

		// ... and let's play!
		module.setup();
		module.run();

		// Is everything going well?
		for (;;) {
			if (!module.isOk()) {
				module.halt("This must not happen!");
			}

			module.shell(commands); // (Re)Spawn shell as needed...

			Core::MW::Thread::sleep(Core::MW::Time::ms(500));
		}

		return Core::MW::Thread::OK;
	} // main
}
