/**
 * @brief Servo_output plugin
 * @file Servo_output.cpp
 * @author Li long
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/ServoOutput.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief ActuatorControl plugin
 *
 * Sends actuator controls to FCU controller.
 */
class ServoOutputPlugin : public plugin::PluginBase {
public:
    ServoOutputPlugin() : PluginBase(),
		nh("~")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		servo_output_pub = nh.advertise<mavros_msgs::ServoOutput>("servo_output_raw", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&ServoOutputPlugin::handle_actuator_control_target),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher servo_output_pub;

	/* -*- rx handlers -*- */

	void handle_actuator_control_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVO_OUTPUT_RAW &servo_output_raw)
	{
		auto servo_output_raw_msg = boost::make_shared<mavros_msgs::ServoOutput>();
        servo_output_raw_msg->header.stamp = m_uas->synchronise_stamp(servo_output_raw.time_usec);

        servo_output_raw_msg->controls[0] = servo_output_raw.servo1_raw;
        servo_output_raw_msg->controls[1] = servo_output_raw.servo2_raw;
        servo_output_raw_msg->controls[2] = servo_output_raw.servo3_raw;
        servo_output_raw_msg->controls[3] = servo_output_raw.servo4_raw;

        servo_output_pub.publish(servo_output_raw_msg);
	}

};  // class ServoOutputPlugin

}	// namespace std_plugins

}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ServoOutputPlugin, mavros::plugin::PluginBase)
