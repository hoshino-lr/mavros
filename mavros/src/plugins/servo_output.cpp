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
        servo_control_sub = nh.subscribe("servo_control", 10, &ServoOutputPlugin::actuator_control_cb, this);
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
    ros::Subscriber servo_control_sub;
	ros::Publisher servo_output_pub;

    /* -*- callbacks -*- */

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

    void actuator_control_cb(const mavros_msgs::ServoOutput::ConstPtr &req) {
        //! about groups, mixing and channels: @p https://pixhawk.org/dev/mixing
        //! message definiton here: @p https://mavlink.io/en/messages/common.html#SET_ACTUATOR_CONTROL_TARGET
        mavlink::common::msg::COMMAND_LONG cmd {};
        cmd.command = 187;
        cmd.confirmation = 0;
        double param1 = (req->controls[0] - 1500) / 500.0;
        double param2 = (req->controls[1] - 1500) / 500.0;
        double param3 = (req->controls[2] - 1500) / 500.0;
        double param4 = (req->controls[3] - 1500) / 500.0;

        param1 = fmax(fmin(1.0,param1),-1.0);
        param2 = fmax(fmin(1.0,param2),-1.0);
        param3 = fmax(fmin(1.0,param3),-1.0);
        param4 = fmax(fmin(1.0,param4),-1.0);

        cmd.param1 = (float)param1;
        cmd.param2 = (float)param2;
        cmd.param3 = (float)param3;
        cmd.param4 = (float)param4;
        cmd.param5 = 0;
        cmd.param6 = 0;
        cmd.param7 = 0;

        cmd.target_system = m_uas->get_tgt_system();
        cmd.target_component = m_uas->get_tgt_component();
        UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
    }

};  // class ServoOutputPlugin

}	// namespace std_plugins

}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ServoOutputPlugin, mavros::plugin::PluginBase)
