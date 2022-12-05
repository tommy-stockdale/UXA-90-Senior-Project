/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <uxa_uic_msgs/remocon.h>
#include <sstream>
#include "../include/uxa_dashboard/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace uxa_dashboard {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"uxa_dashboard");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    dashboard_pub = n.advertise<uxa_uic_msgs::remocon>("uic_driver_remocon", 100);
    motion_pub = n.advertise<uxa_uic_msgs::motion>("uic_driver_motion", 100);
    std_pos_move_pub = n.advertise<uxa_sam_msgs::std_position_move>("sam_driver_std_position_move", 100);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"uxa_dashboard");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
        /*std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
        log(Info,std::string("I sent: ")+msg.data);*/
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::send_msg()
{
    unsigned char temp;
    temp = 0xff;
    uic_pub_msg.btn_code = temp;
    dashboard_pub.publish(uic_pub_msg);


}

void QNode::send_msg(unsigned char input)
{

    uic_pub_msg.btn_code = input;
    dashboard_pub.publish(uic_pub_msg);


}
void QNode::send_msg(std::string str)
{
    uic_motion_msg.motion_name = str;
    motion_pub.publish(uic_motion_msg);
}

void QNode::send_std_position(unsigned int pos)
{
    sam_std_pos_move_msg.id = 24;
    sam_std_pos_move_msg.pos14 = pos;
    std_pos_move_pub.publish(sam_std_pos_move_msg);
}


}  // namespace uxa_dashboard
