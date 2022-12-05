/**
 * @file /include/uxa_dashboard/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef uxa_dashboard_QNODE_HPP_
#define uxa_dashboard_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <uxa_uic_msgs/remocon.h>
#include <uxa_uic_msgs/motion.h>
#include <uxa_sam_msgs/std_position_move.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace uxa_dashboard {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    void send_msg();
    void send_msg(unsigned char input);
    void send_msg(std::string str);

    void send_std_position(unsigned int pos);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

	ros::Publisher chatter_publisher;
    ros::Publisher dashboard_pub;
    ros::Publisher motion_pub;
    ros::Publisher std_pos_move_pub;

    uxa_uic_msgs::remocon uic_pub_msg;
    uxa_uic_msgs::motion uic_motion_msg;

    uxa_sam_msgs::std_position_move sam_std_pos_move_msg;

    QStringListModel logging_model;
};

}  // namespace uxa_dashboard

#endif /* uxa_dashboard_QNODE_HPP_ */
