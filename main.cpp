

#include <fstream>
#include <iostream>
#include <string>
#include <vector>


/*
#include <ros/ros.h>
#include <uxa_sam_msgs/std_position_move.h>
#include <uxa_uic_msgs/motion.h>
#include <uxa_uic_msgs/remocon.h>

void send_msg(std::string STR);
void send_removon(unsigned char remocon);
void send_std_position(unsigned char ID, unsigned int POS);

ros::Publisher motion_pub;
ros::Publisher remocon_pub;
ros::Publisher std_pos_mov_pub;

uxa_uic_msgs::motion uic_motion_msg;
uxa_uic_msgs::remocon uic_remocon_msg;
uxa_sam_msgs::std_position_move sam_std_pos_move_msg;
*/


using namespace std;

//opening the file which contains the commands
//reading the commands from the file line by line
	//process the commands into a format the uic knows
	//store each command into a FIFO queue
//process the FIFO queue

string processCommand(string command);
void sendMessages(vector<string> queue);

int main()
{
	vector<string> queue;
	ifstream command;
	
	
	while (true)
	{
		cout << "Read again?" << endl;
		char x;
		cin >> x;
		command.open("commands.txt");

		if (!command.is_open())
		{
			return -1;
		}
		while (command)
		{
			string c;
			getline(command, c);
			queue.push_back(c);
		}
		command.close();

		sendMessages(queue);
	}
	
	return 0;	
	
}

string processCommand(string command)
{
	return "placeholder";
}

void sendMessages(vector<string> queue)
{
	while (!queue.empty())
	{
		//send_msg(queue[0])
		cout << "sending message: " << queue[0] << endl;
		queue.erase(queue.begin());
	}
}

/*
void send_msg(std::string STR)
{
	uic_motion_msg.motion_name = STR;
	motion_pub.publish(uic_motion_msg);
}

void send_remocon(unsigned char remocon)
{
	uic_remocon_msg.btn_code = remocon;
	remocon_pub.publish(uic_remocon_msg);
}

void send_std_position(unsigned char ID, unsigned int POS)
{
	sam_std_pos_move_msg.id = ID;
	sam_std_pos_move_msg.pos14 = POS;
	std_pos_move_pub.publish(sam_std_pos_move_msg);
}
*/