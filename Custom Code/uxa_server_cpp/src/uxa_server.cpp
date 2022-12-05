#include <ros/ros.h>
#include <uxa_sam_msgs/std_position_move.h>
#include <uxa_sam_msgs/position_move.h>
#include <uxa_uic_msgs/motion.h>
#include <uxa_uic_msgs/remocon.h>

#include <stdio.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cctype>

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <string>
#include <cstdlib>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <netinet/in.h>
#include <netdb.h>
#include <bits/stdc++.h>

void send_msg(std::string STR);
void send_remocon(unsigned char remocon);
void send_std_position(unsigned char ID, unsigned int POS);
void send_position(unsigned char ID, unsigned char TORQLEVEL, unsigned char POS);
std::string convertToString(char* a, int size);
ros::Publisher motion_pub;
ros::Publisher remocon_pub;
ros::Publisher std_pos_move_pub;
ros::Publisher pos_move_pub;

uxa_uic_msgs::motion uic_motion_msg;
uxa_uic_msgs::remocon uic_remocon_msg;
uxa_sam_msgs::std_position_move sam_std_pos_move_msg;
uxa_sam_msgs::position_move sam_pos_move_msg;


using namespace std;
int main(int argc, char *argv[8080]) {
    ros::init(argc, argv, "uxa_server");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    motion_pub = nh.advertise<uxa_uic_msgs::motion>("uic_driver_motion", 100);
    remocon_pub = nh.advertise<uxa_uic_msgs::remocon>("uic_driver_remocon", 100);
    std_pos_move_pub = nh.advertise<uxa_sam_msgs::std_position_move>("sam_driver_std_position_move", 100);
    pos_move_pub = nh.advertise<uxa_sam_msgs::position_move>("sam_driver_position_move", 100);

    char buffer[1000];
    int n;
    sockaddr_in serverAddr;
    int serverSock = socket(AF_INET, SOCK_STREAM, 0);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(5000);
    bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr));
    listen(serverSock, 1);
    cout << "Waiting for a client to connect..."<<endl;
    listen(serverSock, 5);
    sockaddr_in clientAddr;
    socklen_t sin_size = sizeof(struct sockaddr_in);
    int clientSock = accept(serverSock, (struct sockaddr*)&clientAddr, &sin_size);
    while (1 == 1)
    {
        bzero(buffer, 1000);

        n = read(clientSock, buffer, 1000);
        cout<<"n: " <<n<<endl;
        if (n==0) break;
        cout<<"Server received: "<<buffer<<endl;
        string command = convertToString(buffer, n);
        cout<<"Command: "<<command<<endl;
        send_msg(command);
        strcpy(buffer, "test");
        n = write(clientSock, buffer, strlen(buffer));

    }
    close(clientSock);
    close(serverSock);
    return 0;
}

#pragma region uxa_functions
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
void send_position(unsigned char ID, unsigned char TORQLEVEL, unsigned char POS)
{
    sam_pos_move_msg.id = ID;
    sam_pos_move_msg.torqlevel = TORQLEVEL;
    sam_pos_move_msg.pos = POS;
    pos_move_pub.publish(sam_pos_move_msg);
}
string convertToString(char* a, int size)
{
    int i;
    string s = "";
    for (i = 0; i < size; i++){
        s = s + a[i];
    }
    return s;
}
