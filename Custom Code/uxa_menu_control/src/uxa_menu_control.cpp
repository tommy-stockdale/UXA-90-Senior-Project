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

void send_msg(std::string STR);
void send_remocon(unsigned char remocon);
void send_std_position(unsigned char ID, unsigned int POS);
void send_position(unsigned char ID, unsigned char TORQLEVEL, unsigned char POS);

ros::Publisher motion_pub;
ros::Publisher remocon_pub;
ros::Publisher std_pos_move_pub;
ros::Publisher pos_move_pub;

uxa_uic_msgs::motion uic_motion_msg;
uxa_uic_msgs::remocon uic_remocon_msg;
uxa_sam_msgs::std_position_move sam_std_pos_move_msg;
uxa_sam_msgs::position_move sam_pos_move_msg;

//Motion Control
#pragma region motion_control

//Motion control array
const char* menu[] = {
    "turn_right",
    "turn_left",
    "walk_right",
    "walk_left",
    "walk_forward_short",
    "walk_foward_4step",
    "basic_motion",
    "demo_introduction",
    "dance_gangnamstyle",
    "stop",
    "sit_down",
    "kick_right" };

//get the size of the menu
unsigned int menu_size = sizeof(menu) / sizeof(menu[0]);

void print_motion_menu();
void do_motion(unsigned int option);

void motion_control()
{
    //sentinal value
    unsigned char sentinal = 'y';
    do
    {
        //print motion menu
        print_motion_menu();
        //get user input
        unsigned int x;
        std::cin >> x;
        //execute the motion
        do_motion(x);
        //see if loop continues
        std::cout << "Continue?(y/n): ";
        std::cin >> sentinal;

    } while (sentinal == 'y');
    return;
}

void print_motion_menu()
{
    std::cout << "This is a menu of motions the robot can execute." << std::endl;
    for (unsigned int i = 0; i < menu_size; i++)
    {
        std::cout << i << ". " << menu[i] << std::endl;
    }
}
void do_motion(unsigned int option)
{
    if (option < 0 || option >(menu_size) - 1)
    {
        std::cout << "Choice not valid." << std::endl;
        return;
    }
    send_msg(menu[option]);
}

#pragma endregion motion_control

#pragma region sam_control

//this is not the correct angle range for every motor
const int ANGLE_MAX = 255;
const int ANGLE_MIN = 1;
const int NUM_MOTORS = 24;

int request_sam_id();
void keyboard_controller(int id, unsigned int step);
char get_ch();
int defaultAngleLookUp(int id);
void posMemInit();

int positionMem[NUM_MOTORS];

void sam_control()
{
    char sentinal;
    do
    {
        int id;
        unsigned int dpos;
        //get the SAM ID
        id = request_sam_id();
        std::cout << "Enter how much to change the angle(int): ";
        std::cin >> dpos;
        //once the ID is gotten, enable keyboard control of the motor
        keyboard_controller(id, dpos);

        std::cout << "Continue? (y/n): ";
        std::cin >> sentinal;
    } while (sentinal == 'y');
}

int request_sam_id()
{
    int id;
    do
    {
        std::cout << "Enter the SAM motor ID (12-19 or 22-24): ";
        std::cin >> id;
    } while (id < 12 || id > 24 || id == 20 || id == 21);
    return id;
}

void keyboard_controller(int id, unsigned int step)
{

    int position = positionMem[id];
    unsigned char t = 2;
    std::cout << "Controls:\n'w' = increase angle\n's' = decrease angle\n'q' = exit control\n";
    while (true)
    {
        //get the character
        char command = tolower(get_ch());

        switch (command)
        {
        case 'w': (position < ANGLE_MAX) ? position += step : position = ANGLE_MAX; break;
        case 's': (position > ANGLE_MIN) ? position -= step : position = ANGLE_MIN; break;
        case 'q': return;
        default: break;
        }
        std::cout << "ID: " << id << std::endl;
        std::cout << "POS: " << position << std::endl;

        positionMem[id] = position;
        send_position((unsigned char)id, t, (unsigned char)position);
        sleep(1);
    };
}

char get_ch()
{
    char buf = 0;
    struct termios old = { 0 };
    fflush(stdout);
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    printf("%c\n", buf);
    return buf;
}

int defaultAngleLookUp(int id)
{
    int d;
    switch (id)
    {
    case 4:
    case 19:
        d = 48; break;
    case 5:
    case 18:
        d = 206; break;
    case 12: d = 80; break;
    case 13: d = 174;  break;
    case 14: d = 137; break;
    case 15: d = 117; break;
    default: d = 127;
    }

    return d;
}

void memInit()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        positionMem[i] = defaultAngleLookUp(i);
    }
}

#pragma endregion sam_control

#pragma region main_region

const char* main_menu[] = {
    "Motion Control",
    "SAM Motor Control"
};
unsigned int main_menu_size = sizeof(main_menu) / sizeof(menu[0]);

void print_main_menu();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "menu_control");

    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    motion_pub = nh.advertise<uxa_uic_msgs::motion>
        ("uic_driver_motion", 100);
    remocon_pub = nh.advertise<uxa_uic_msgs::remocon>
        ("uic_driver_remocon", 100);

    std_pos_move_pub = nh.advertise<uxa_sam_msgs::std_position_move>
        ("sam_driver_std_position_move", 100);
    pos_move_pub = nh.advertise<uxa_sam_msgs::position_move>("sam_driver_position_move", 100);


    sleep(2);
    send_msg("pc_control");
    sleep(1);
    // send_msg("stop");
    // sleep(7);
    //send_msg("basic_motion");

    while (ros::ok())
    {
        //print out the main menu
        print_main_menu();
        //get user input for which control they want to use
        int x;
        std::cin >> x;
        switch (x)
        {
        case 0: motion_control(); break;
        case 1: sam_control(); break;
        default: break;
        }
    }
}

void print_main_menu()
{
    std::cout << "Main Menu" << std::endl;
    for (unsigned int i = 0; i < main_menu_size; i++)
        std::cout << i << ". " << main_menu[i] << std::endl;
}

#pragma endregion main_region

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

#pragma endregion uxa_functions
