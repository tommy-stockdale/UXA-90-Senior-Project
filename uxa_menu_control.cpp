#include <iostream>

const char* menu[] = {
	"turn_right",
	"turn_left",
	"walk_right",
	"walk_left"};

unsigned int menu_size = sizeof(menu) / sizeof(menu[0]);

void print_menu()
{
	std::cout << "This is a menu of motions the robot can execute." << std::endl;
	for (unsigned int i = 0; i < menu_size; i++)
	{
		std::cout << i << ". " << menu[i] << std::endl;
	}
}

void send_msg(const char* msg)
{
	std::cout << "Executing command: " << msg << std::endl;
}


void do_motion(unsigned int option)
{
	if (option < 0 || option > (menu_size)-1)
	{
		std::cout << "Choice not valid." << std::endl;
		return;
	}
	send_msg(menu[option]);
}

int main()
{
	std::cout << menu_size << std::endl;
	while (true)
	{
		print_menu();
		int x;
		std::cin >> x;
		do_motion(x);
	}
}