#include "E101.h"
#include <iostream>
#include "AVC_utils.h"

int main(){
	init(0);
	set_motors(MOTOR_LEFT, SPEED_STOP);
	set_motors(MOTOR_RIGHT, SPEED_STOP);
	sleep1(100);
    hardware_exchange(); // Send stop command to motors
    sleep1(300);
    stoph(); // Fully stop the hardware
    
    close_screen_stream();
    std::cout << "\n Motors stopping. Exiting...\n";
    exit(0); // Exit the program




return 0;
}
