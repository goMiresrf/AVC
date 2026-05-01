#include "E101.h"
#include <iostream>
#include <csignal>
#include "AVC_utils.h"

//g++ -Wall -c "%f" -o "%e.o"

const int MOTOR_LEFT_LOCAL = 1;
const int MOTOR_RIGHT_LOCAL = 3;
const int SPEED_FORWARD_L_LOCAL = 65;
const int SPEED_REVERSE_L_LOCAL= 30;
const int SPEED_FORWARD_R_LOCAL= 30;
const int SPEED_REVERSE_R_LOCAL = 65;
const int SPEED_STOP_LOCAL = 48;


const int ANALYSIS_ROW = 120;


void move_forward() {
    std::cout << "[TEST] Moving forward...\n";
    set_motors(MOTOR_LEFT_LOCAL, SPEED_FORWARD_L_LOCAL);
    set_motors(MOTOR_RIGHT_LOCAL, SPEED_FORWARD_R_LOCAL);
    hardware_exchange();
    sleep1(2000);
}

void move_reverse() {
    std::cout << "[TEST] Moving reverse...\n";
    set_motors(MOTOR_LEFT_LOCAL, SPEED_REVERSE_L_LOCAL);
    set_motors(MOTOR_RIGHT_LOCAL, SPEED_REVERSE_R_LOCAL);
    hardware_exchange();
    sleep1(2000);
}

void spin_left() {
    std::cout << "[TEST] Spinning left...\n";
    set_motors(MOTOR_LEFT_LOCAL, SPEED_REVERSE_L_LOCAL);
    set_motors(MOTOR_RIGHT_LOCAL, SPEED_FORWARD_R_LOCAL);
    hardware_exchange();
    sleep1(2000);
}

void spin_right() {
    std::cout << "[TEST] Spinning right...\n";
    set_motors(MOTOR_LEFT_LOCAL, SPEED_FORWARD_L_LOCAL);
    set_motors(MOTOR_RIGHT_LOCAL, SPEED_REVERSE_R_LOCAL);
    hardware_exchange();
    sleep1(2000);
}

void stop_motors() {
    set_motors(MOTOR_LEFT_LOCAL, SPEED_STOP_LOCAL);
    set_motors(MOTOR_RIGHT_LOCAL, SPEED_STOP_LOCAL);
    hardware_exchange();
    sleep1(500);
}

void test_camera_motor() {
    std::cout << "[TEST] Starting camera motor test (M5)...\n";
    
    init(0); // Ensure hardware is initialized

    for (int i = 0; i < 3; i++) {
        std::cout << "[TEST] Tilting camera up...\n";
        set_motors(MOTOR_CAMERA, 61);  // Full forward
        hardware_exchange();
        sleep1(800);  // Run for 800 ms

        std::cout << "[TEST] Stopping...\n";
        set_motors(MOTOR_CAMERA, SPEED_STOP);  // Stop
        hardware_exchange();
        sleep1(500);

        std::cout << "[TEST] Tilting camera down...\n";
        set_motors(MOTOR_CAMERA, 31);  // Full reverse
        hardware_exchange();
        sleep1(800);

        std::cout << "[TEST] Stopping...\n";
        set_motors(MOTOR_CAMERA, SPEED_STOP);
        hardware_exchange();
        sleep1(500);
    }

    std::cout << "[TEST] Camera motor test completed.\n";
}


void test_line_analysis() {
	
    std::cout << "[VISION] Capturing image and analyzing line...\n";
    take_picture();

    BlackPixelAnalysis near_line = detect_black_line(ANALYSIS_ROW);
	BlackPixelAnalysis far_line  = detect_black_line(ROW_FAR);
	ColourCentroid  colour_analysis= detect_colour_centroids();
	
	std::cout << "[RESULT] FAR: Error (centroid offset): " << far_line.error << "\n";
    std::cout << "[RESULT] FAR: Black pixels total: " << far_line.black_pixel_count << "\n";
    std::cout << "[RESULT] FAR: Left-side black pixels: " << far_line.left_black << "\n";
    std::cout << "[RESULT] FAR: Right-side black pixels: " << far_line.right_black << "\n";
	std::cout << "[RESULT] FAR: Density: " << far_line.black_density << "\n";
	
    std::cout << "[RESULT] NEAR: Error (centroid offset): " << near_line.error << "\n";
    std::cout << "[RESULT] NEAR: Black pixels total: " << near_line.black_pixel_count << "\n";
    std::cout << "[RESULT] NEAR: Left-side black pixels: " << near_line.left_black << "\n";
    std::cout << "[RESULT] NEAR: Density: " << near_line.black_density << "\n";
   
    
    std::cout << "[COLOURS] R: " << colour_analysis.red_count << std::endl;

}



int main() {
    init(0);
    
    signal(SIGINT, handle_sigint);
    open_screen_stream();
    //display screen
    

    std::cout << "[INFO] Starting full motor and vision test...\n";
	
	
    /*move_forward();
    move_reverse();
    spin_left();
    spin_right();
    stop_motors();
	std::cout << "[INFO] Starting continuous analysis loop...\n";
*/
while (true){
 
	take_picture();
	update_screen();
	test_line_analysis();
    drive_motors(SPEED_STOP,SPEED_STOP);   
}


return 0;
}
