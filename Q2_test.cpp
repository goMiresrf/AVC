#include <iostream>
#include <iomanip>
#include <cmath>
#include "E101.h"
#include <csignal>
#include "AVC_utils.h"

using namespace std;


int compute_dv_Q2_this(float error, int black_pixel_count) {
   
   
    error = error / 10.0f;
   
    if (std::abs(error) < 0.6f) return 0;

    if (black_pixel_count < 10) {
        failsafe();
        return 0;
    }

    static float last_error = 0;
    bool sign_flip = (error * last_error < 0);

    // === Gain shaping ===
   
    float shaped = std::pow(std::min(std::abs(error) / 20.0f, 1.0f), 1.2f);
    float KP_this = 0.55f + 0.22f * shaped;    // max ~0.60
    float KD_this = 0.20f + 0.22f * shaped;    // max ~0.42

    float diff = error - last_error;
    last_error = error;

    if (sign_flip) {
        KD_this *= 1.3f;  // Less harsh than 1.5
    }

    float raw_dv = KP_this * error + KD_this * diff;

    // === Center fade (only very near zero) ===
    float center_fade = 1.0f;
    if (std::abs(error) < 8.0f) {
        center_fade = 0.7f + 0.3f * (std::abs(error) / 8.0f); // never below 0.7
    }
    raw_dv *= center_fade;

    // === Soft curve enhancement at extreme error ===
    if (std::abs(error) > 12.0f)
        raw_dv *= 1.1f;

    int dv = static_cast<int>(raw_dv);
    dv = clamp(dv, MIN_DV, MAX_DV);

    // === Limit dv more for small errors ===
    if (std::abs(error) < 2.0f)
        dv = clamp(dv, -3, 3);
    else if (std::abs(error) < 6.0f)
        dv = clamp(dv, -6, 6);

    return dv;
}

void Q2_test(int row) {
    
   

    // === Step 1: Detect line ===
    BlackPixelAnalysis near = detect_black_line(row);
    int error = near.error;
	//float scaled_error = std::abs(error / 10.0f);
    // === Step 2: Base forward speeds ===
    
	//int base_speed = compute_base_speed(scaled_error);
	int base_speed=5;
    
    // === Step 3: PD control ===
    int dv = compute_dv_Q2_this(near.error, near.black_pixel_count);

    
    // === Step 4: Apply dv ===
    int left_speed = SPEED_STOP +  base_speed  + dv;
    int right_speed = SPEED_STOP - base_speed + dv;

    // === Step 5: Clamp speeds to motor range ===
    left_speed = clamp(left_speed, 30, 65);
    right_speed = clamp(right_speed, 30, 65);

    drive_motors(left_speed, right_speed);

    // === Step 6: Debug Output ===
    std::cout << "[Q2] Err: " << std::setw(4) << error
              << " | dv: " << std::setw(4) << dv
              << " | L_PWM: " << std::setw(4) << left_speed
              << " | R_PWM: " << std::setw(4) << right_speed
              << std::endl;
              
    std::cout << "[RESULT] NEAR: Black pixels total: " << near.black_pixel_count << "\n";
    std::cout << "[RESULT] NEAR: Left-side black pixels: " << near.left_black << "\n";
    std::cout << "[RESULT] NEAR: Right-side black pixels: " << near.right_black << "\n";
    std::cout << "[RESULT] NEAR: Density: " << near.black_density << "\n";

}


int main() {
    int err = init(0);
    sleep1(1000);  // Wait 1 second after init
    std::cout<<"Error: "<<err<<std::endl;
    signal(SIGINT, handle_sigint); // Capture Ctrl+C
    open_screen_stream();
	tilt_camera("DOWN");


    while (true) {
        take_picture();
        update_screen();		
      
        Q2_test(ROW_NEAR);	
		
        /*if (red_patch_detected(ROW_FAR)) { 
            std::cout << "[Q2] Red patch Detected. Transitioning to Q3." << std::endl;
            currentQ = Q3;
            q3_state = Q3_STATE1;
            drive_motors(SPEED_STOP, SPEED_STOP, CAMERA_LOOK_DOWN);
            stoph();
            break;
        } 	*/
    }

    return 0;
}

	
