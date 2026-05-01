#include <iostream>
#include <iomanip>
#include <cmath>
#include "E101.h"
using namespace std;

// --- DEBUG MODE ACTIVATION---
const bool DEBUG_MODE = true;  // Set to false to disable debug output


// --- Motor Assignments ---
const int MOTOR_LEFT   = 1; // all motors need testing
const int MOTOR_RIGHT  = 3;
const int MOTOR_CAMERA = 5;

// --- Speed Constants ---
const int SPEED_STOP    = 48;
const int SPEED_FORWARD_L = 65;
const int SPEED_REVERSE_L = 30;
const int SPEED_FORWARD_R = 30;
const int SPEED_REVERSE_R = 65;
const int ALIGNMENT_THRESHOLD = 10; // Used in Q3, if error is within 10 pixels = going stright

// --- PID Constants ---
const int Kp = 15; // proportional control, makes the robot react based on how far off the line is right now
const int Kd = 5; // makes the robot respond to how quickly the error is changing
const float SHARP_TURN_RATIO = 0.4;  // if more than 40% of black pixels in the row skew either side, it is a sharp turn

// Clamp dv to stay within motor capability range
const int MAX_DV = SPEED_FORWARD_L - SPEED_STOP; // = 17
const int MIN_DV = SPEED_REVERSE_L - SPEED_STOP; // = -18

// --- Adaptive Gain Scaling Factors ---
const float KP_FACTOR = 10.0f;
const float KD_FACTOR = 3.0f;


// --- Scanline Rows ---
const int ROW_NEAR = 120; // Near row to scan
const int ROW_FAR  = 60; // far row to scan

const int IMG_WIDTH  = 320;
const int IMG_CENTER = IMG_WIDTH / 2;

int last_error = 0; 

// --- Global tracking for smooth motion ---
float last_left_speed = SPEED_STOP;
float last_right_speed = SPEED_STOP;
bool is_spinning = false;  // Used to bypass speed limiting during spins



// =========== Utility Functions========================
void print_debug(float blended_error, int diff, int adaptive_Kp, int adaptive_Kd, int dv,float imbalance_ratio, float speed_factor,bool is_spinning, float left_speed, float right_speed, int black_pixels, int red_pixels, int green_pixels, int blue_pixels)
{
    static int frame_counter = 0;
    frame_counter++;
    const int DEBUG_INTERVAL = 5;

    if (!DEBUG_MODE || frame_counter % DEBUG_INTERVAL != 0) return;

    cout << fixed;
    cout.precision(2);
    cout << "[DEBUG] "
         << "Err=" << setw(6) << blended_error
         << " dErr=" << setw(6) << diff
         << " Kp=" << setw(2) << adaptive_Kp
         << " Kd=" << setw(2) << adaptive_Kd
         << " dv=" << setw(4) << dv
         << " Imb=" << setw(5) << imbalance_ratio
         << " SF=" << setw(5) << speed_factor
         << " Spin=" << (is_spinning ? "YES" : " NO")
         << " L=" << setw(3) << int(round(left_speed))
         << " R=" << setw(3) << int(round(right_speed))
         << " BLK=" << setw(4) << black_pixels
		 << " R=" << setw(4) << red_pixels
		 << " G=" << setw(4) << green_pixels
		 << " B=" << setw(4) << blue_pixels


         << endl;
}

// === ======= Utility Clamp Functions ========================
int clamp(int value, int min_val, int max_val) {					//int
    return max(min_val, min(value, max_val));
}
float clampf(float value, float min_val, float max_val) {			//float 
    return max(min_val, min(value, max_val));
}


// --- Utility Red Detection ---
bool is_red(int r, int g, int b) {								// for Q transitions
    return r > 1.3 * g && r > 1.3 * b && r > 50;
}



/* --- Utility PID Gain Calculator ---

         ====== Usage ========
 *-  Kp & Kd are base gain constants (tune them as needed).
 *-  out_Kp & out_Kd are dynamically adjusted based on imbalance_ratio.
 *-  KP_FACTOR & KP_FACTOR are scale factors of imbalence_ratio.(tune them as needed)
 *-  clamp() ensures gains stay within a safe range.
 * */
void compute_adaptive_gains(float imbalance_ratio, int& out_Kp, int& out_Kd) {
    out_Kp = clamp(Kp + imbalance_ratio * KP_FACTOR, 15, 18);  // instead of kp/kd alone, we scale the imbalanc_ratio.
    out_Kd = clamp(Kd + imbalance_ratio * KD_FACTOR, 5, 10);
}



// --- Line Analysis Struct ---
struct LineAnalysis {
    int error;
    int black_pixel_count;
    int left_black;
    int right_black;
    bool is_cluster;
    bool red_flag_detected;
    int red_pixel_count;
    int green_pixel_count;
    int blue_pixel_count;
};

// --- Utility Line Analysis function ---
LineAnalysis get_line_analysis(int row) {
    
      // Accumulators
    int weighted_sum = 0;
    int total_weight = 0;
    int num_black = 0;
    int left_black = 0;
    int right_black = 0;
    int min_brightness = 255, max_brightness = 0;
    int red_pixel_count = 0;
    int green_pixel_count = 0;
	int blue_pixel_count = 0;

	// --- First pass: Get brightness extremes and detect red pixels
    for (int col = 0; col < IMG_WIDTH; col++) {
        int pixel = get_pixel(row, col, 3);// Brightness from greyscale channel
        min_brightness = min(min_brightness, pixel);
        max_brightness = max(max_brightness, pixel);

		// Check for red flag detection (for Q transitions)
        int red = get_pixel(row, col, 0);
        int green = get_pixel(row, col, 1);
        int blue = get_pixel(row, col, 2);
        if (is_red(red, green, blue)) red_pixel_count++;// detects if we've reached quadrant transition
        if (green > red && green > blue && green > 50) green_pixel_count++;
		if (blue > red && blue > green && blue > 50)   blue_pixel_count++;
    }
	
	// Compute dynamic brightness threshold
    int threshold = (min_brightness + max_brightness) / 2;

	// --- Second pass: Compute weighted centroid of black pixels
    for (int col = 0; col < IMG_WIDTH; col++) {
        int pixel = get_pixel(row, col, 3);
        if (pixel < threshold) {
            int offset = col - IMG_CENTER;		// Horizontal distance from center
            int weight = threshold - pixel;		// Darker pixels = higher weight
           
            weighted_sum += offset * weight;
            total_weight += weight;
           
            num_black++;						// Used for is_cluster logic
            if (col < IMG_CENTER) left_black++;
            else right_black++;
        }
    }
	
	// Compute final weighted centroid-based error
    int error = (total_weight > 0) ? weighted_sum / total_weight : 0; 
	
	// Pack results into the LineAnalysis struct
    LineAnalysis result;
    result.error = error;
    result.black_pixel_count = num_black;
    result.left_black = left_black;
    result.right_black = right_black;
    result.is_cluster = (num_black > 190);				// Intersection detection
    result.red_flag_detected = (red_pixel_count > 50);	// Flag for Q transitions
	result.green_pixel_count = green_pixel_count;
	result.blue_pixel_count = blue_pixel_count;
    return result;
}

// --- Utility Drive motor function ---
void drive_motors(float left_speed, float right_speed) {
    // Round, clamp, then send values to motors
    int left_pwm  = clamp(int(round(left_speed)), SPEED_REVERSE_L, SPEED_FORWARD_L);
    int right_pwm = clamp(int(round(right_speed)), SPEED_FORWARD_R, SPEED_REVERSE_R);
    
    set_motors(MOTOR_LEFT, left_pwm);
    set_motors(MOTOR_RIGHT, right_pwm);
    hardware_exchange();
}


// --- Utility Line Fowlling function ---
void line_following(int row_near, int row_far, bool use_far_prediction) {
    LineAnalysis near = get_line_analysis(row_near);
    LineAnalysis far  = get_line_analysis(row_far);

    // === 1. Predict sharpness of upcoming turn ===
		imbalance_ratio = (far.black_pixel_count > 0) ? abs(far.left_black - far.right_black) / float(far.black_pixel_count) : 0.0f; // Uses far detection,detects imbalence further down


	// === 2. Compute adaptive PID gains based on sharpness ===
		int adaptive_Kp, adaptive_Kd;
		compute_adaptive_gains(imbalance_ratio, adaptive_Kp, adaptive_Kd);

    // === 3. Blend near and far errors dynamically ===
		float far_near_assign = clampf(0.5f + 0.4f * imbalance_ratio, 0.5f, 0.9f);// assigns more weight to close detection if turn is sharp, equal weight if stright.
		float blended_error = far_near_assign * near.error + (1.0f - far_near_assign) * far.error;// error now is the result of far detection and close detection blended.

					
	// === 4. PID derivative and control ===
		int diff = blended_error - last_error; // uses blended error to calculate difference in offset each frame.
		last_error = blended_error;
		int dv = adaptive_Kp * blended_error + adaptive_Kd * diff; // PID formula: how much speed gets added/deducted in each motor
		dv = clamp(dv, MIN_DV, MAX_DV); // Clamp dv to stay within motor capability range

					
					
	// === 5. Sharp turn slowdown logic ===
		float speed_factor = 1.0;
		if (use_far_prediction) {
	    bool sharp_turn = far.is_cluster || (imbalance_ratio > SHARP_TURN_RATIO);
		if (sharp_turn) speed_factor = 1.0f - 0.8f * min(imbalance_ratio, 1.0f); //dynamically applies braking 
    }

	// === 6. Compute raw motor speeds ===
	float left_speed  = SPEED_FORWARD_L + dv;
	float right_speed = SPEED_FORWARD_R + dv;



	// === 7. Scale speeds down based on slowdown factor===
	left_speed  = SPEED_STOP + (left_speed  - SPEED_STOP) * speed_factor;
	right_speed = SPEED_STOP + (right_speed - SPEED_STOP) * speed_factor;



	// === 8. Acceleration limiting (skip if spinning) ===
		if (!is_spinning) {
		float max_change = 5.0; // might be too strict
		left_speed  = clamp(left_speed,  last_left_speed  - max_change, last_left_speed  + max_change);
		right_speed = clamp(right_speed, last_right_speed - max_change, last_right_speed + max_change);
    }

		last_left_speed = left_speed;
		last_right_speed = right_speed;
x
	 // === 9. Clamp to legal motor range ===
	     left_speed  = clamp(int(left_speed), SPEED_REVERSE_L, SPEED_FORWARD_L);
		 right_speed = clamp(int(right_speed), SPEED_FORWARD_R, SPEED_REVERSE_R);
   
   //===== DEBUGGING========
    print_debug(blended_error, diff, adaptive_Kp, adaptive_Kd, dv,
            imbalance_ratio, speed_factor, is_spinning,
            left_speed, right_speed,
            near.num_black, near.red_pixel_count, near.green_pixel_count, near.blue_pixel_count);

   
     // === 10. Drive the motors ===
	 drive_motors(left_speed, right_speed);
	 
	 
	
}



// --- Utility Spinning function ---
void spin_in_place(bool right, int error) {
    is_spinning = true;

    int abs_err = abs(error);
    int spin_speed;

    if (abs_err > 50)       spin_speed = 18;
    else if (abs_err > 30)  spin_speed = 12;
    else                    spin_speed = 8;

    // logic: 
    // - Spin right → left motor forward, right motor reverse
    // - Spin left  → left motor reverse, right motor forward
    int left_pwm  = SPEED_STOP + (right ? spin_speed : -spin_speed);
    int right_pwm = SPEED_STOP + (right ? -spin_speed : spin_speed);

    drive_motors(left_pwm, right_pwm);


    is_spinning = false;
}
