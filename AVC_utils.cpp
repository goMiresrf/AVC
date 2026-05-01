#include <iostream>
#include <cmath>
#include <iomanip>
#include "E101.h"
#include "AVC_utils.h"
#include <csignal>
#include <string>


// ======Global Initilisers===============

Quadrant currentQ = Q1;
bool allow_failsafe = true;
int post_turn_cooldown = 0;
int spin_strength_override = -1;


// ==== DEBUGGING=================

bool DEBUG_MODE = false;



//===============Q1 ===============

bool gate_opened = false;

void handle_Q1() {

if (!gate_opened) {
	char request[] = "Please";
	char server[] = "130.195.3.53";
	char response[24];
	
	std::cout << "[Q1] Connecting to gate server...\n";
	connect_to_server(server, 1024);
	send_to_server(request);
	receive_from_server(response);
	send_to_server(response);
	
	
	gate_opened = true;
	std:: cout << "[Q1] Gate opened. Driving forward to pass through..." << std::endl;
	currentQ = Q2;
	line_following(ROW_NEAR);
	
} 
}




//============== Q2 ==============

void handle_Q2() {
    // Step 1: Check for transition condition
    if (red_patch_detected(ROW_FAR)) {
        std::cout << "[Q2] Red patch detected — transitioning to Q3...\n";
        currentQ = Q3;
        return;
    }

    // Step 2: Continue line following
    line_following(ROW_NEAR);
}





//============== Q3 ==============

bool lineOnTop = false;
bool lineOnLeft = false;
bool lineOnRight = false;
	
void handle_Q3(){

if (red_patch_detected(ROW_FAR) && currentQ == Q3) { 
			std::cout << "[Q3] Red patch Detected. Transitioning to Q4." << std::endl;
			currentQ = Q4;
			drive_motors(SPEED_STOP, SPEED_STOP);
			tilt_camera("UP");
			sleep1(200);
}    
			
		line_following_Q3();
		
}

void spin_until_aligned(int direction) {
    const int SPIN_SPEED_MAX = 8;
    const int ERROR_START_THRESHOLD = 40;
    const int ERROR_END_THRESHOLD = 30;
    const int SETTLE_ERROR_THRESHOLD = 20;
    const int MIN_SPIN_FRAMES = 6;
    const int MAX_SPIN_FRAMES = 40;
    

    bool saw_big_error = false;
	allow_failsafe = false;
    int spin_frames = 0;
	
    while (true) {
        take_picture();
        BlackPixelAnalysis near = detect_black_line(200);

        float error = near.error;
        spin_frames++;

        // Stage 1: mark when a big error is seen
        if (std::abs(error) > ERROR_START_THRESHOLD) {
            saw_big_error = true;
        }

		update_intersection_flags();


        // Stage 2: allow alignment only after big error and enough frames
        if (saw_big_error && std::abs(error) < ERROR_END_THRESHOLD && spin_frames >= MIN_SPIN_FRAMES && lineOnTop) {            
            std::cout << "[SPIN] Alignment complete after valid turn.\n";
            sleep1(100); // tune down if sluggish 
            break;
        }

        if (spin_frames >= MAX_SPIN_FRAMES) {
            std::cout << "[SPIN] Max spin timeout — breaking to avoid infinite loop.\n";
            break;
        }

        // === Exponential spin strength based on error ===
        float norm_error = std::min(std::abs(error) / 80.0f, 1.0f);
        float shaped = std::pow(norm_error, 2.0f);
        const int SPIN_SPEED_MIN = 4;
        const int SPIN_SPEED_SETTLE = 2;
        int spin_strength = (spin_strength_override > 0) ? spin_strength_override : std::max(SPIN_SPEED_MIN, std::min((int)(SPIN_SPEED_MAX * shaped), SPIN_SPEED_MAX));

        //int spin_strength = std::max(SPIN_SPEED_MIN, (int)(SPIN_SPEED_MAX * shaped));
        
        // === Final settling phase: gentle spin when very close to alignment ===
		if (std::abs(error) < SETTLE_ERROR_THRESHOLD && saw_big_error) {
		spin_strength = std::min(spin_strength, SPIN_SPEED_SETTLE + (int)(shaped * 2));
}

        int left_pwm  = SPEED_STOP + direction * spin_strength;
        int right_pwm = SPEED_STOP + direction * spin_strength;

        drive_motors(left_pwm, right_pwm);

        std::cout << "[SPIN] Turning " << (direction > 0 ? "RIGHT" : "LEFT")
                  << " | Error: " << error
                  << " | SpinStrength: " << spin_strength
                  << " | SawBigError: " << saw_big_error << "\n";
    }

    drive_motors(SPEED_STOP, SPEED_STOP);
    allow_failsafe = true;
    post_turn_cooldown = 10;
    sleep1(200);
      

}

void update_intersection_flags() {
    lineOnTop = false;
    lineOnLeft = false;
    lineOnRight = false;

    // --- Top Edge: use row 5 instead of 0 ---
    BlackPixelAnalysis top = detect_black_line(0);
    if (top.black_pixel_count > 50) lineOnTop = true;

    int left_black = 0;
    int right_black = 0;

    for (int row = ROW_Q3-1; row < IMG_HEIGHT; row++) {
        for (int col = 0; col < 5; col++) {
            int grey = get_pixel(row, col, 3);
            if (grey < 100) left_black++;
        }
        for (int col = IMG_WIDTH - 5; col < IMG_WIDTH; col++) {
            int grey = get_pixel(row, col, 3);
            if (grey < 100) right_black++;
        }
    }

    if (left_black > 70) lineOnLeft = true;
    if (right_black > 70) lineOnRight = true;

    std::cout << "[DEBUG] Top: " << top.black_pixel_count
              << " | Left: " << left_black
              << " | Right: " << right_black << "\n";
}

void drive_forward() {
    std::cout << "[ADVANCE] Driving fixed distance into junction...\n";
    allow_failsafe = false;

    for (int i = 0; i < 8; i++) {
        update_intersection_flags();

        drive_motors(50, 46);
        sleep1(100);
    }

    drive_motors(SPEED_STOP, SPEED_STOP);
    sleep1(150);
}

void line_following_Q3() {

    static int turn_cooldown = 0;  // prevents back-to-back false turns
    static bool intersection_turned = false;
    static bool intersection_turned_2 = false;    
    
    
    update_intersection_flags();

    if (turn_cooldown > 0) {
        turn_cooldown--;
        return;
    }

    if (post_turn_cooldown> 0) {
        post_turn_cooldown--;
        std::cout << "[Q3 TEST] Cooldown active — skipping turn detection.\n";
        allow_failsafe = true;
        line_following(ROW_Q3);
        return;
    }


    // === Initial soft detection ===
    bool possible_junction = (lineOnLeft || lineOnRight);
    bool confirmed_junction = false;
    bool saw_full_intersection = false;

    
    if (possible_junction) {
        std::cout << "[Q3 TEST] Possible junction detected — rolling forward to confirm.\n";

        // Drive forward slightly to get full view
		for (int i = 0; i < 5; i++) {
			take_picture();
			update_screen();
			update_intersection_flags();

			if (lineOnLeft && lineOnRight) {
				saw_full_intersection = true;
				break;
			}

}


        drive_motors(SPEED_STOP, SPEED_STOP);
        sleep1(100);

        if (saw_full_intersection) {
            confirmed_junction = true;
            std::cout << "[Q3 TEST] Full intersection confirmed.\n";
        }
    }

    if (intersection_turned_2 && confirmed_junction) {
        std::cout << "[Q3 TEST] Second turn — spinning right.\n";
        drive_forward();
        spin_until_aligned(+1);
        turn_cooldown = 10;
        return;
    }

    if (confirmed_junction && intersection_turned) {
        std::cout << "[Q3 TEST] Cleanup spin — turning left.\n";
        drive_forward();
        spin_strength_override = 8;
        spin_until_aligned(-1);
        spin_strength_override= -1;
        turn_cooldown = 10;
        intersection_turned_2 = true;
        intersection_turned = false;
        return;
    }

    if (confirmed_junction && !intersection_turned) {
        std::cout << "[Q3 TEST] First turn — spinning left.\n";
        drive_forward();
        spin_until_aligned(-1);
        turn_cooldown = 10;
        intersection_turned = true;
        return;
    }

    // === Detect Sharp Turns Outside Junctions ===
    bool sharp_right =  lineOnRight; //near.black_density > 0.4f &&
    bool sharp_left  =  lineOnLeft; //near.black_density > 0.4f &&

    if (sharp_right) {
        std::cout << "[Q3 TEST] Sharp RIGHT confirmed — spinning right.\n";
        drive_forward();
        spin_until_aligned(+1);
        turn_cooldown = 10;
        return;
    }

    if (sharp_left) {
        std::cout << "[Q3 TEST] Sharp LEFT confirmed — spinning left.\n";
        drive_forward();
        spin_until_aligned(-1);
        turn_cooldown = 10;
        return;
    }

    // === No spin — follow line normally ===
    line_following(ROW_Q3);
}





//============== Q4 ==============

void handle_Q4(){
	
    switch (q4_state) {
        case Q4_STATE0:
            // Free-floating entry state – do nothing or wait for trigger
            break;

        case Q4_STATE1:
            follow_colour(GREEN, APPROACH, get_settings_for(GREEN), true);
            q4_state = Q4_STATE2;
            break;

        case Q4_STATE2:
            follow_colour(GREEN, CLEAR, get_settings_for(GREEN), false); // reverse
            spin_until_colour_detected(RED, 2000, -1);  // spin left to find red
            q4_state = Q4_STATE3;
            break;

        case Q4_STATE3:
            follow_colour(RED, APPROACH, get_settings_for(RED), true);
            q4_state = Q4_STATE4;
            break;

        case Q4_STATE4:
            follow_colour(RED, CLEAR, get_settings_for(RED), false); // reverse
            spin_until_colour_detected(BLUE, 1000, -1); // spin left to find blue
            q4_state = Q4_STATE5;
            break;

        case Q4_STATE5:
            follow_colour(BLUE, APPROACH, get_settings_for(BLUE), true);
            q4_state = Q4_STATE6;
            break;

        case Q4_STATE6:
            follow_colour(BLUE, CLEAR, get_settings_for(BLUE), false); // reverse
            spin_until_colour_detected(RED, 2000, -1); // spin left to find final red
            q4_state = Q4_STATE7;
            break;

        case Q4_STATE7: {
			ColourFollowSettings red_final = get_settings_for(RED);
			red_final.approach_threshold = 600000;  // Ensure we get really close

			follow_colour(RED, APPROACH, red_final, true);  // Drive into the final red
			std::cout << "[Q4] Final RED reached — sleeping to knock it off...\n";
			sleep1(500);  // Stay pushed against the tower for a bit (adjust as needed)

			q4_state = Q4_DONE;
			break;
}


        case Q4_DONE:
            break;
    }
}	


Q4State q4_state = Q4_STATE0;

void spin_until_colour_detected(Colour color, int threshold, int direction = +1) {
    ColourFollowSettings settings = get_settings_for(color);
    std::string name = (color == RED) ? "RED" : (color == GREEN) ? "GREEN" : "BLUE";

    std::cout << "[Q4] Spinning to find " << name << "...\n";

    // === Step 1: Priming spin to avoid stale background detection ===
    for (int i = 0; i < 15; i++) {  // ~500ms
        drive_motors(SPEED_STOP + direction * settings.spin_speed,
                     SPEED_STOP + direction * settings.spin_speed);
        sleep1(30);
    }

    // === Step 2: Begin actual detection ===
    while (true) {
        take_picture();
        ColourCentroid cc = detect_colour_centroids();

        int count = 0;
        if (color == RED)   count = cc.red_count;
        if (color == GREEN) count = cc.green_count;
        if (color == BLUE)  count = cc.blue_count;

        std::cout << "[Q4] Searching... " << name << " count: " << count << "\n";

        if (count > threshold) {
            std::cout << "[Q4] Found " << name << "!\n";
            break;
        }

        drive_motors(SPEED_STOP + direction * settings.spin_speed,
                     SPEED_STOP + direction * settings.spin_speed);
        sleep1(30);
    }

}

ColourCentroid detect_colour_centroids() {
    int red_count = 0, red_sum = 0;
    int green_count = 0, green_sum = 0;
    int blue_count = 0, blue_sum = 0;

    for (int row = 0; row < 240; row++) {
        for (int col = 0; col < 320; col++) {
            int r = get_pixel(row, col, 0);
            int g = get_pixel(row, col, 1);
            int b = get_pixel(row, col, 2);

			b = std::min(static_cast<int>(b * 1.1), 255);


            bool is_red = (r > 1.5 * g && r > 1.5 * b && r > 100);
            bool is_green = (g > 1.3 * r &&  g > 1.3 * b   &&  g > 100);
            bool is_blue = (b > 1.2 * r && b > 1.2 * g && b > 100);
           
            if (b > 1.2 * r && b > 1.2 * g && b > 40) {
			r = std::max(r - 100, 0); // Temporarily crush red
}



            if (is_red) {
                red_count++;
                red_sum += col;
                if (DEBUG_MODE) set_pixel(row, col,(char) 0,(char) 0,(char) 255);
            }
            if (is_green) {
                green_count++;
                green_sum += col;
                if (DEBUG_MODE) set_pixel(row, col,(char) 0,(char) 255,(char) 0);
            }
            if (is_blue) {    
                blue_count++;
                blue_sum += col;
                if (DEBUG_MODE) set_pixel(row, col, (char)255,(char) 0, (char) 255);
            }
        }
    }

    ColourCentroid cc = {0};
    cc.red_count = red_count;
    cc.red_centroid = (red_count > 0) ? red_sum / red_count : -1;
    cc.green_count = green_count;
    cc.green_centroid = (green_count > 0) ? green_sum / green_count : -1;
    cc.blue_count = blue_count;
    cc.blue_centroid = (blue_count > 0) ? blue_sum / blue_count : -1;

	
	// === Debugging Output ===
    std::cout << std::fixed << std::setprecision(2)
              << "[COLOUR DETECTION] "
              << "RED: Count=" << cc.red_count << " | Centroid=" << cc.red_centroid << "   | "
              << "GREEN: Count=" << cc.green_count << " | Centroid=" << cc.green_centroid << "   | "
              << "BLUE: Count=" << cc.blue_count << " | Centroid=" << cc.blue_centroid << "\n";
    
    return cc;
}

ColourFollowSettings get_settings_for(Colour c) {
ColourFollowSettings s;

    if (c == GREEN) {
		s.start_approach_threshold = 1000;
        s.approach_threshold = 30000;
        s.clear_threshold = 8000;
        s.reacquire_threshold = 10;
        s.base_speed = 5;
        s.spin_speed = 4;
    } else if (c == RED) {
        s.start_approach_threshold = 1000;
        s.approach_threshold = 50000;
        s.clear_threshold = 8000;
        s.reacquire_threshold = 10;
        s.base_speed = 5;
        s.spin_speed = 4;
    } else if (c == BLUE) {
       s.start_approach_threshold = 1000;
        s.approach_threshold = 60000;
        s.clear_threshold = 8000;
        s.reacquire_threshold = 10;
        s.base_speed = 5;
        s.spin_speed = 4;
    } else {
        // fallback
        s.start_approach_threshold = 1000;
        s.approach_threshold = 20000;
        s.clear_threshold = 8000;
        s.reacquire_threshold = 10;
        s.base_speed = 5;
        s.spin_speed = 4;
    }

    return s;
}

void follow_colour(Colour color, ColourGoalType goal, const ColourFollowSettings& settings, bool forward = true) {
    
    
    
    std::string color_name = (color == RED) ? "RED" : (color == GREEN) ? "GREEN" : "BLUE";

    while (true) {
        take_picture();
        ColourCentroid cc = detect_colour_centroids();

        int centroid = -1;
        int count = 0;

        if (color == RED)   { centroid = cc.red_centroid;   count = cc.red_count; }
        if (color == GREEN) { centroid = cc.green_centroid; count = cc.green_count; }
        if (color == BLUE)  { centroid = cc.blue_centroid;  count = cc.blue_count; }

        std::cout << "[Q4] " << color_name << " | Centroid: " << centroid << " | Count: " << count << "\n";


       
       // --- Goal check ---
        if (goal == APPROACH && count > settings.approach_threshold) {
            std::cout << "[Q4] Close enough to " << color_name << "!\n";
            break;
        }

        if (goal == CLEAR && count < settings.clear_threshold) {
            std::cout << "[Q4] " << color_name << " cleared from view.\n";
            break;
        }

        // --- Reacquire if lost ---
        if (count < 10 || centroid == -1) {
            std::cout << "[Q4] Lost " << color_name << " — spinning to reacquire...\n";

            while (true) {
                take_picture();
                update_screen();
                ColourCentroid reacquire = detect_colour_centroids();

                int found = 0;
                if (color == RED)   found = reacquire.red_count;
                if (color == GREEN) found = reacquire.green_count;
                if (color == BLUE)  found = reacquire.blue_count;

                std::cout << "[Q4] Searching... Found " << color_name << " Pixels: " << found << "\n";

                if (found > settings.reacquire_threshold) {
                    std::cout << "[Q4] " << color_name << " reacquired.\n";
                    break;
                }

                int spin_pwm = SPEED_STOP + settings.spin_speed;
                drive_motors(spin_pwm, spin_pwm);
                sleep1(30);
            }

            drive_motors(SPEED_STOP, SPEED_STOP);
            sleep1(100);
            continue;  // retry frame
        }

        // === Step 1: Calculate error ===
        int error = centroid - IMG_CENTER;

        // === Step 2: Compute dv ===
        int dv = compute_dv(error);

        // === Step 3: Apply dv ===
        int base = forward ? settings.base_speed : -settings.base_speed;
        int left_speed  = SPEED_STOP + base + dv;
        int right_speed = SPEED_STOP - base + dv;

        // === Step 4: Clamp ===
        left_speed  = clamp(left_speed, 30, 65);
        right_speed = clamp(right_speed, 30, 65);

        // === Step 5: Drive and Debug ===
        drive_motors(left_speed, right_speed);
        sleep1(30);

        std::cout << "[Q4] " << color_name
                  << " | Err: " << error
                  << " | dv: " << dv
                  << " | L_PWM: " << left_speed
                  << " | R_PWM: " << right_speed
                  << " | Count: " << count
                  << "\n";
    }

    drive_motors(SPEED_STOP, SPEED_STOP);
    sleep1(100);
}





// =========== GLOBAL Utility Functions========================


int compute_dv(float error) {
    error = error / 10.0f;
    if (std::abs(error) < 0.6f) return 0;


    static float last_error = 0;
    bool sign_flip = (error * last_error < 0);

    // === Gain shaping ===
    float shaped = std::pow(std::min(std::abs(error) / 20.0f, 1.0f), 1.2f);  // softer curve
    float KP = 0.55f + 0.22f * shaped;    // max ~0.77
    float KD = 0.20f + 0.22f * shaped;    // max ~0.42

    float diff = error - last_error;
    last_error = error;

    if (sign_flip) {
        KD *= 1.3f;  // Less harsh than 1.5
    }

    float raw_dv = KP * error + KD * diff;

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


void handle_sigint(int signum) {
    
    // Stop all motors
	set_motors(MOTOR_LEFT, SPEED_STOP);
	set_motors(MOTOR_RIGHT, SPEED_STOP);

    hardware_exchange(); // Send stop command to motors
    sleep1(300);
    stoph(); // Fully stop the hardware
    
    std::cout << "\nInterrupted due to ctr+c Motors stopped. Exiting...\n";
    exit(signum); // Exit the program
}


void failsafe() {
    if(!red_patch_detected(ROW_FAR)){
        set_motors(MOTOR_LEFT, 44);
        set_motors(MOTOR_RIGHT, 52);
        hardware_exchange();
        sleep1(300);  // Reverse briefly
}
}


int clamp(int value, int min_val, int max_val) {					//int
    return std::max(min_val, std::min(value, max_val));
}

float clampf(float value, float min_val, float max_val) {			//float 
    return std::max(min_val, std::min(value, max_val));
}


bool red_patch_detected(int row) {
    int red_count = 0;

    for (int col = 0; col < IMG_WIDTH; col++) {
        int r = get_pixel(row, col, 0);
        int g = get_pixel(row, col, 1);
        int b = get_pixel(row, col, 2);

        bool is_red = (r > (1.3 * g) && r > (1.3 * b) && r > 100);

        if (is_red) {
            red_count++;
            if (DEBUG_MODE) set_pixel(row, col,(char) 0,(char) 0,(char) 255); // blue mark
        }
}
			return red_count >100 ;
}


BlackPixelAnalysis detect_black_line(int row) {
    int weighted_sum = 0;
    int total_weight = 0;
    int black_pixel_count = 0;
    int left_black = 0;
    int right_black = 0;
    int min_brightness = 255;
    int max_brightness = 0;
	
	//--- First pass - Find Brightness Extremes --------
			
    for (int col = 0; col < IMG_WIDTH; col++) {
         
        int grey = get_pixel(row, col, 3);
       
        if (grey > 20 && grey <220){
        min_brightness = std::min(min_brightness, grey);
        max_brightness = std::max(max_brightness, grey);
      
      }    
    }
    int threshold = (min_brightness + max_brightness) / 2; //adjust to changing lighting conditions. Any pixel darker than this (with margin) is considered "possibly black".


// -- Second Pass – Detect Black Pixels and Calculate Centroid------
    for (int col = 0; col < IMG_WIDTH; col++) {
        int r = get_pixel(row, col, 0);
        int g = get_pixel(row, col, 1);
        int b = get_pixel(row, col, 2);
        int grey = get_pixel(row, col, 3);

        int diff = threshold - grey;// how much darker the current pixel is compared to the dynamic brightness threshold.
        //bool gray_dark = (diff > 5); // Is it darker than threshold?
        bool gray_dark= grey < threshold - 10;
        bool rgb_dark = (r < 100 && g < 100 && b < 100); //Is it really dark in all RGB channels?

        if (gray_dark && rgb_dark) {
            int offset = col - IMG_CENTER; //How far is the pixel from the image center (negative = left, positive = right)
            int weight = std::min(diff, 50); //how dark the pixel is (blacker = higher weight)
            weighted_sum += offset * weight;// weighted average offset of black pixels from the image center
           
            total_weight += weight;
            black_pixel_count++;
            if (col < IMG_CENTER) left_black++;
            else right_black++;

            if (DEBUG_MODE) set_pixel(row, col,(char) 0,(char)255,(char)0);
        }
    }

    int error = (total_weight > 0) ? weighted_sum / total_weight : 0; //how far the black line is from the center of the image
    float black_density = black_pixel_count / float(IMG_WIDTH); // how dense the black pixels are
    float imbalance = (black_pixel_count > 0) ? abs(left_black - right_black) / float(black_pixel_count) : 0.0f;

    BlackPixelAnalysis result = {0};
    result.error = error;
    result.black_pixel_count = black_pixel_count;
    result.left_black = left_black;
    result.right_black = right_black;
    result.black_density= black_density;
    result.imbalance= imbalance;
    return result;
}


void tilt_camera(const std::string& direction) {
    int target_pwm;
    if (direction == "DOWN") {
        target_pwm = 30;
    } else if (direction == "UP") {
        target_pwm = 65;
    } else {
        std::cout << "[TILT] Invalid direction. Use \"UP\" or \"DOWN\"." << std::endl;
        return;
    }

    set_motors(MOTOR_CAMERA, target_pwm);
    hardware_exchange();
    sleep1(400);
}


void drive_motors(float left_speed, float right_speed) {
    
    // Roundthen send values to motors
    int left_pwm = clamp(int(round(left_speed)),30,65);
    int right_pwm = clamp(int(round(right_speed)),30,65);

    set_motors(MOTOR_LEFT, left_pwm);
    set_motors(MOTOR_RIGHT, right_pwm);
	
    
    hardware_exchange();
}


void line_following(int row) {
    
    // === Step 1: Detect line ===
    BlackPixelAnalysis near = detect_black_line(row);
    int error = near.error;
    
	if (allow_failsafe && near.black_pixel_count == 0) {
		failsafe();
}

    
    // === Step 2: PD control ===
    int dv = compute_dv(near.error);
    
    // === Step 3: Apply dv ===
    int left_speed = SPEED_STOP  +  base_speed  + dv;
    int right_speed = SPEED_STOP - base_speed + dv;

    // === Step 4: Clamp speeds to motor range ===
    left_speed = clamp(left_speed, 30, 65);
    right_speed = clamp(right_speed, 30, 65);

    drive_motors(left_speed, right_speed);

    // === Step 5: Debug Output ===
    std::cout << "[" << currentQ << "] Err: " << std::setw(4) << error
              << " | dv: " << std::setw(4) << dv
              << " | L_PWM: " << std::setw(4) << left_speed
              << " | R_PWM: " << std::setw(4) << right_speed
              << std::endl;
              
    std::cout << "[RESULT] NEAR: Black pixels total: " << near.black_pixel_count << "\n";
    std::cout << "[RESULT] NEAR: Left-side black pixels: " << near.left_black << "\n";
    std::cout << "[RESULT] NEAR: Right-side black pixels: " << near.right_black << "\n";
    std::cout << "[RESULT] NEAR: Density: " << near.black_density << "\n";
}


