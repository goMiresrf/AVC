#include <iostream>
#include <iomanip>
#include <cmath>
#include "E101.h"
#include <csignal>
#include "AVC_utils.h"

using namespace std;

int post_turn_cooldown_this = 0;
int spin_strength_override_this = -1;  // -1 means "no override"


void spin_until_aligned_this(int direction) {
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
        int spin_strength = (spin_strength_override_this > 0) ? spin_strength_override_this : std::max(SPIN_SPEED_MIN, std::min((int)(SPIN_SPEED_MAX * shaped), SPIN_SPEED_MAX));

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
    post_turn_cooldown_this = 10;
    sleep1(200);
      

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



/*void spin_until_aligned_this_3(int direction) {
    const int SPIN_SPEED_MAX = 8;
    const int ERROR_START_THRESHOLD = 40;
    const int ERROR_END_THRESHOLD = 10;
    const int MIN_SPIN_FRAMES = 6;
    const int MAX_SPIN_FRAMES = 60;

    bool saw_big_error = false;
    int spin_frames = 0;

    std::cout << "[SPIN] Checking for intersection before spin...\n";

    // === Step 1: Detect intersection before advancing
    
    BlackPixelAnalysis initial = detect_black_line(ROW_NEAR);
    update_intersection_flags();

    bool saw_side_lines = (lineOnLeft || lineOnRight);

    if (!saw_side_lines) {
        std::cout << "[SPIN] No side lines detected .\n";
        return;
    }

    std::cout << "[SPIN] Intersection confirmed — forward until side lines disappear...\n";

    // === Step 2: Drive forward until both side lines disappear
    while (true) {
        take_picture();
        update_screen();
        //detect_black_line(200); // optional for error context
        update_intersection_flags();

        if (!lineOnLeft && !lineOnRight) {
            std::cout << "[SPIN] Side lines gone — ready to begin spin.\n";
            break;
        }

        drive_motors(50,46);
        sleep1(50);
    }

    drive_motors(SPEED_STOP, SPEED_STOP);
    sleep1(100);

    std::cout << "[SPIN] Starting spin " << (direction > 0 ? "RIGHT" : "LEFT") << "...\n";

    // === Step 3: Spin until aligned and side lines are cleared
    while (true) {
        take_picture();
        BlackPixelAnalysis near = detect_black_line(200);
        update_intersection_flags();

        float error = near.error;
        spin_frames++;

        if (std::abs(error) > ERROR_START_THRESHOLD) {
            saw_big_error = true;
        }

        bool visually_aligned = saw_big_error &&
                                std::abs(error) < ERROR_END_THRESHOLD &&
                                spin_frames >= MIN_SPIN_FRAMES &&
                                near.black_density > 0.2f;

        bool side_lines_cleared = lineOnTop && !lineOnLeft && !lineOnRight;

        if (visually_aligned && side_lines_cleared) {
            std::cout << "[SPIN] Alignment complete AND side lines gone — safe to exit spin.\n";
            break;
        }

        if (spin_frames >= MAX_SPIN_FRAMES) {
            std::cout << "[SPIN] Timeout — forced stop.\n";
            failsafe();
            break;
        }

        float norm_error = std::min(std::abs(error) / 100.0f, 1.0f);
        float shaped = std::pow(norm_error, 1.5f);
        const int SPIN_SPEED_MIN = 4;
        int spin_strength = std::max(SPIN_SPEED_MIN, std::min((int)(SPIN_SPEED_MAX * shaped), SPIN_SPEED_MAX));


        int left_pwm  = SPEED_STOP + direction * spin_strength;
        int right_pwm = SPEED_STOP + direction * spin_strength;

        drive_motors(left_pwm, right_pwm);

        std::cout << "[SPIN] Turning " << (direction > 0 ? "RIGHT" : "LEFT")
                  << " | Error: " << error
                  << " | SpinStrength: " << spin_strength
                  << " | L:" << lineOnLeft << " R:" << lineOnRight
                  << " | T:" << lineOnTop << "\n";

        sleep1(50);
    }

    drive_motors(SPEED_STOP, SPEED_STOP);
    sleep1(200);
}

void spin_until_aligned_this(int direction) {
    const int SPIN_SPEED_MAX = 8;
    const int ERROR_START_THRESHOLD = 40;
    const int ERROR_END_THRESHOLD = 10;
    const int MIN_SPIN_FRAMES = 6;
    const int MAX_SPIN_FRAMES = 60;

    bool saw_big_error = false;
    bool saw_side_lines = false;
    int spin_frames = 0;

    std::cout << "[SPIN] Starting spin " << (direction > 0 ? "RIGHT" : "LEFT") << "\n";

    while (true) {
				
        BlackPixelAnalysis near = detect_black_line(200);
        update_intersection_flags();

        float error = near.error;
        spin_frames++;

        // Mark if we saw strong deviation
        if (std::abs(error) > ERROR_START_THRESHOLD) {
            saw_big_error = true;
        }

        // Mark if we saw an intersection (T)
        if (lineOnLeft || lineOnRight) {
            saw_side_lines = true;
        }

        // === Condition 1: Alignment confirmed visually ===
        bool visually_aligned = saw_big_error &&
                          std::abs(error) < ERROR_END_THRESHOLD &&
                          spin_frames >= MIN_SPIN_FRAMES &&
                          near.black_density > 0.2f;  // at least some black line detected

        
        // === Condition 2: T-junction exited ===
        bool side_lines_cleared =  saw_side_lines && lineOnTop && !lineOnLeft && !lineOnRight;
        
        
       

			if (visually_aligned && side_lines_cleared) {
				std::cout << "[SPIN] Alignment complete AND side lines gone — safe to exit spin.\n";
				break;
}

            if (spin_frames >= MAX_SPIN_FRAMES) {
            std::cout << "[SPIN] Timeout — forced stop.\n";
            break;
        }

        // === Dynamic spin strength ===
        float norm_error = std::min(std::abs(error) / 100.0f, 1.0f);
        float shaped = std::pow(norm_error, 1.5f);
        int spin_strength = std::min((int)(SPIN_SPEED_MAX * shaped), SPIN_SPEED_MAX);

        int left_pwm  = SPEED_STOP + direction * spin_strength;
        int right_pwm = SPEED_STOP + direction * spin_strength;

        drive_motors(left_pwm, right_pwm);

        std::cout << "[SPIN] Turning " << (direction > 0 ? "RIGHT" : "LEFT")
                  << " | Error: " << error
                  << " | SpinStrength: " << spin_strength
                  << " | SawBigErr: " << saw_big_error
                  << " | L:" << lineOnLeft << " R:" << lineOnRight << "\n";

        sleep1(50);
    }

    drive_motors(SPEED_STOP, SPEED_STOP);
    sleep1(200);
}*/

void Q3_test() {

    static int turn_cooldown = 0;  // prevents back-to-back false turns
    static bool intersection_turned = false;
    static bool intersection_turned_2 = false;    
    
    
    update_intersection_flags();

    if (turn_cooldown > 0) {
        turn_cooldown--;
        return;
    }

    if (post_turn_cooldown_this > 0) {
        post_turn_cooldown_this--;
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
        spin_until_aligned_this(+1);
        turn_cooldown = 10;
        return;
    }

    if (confirmed_junction && intersection_turned) {
        std::cout << "[Q3 TEST] Cleanup spin — turning left.\n";
        drive_forward();
        spin_strength_override_this = 8;
        spin_until_aligned_this(-1);
        spin_strength_override_this = -1;
        turn_cooldown = 10;
        intersection_turned_2 = true;
        intersection_turned = false;
        return;
    }

    if (confirmed_junction && !intersection_turned) {
        std::cout << "[Q3 TEST] First turn — spinning left.\n";
        drive_forward();
        spin_until_aligned_this(-1);
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
        spin_until_aligned_this(+1);
        turn_cooldown = 10;
        return;
    }

    if (sharp_left) {
        std::cout << "[Q3 TEST] Sharp LEFT confirmed — spinning left.\n";
        drive_forward();
        spin_until_aligned_this(-1);
        turn_cooldown = 10;
        return;
    }

    // === No spin — follow line normally ===
    line_following(ROW_Q3);
}
/*void drive_forward_until_cleared() {
   
    while (true) {
		take_picture();
		update_screen();
        update_intersection_flags();

		if (lineOnLeft && lineOnRight) {
            std::cout << "[ADVANCE] Side lines gone — stopping advance.\n";
            break;
        }

        std::cout << "[T-JUNC] Driving forward... L:" << lineOnLeft << " R:" << lineOnRight << "\n";
        
        drive_motors(50,46);
        
        sleep1(50);
    }

    // Stop when side lines are gone
    drive_motors(SPEED_STOP, SPEED_STOP);
    sleep1(200);
}*/


/*void Q3_test_2() {
    
    static int turn_cooldown = 0;  // prevents back-to-back false turns
    BlackPixelAnalysis near = detect_black_line(200);
    update_intersection_flags();

    if (turn_cooldown > 0) {
        turn_cooldown--;
        return;
    }
	
	static bool intersection_turned= false;
	static bool intersection_turned_2=false;
	
    // === Detect Sharp RIGHT Turn ===
    bool sharp_right = near.black_density > 0.3f && lineOnRight;// && near.right_black > 2.0f * near.left_black && near.black_pixel_count > 110;

    // === Detect Sharp LEFT Turn ===
    bool sharp_left = near.black_density > 0.3f && lineOnLeft;// && near.left_black > 2.0f * near.right_black && near.black_pixel_count > 110;
	
	bool intersection = (near.black_density > 0.3f) && lineOnLeft && lineOnRight;
	
	//bool intersection_2 = (near.black_density > 0.5f) && lineOnLeft && lineOnRight && lineOnTop;
	
	
	
	
	if (intersection_turned_2 && intersection){
		std::cout << "[Q3 TEST] Sharp right confirmed — spinning right...\n";
        spin_until_aligned_this(+1);
        turn_cooldown = 10;
        return;
				
	}
	
	
if (intersection && intersection_turned) {
    std::cout << "[Q3 TEST] Sharp LEFT confirmed — spinning left...\n";
    drive_forward_until_cleared();
    spin_until_aligned_this(-1);
	turn_cooldown = 10;
    intersection_turned_2 = true;
    intersection_turned= false;
    return;
}
    
if (intersection) {
    std::cout << "[Q3 TEST] Sharp LEFT confirmed — spinning left...\n";
    update_intersection_flags();
    spin_until_aligned_this(-1);
    turn_cooldown = 10;
    std::cout << "[Q3 TEST] Side lines gone — confirming turn.\n";
    intersection_turned = true;
    return;
}

	
    if (!intersection && sharp_right) {
        std::cout << "[Q3 TEST] Sharp RIGHT confirmed — spinning right...\n";
        spin_until_aligned_this(+1);
        turn_cooldown = 10;
        return;
    }

    if (!intersection && sharp_left) {
        std::cout << "[Q3 TEST] Sharp LEFT confirmed — spinning left...\n";
        spin_until_aligned_this(-1);
        turn_cooldown = 10;
        return;
    }
    

    // === No spin — follow line normally ===
    base_speed = 5;
    
    line_following();
}*/


int main() {
  
    init(0);
    signal(SIGINT, handle_sigint);
    open_screen_stream();
	tilt_camera("DOWN");
	
    while (true) {
       
		take_picture();
		update_screen();
		//update_intersection_flags();
		//line_following_Q3();
		Q3_test();
    
}

return 0;

}
