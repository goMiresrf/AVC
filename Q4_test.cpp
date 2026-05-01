#include <iostream>
#include <iomanip>
#include <cmath>
#include "E101.h"
#include <csignal>
#include <string>
#include "AVC_utils.h"




void spin_until_colour_clear_this(Colour c1, Colour c2 = NONE, int direction = +1) {
    while (true) {
        take_picture();
        ColourCentroid cc = detect_colour_centroids();  // You already wrote this

        bool c1_detected = false, c2_detected = false;

        if (c1 == RED)   c1_detected = cc.red_count > 20;
        if (c1 == GREEN) c1_detected = cc.green_count > 20;
        if (c1 == BLUE)  c1_detected = cc.blue_count > 20;

        if (c2 != NONE) {
            if (c2 == RED)   c2_detected = cc.red_count > 20;
            if (c2 == GREEN) c2_detected = cc.green_count > 20;
            if (c2 == BLUE)  c2_detected = cc.blue_count > 20;
        }

        if (!c1_detected && !c2_detected) break;

        int left_pwm  = SPEED_STOP + direction * 4;
        int right_pwm = SPEED_STOP + direction * 4;
        drive_motors(left_pwm, right_pwm);
        sleep1(30);
    }

    drive_motors(SPEED_STOP, SPEED_STOP);
}

void spin_until_colour_detected_this(Colour color, int threshold, int direction = +1) {
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




void run_Q4() {
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





int main() {

	currentQ = Q4;
    init(0);
    signal(SIGINT, handle_sigint);
    open_screen_stream();
    q4_state = Q4_STATE1;  
	tilt_camera("UP");

while(true){
	take_picture();
	update_screen();
	//detect_colour_centroids();
	run_Q4();
}


return 0 ;
}
