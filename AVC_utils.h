// AVC_utils.h
#pragma once

#include <string>


void handle_Q1();
void handle_Q2();
void handle_Q3();
void handle_Q4();



// --- Motor Assignments ---
const int MOTOR_LEFT   = 1;
const int MOTOR_RIGHT  = 3;
const int MOTOR_CAMERA = 5;

// --- Scanlines ---
const int ROW_NEAR = 90; // Near row to scan
const int ROW_FAR = 60;
const int ROW_Q3  = 160; 
const int IMG_WIDTH  = 320;
const int IMG_HEIGHT = 240;
const int IMG_CENTER = IMG_WIDTH / 2;
const int TOTAL_PIXELS = 320 * 240;  // Full camera frame


// --- Speed & PID Constants ---

const int base_speed = 5;
const int SPEED_STOP= 48;
const int MAX_SPEED_FORWARD_L  = 65;
const int MAX_SPEED_REVERSE_L  = 30;
const int MAX_SPEED_FORWARD_R  = 30;
const int MAX_SPEED_REVERSE_R  = 65;

const int MAX_DV = MAX_SPEED_FORWARD_L - SPEED_STOP; // = 17 Clamp dv to stay within motor capability range
const int MIN_DV = MAX_SPEED_FORWARD_R - SPEED_STOP; // = -18

 
//---  Q1 variables ------------

extern bool gate_opened;




// --- Q3 variables ------------

extern bool lineOnTop;
extern bool lineOnLeft;
extern bool lineOnRight;
extern bool allow_failsafe;

extern int post_turn_cooldown;
extern int spin_strength_override;  // -1 means "no override"

void line_following_Q3();






// --- Q4 variables ------------

enum Colour { RED, GREEN, BLUE, NONE };
enum Q4State {
    Q4_STATE0,  // idle / entry
    Q4_STATE1,  // green approach
    Q4_STATE2,  // green reverse + spin
    Q4_STATE3,  // red approach
    Q4_STATE4,  // red reverse + spin
    Q4_STATE5,  // blue approach
    Q4_STATE6,  // blue reverse + spin
    Q4_STATE7,  // final red approach
    Q4_DONE     // stop
};
enum ColourGoalType { APPROACH, CLEAR };

extern Q4State q4_state;

struct ColourFollowSettings {
    int start_approach_threshold; // when to start approaching
    int approach_threshold;   // when to stop approaching
    int clear_threshold;      // when to stop reversing
    int reacquire_threshold;  // how many pixels = "found again"
    int base_speed;           // forward/reverse motor base
    int spin_speed;           // used for reacquiring
};
struct ColourCentroid {
    int red_count;
    int red_centroid;
    int green_count;
    int green_centroid;
    int blue_count;
    int blue_centroid;
};

ColourCentroid detect_colour_centroids();

ColourFollowSettings get_settings_for(Colour c);


void follow_colour(Colour color, ColourGoalType goal, const ColourFollowSettings& settings, bool forward );
void spin_until_colour_detected(Colour color, int threshold, int direction);


// =========== GLOBAL SHARED Functions===========================================================================================


// --- Current Quadrant ---//
enum   Quadrant {Q1, Q2, Q3, Q4 }; // FSM, keeps track of what quadrant we in
extern Quadrant currentQ;


//--- line analysis ---//
struct BlackPixelAnalysis {
    int error;
    int black_pixel_count;
    int left_black;
    int right_black;
    float black_density;
    float imbalance;
};

BlackPixelAnalysis detect_black_line(int row);


void handle_sigint(int signum);

int clamp(int value, int min_val, int max_val);

float clampf(float value, float min_val, float max_val);
							
void tilt_camera(const std::string& direction);

void drive_motors(float left_speed, float right_speed);

int compute_dv(float error);

void spin_until_aligned(int direction);

bool red_patch_detected(int row);

void failsafe();

void update_intersection_flags() ;

void drive_forward();

void line_following(int row);



