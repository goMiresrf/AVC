# Autonomous Vision-Based Robot

A Raspberry Pi-based autonomous robot developed for the ENGR101 Autonomous Vehicle Challenge.  
The robot uses real-time camera input, embedded C++ control logic, and adaptive PID steering to navigate a multi-stage course involving gate communication, line following, intersection handling, and colour-based object interaction.

## Demo

[Watch the demo video here](https://youtube.com/shorts/0gOaX4Mvt7Q?feature=share)


## Project Overview

This project focused on building a small autonomous vehicle capable of completing a four-quadrant robotics course. The system combined:

- UDP-based communication for gate access
- Camera-guided black line following
- Dynamic thresholding and centroid-based image processing
- Exponential PID steering control
- State-based turn handling for intersections and sharp turns
- Colour detection for red, green, and blue target objects

The robot was designed around a modular software structure so that individual subsystems could be tested and debugged separately before full-course integration.

## Key Features

### Real-Time Line Following

The robot captures camera frames and scans image rows to detect the black track line.  
A dynamic brightness threshold is calculated from each frame, allowing the robot to adapt to changing lighting conditions.

The line position is estimated using a weighted centroid of detected black pixels, producing a horizontal error value relative to the camera centre.

### Exponential PID Steering

Instead of using a fixed linear PID response, the robot applies exponential gain shaping.  
This allows small corrections near the centre line while still producing stronger steering adjustments when the robot is far from the line.

This helped reduce oscillation and improved stability during curved-line navigation.

### Intersection and Turn Logic

For the third quadrant, the robot uses a lightweight flag-based control system to detect and respond to T-junctions, sharp turns, and line realignment.

The system checks for line presence at the top, left, and right areas of the camera frame, then triggers controlled turning routines based on the current navigation stage.

### Colour-Based Object Handling

In the final quadrant, the robot detects coloured objects using RGB channel dominance thresholds.  
The robot can identify red, green, and blue targets, steer toward them using centroid-based control, reverse after reaching them, and reacquire targets if temporarily lost.

## Hardware

- Raspberry Pi Zero W
- Raspberry Pi Camera Module
- Differential drive motor setup
- Servo-mounted camera
- Portable USB power bank
- Custom robot chassis

## Software

The software was written in C++ and structured into modular files:

```text
.
├── AVC_ver3.cpp        # Main quadrant logic and robot behaviour
├── AVC_utils.cpp       # Helper functions for image processing and motor control
├── AVC_utils.h         # Function declarations and shared constants
├── Q1_test.cpp         # Gate communication testing
├── Q3_test.cpp         # Turn and intersection testing
├── Makefile            # Build and compilation setup
└── README.md