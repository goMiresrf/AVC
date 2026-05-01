#include <iostream>
#include <iomanip>
#include <cmath>
#include "E101.h"
#include <csignal>
#include "AVC_utils.h"

int main() {
    int err = init(0);
    std:: cout<<"Error: "<<err<<std:: endl;
    signal(SIGINT, handle_sigint); // Capture Ctrl+C
    open_screen_stream();	
	tilt_camera("DOWN");
	
while (true){

	take_picture();
	update_screen(); 	

if (!gate_opened) {
	char request[] = "Please";
	char server[] = "130.195.3.53";
	char response[24];
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
    return 0;
}
