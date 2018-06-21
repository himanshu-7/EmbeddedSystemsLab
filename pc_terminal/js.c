
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include "joystick.h"
#include "../crc.h"
#include "protocol.h"

#define JS_DEV	"/dev/input/js0"


/* current axis and button readings
 */

int fd;
char BUTTON;
struct js_event js;
struct packet j_obj;
int16_t	axis[6];
int8_t new_axis[6];
int8_t temp_axis[6];
int	button[12];


unsigned int    mon_time_ms(void)
{
        unsigned int    ms;
        struct timeval  tv;
        struct timezone tz;

        gettimeofday(&tv, &tz);
        ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
        ms = ms + tv.tv_usec / 1000;
        return ms;
}

void    mon_delay_ms(unsigned int ms)
{
        struct timespec req, rem;

        req.tv_sec = ms / 1000;
        req.tv_nsec = 1000000 * (ms % 1000);
        assert(nanosleep(&req,&rem) == 0);
}

void init_js(){

	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}

	fcntl(fd, F_SETFL, O_NONBLOCK);
}



void make_j_packet(){

	j_obj.header = J_CONTROL;
	//j_obj.crc8 = 0x00;
	uint8_t lift_thing = (uint8_t)((int16_t)(-1*new_axis[0])+127);
	//printf("\n\nlift_thing: %d\n", lift_thing);
	new_axis[0] = lift_thing;
	j_obj.crc8 = make_crc8_tabled(j_obj.header, (uint8_t*)new_axis, 4);
	rs232_putchar(j_obj.header);
	for (uint8_t i = 0; i < 4; i++) {
		rs232_putchar(new_axis[i]);
	}
	rs232_putchar(j_obj.crc8);
	
}


//Read the joystick input and determine the input command

void read_js_values(){
	unsigned int range = 65535;
	unsigned int new_range = 255;
	int min = -32768;
	int new_min = -128;

	while(read(fd, &js, sizeof(struct js_event)) == 
	       			sizeof(struct js_event))  {
		
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				temp_axis[js.number] = (int8_t)(((js.value - min) * new_range) / range) + new_min;   //scale down joystick values to 1 byte
				break;
		}
	}
	
//if joystick cable unplugged go to panic mode.

	if (errno != EAGAIN) {
		j_obj.header=MODESET;
		j_obj.data=PANIC;
		j_obj.crc8=make_crc8_tabled(j_obj.header, &j_obj.data, 1);								
		rs232_putchar(j_obj.header);
		rs232_putchar(j_obj.data);
		rs232_putchar(j_obj.crc8);
		sleep(1);
		perror("\njs: error reading (EAGAIN)");
		exit (1);
	}
}


/*-----------------------------------------------------------------------------------------
* send_j_packet() -	function to detect joystick event, create appropriate packet and 
* 						send it  to the controller.
*
* Author: Himanshu Shah
* Date : 23/05/18
*------------------------------------------------------------------------------------------
*/

void send_j_packet()
{
	

	read_js_values();

//Joystick commands should be in order:
//0 -> Lift, 1 -> Roll, 2 -> Pitch, 3 -> Yaw
	new_axis[0] = temp_axis[3];
	new_axis[1] = temp_axis[0];
	new_axis[2] = temp_axis[1];
	new_axis[3] = temp_axis[2];

	if (button[0]){BUTTON='0';}
	else if (button[1]){BUTTON='1';}
	else if (button[2]){BUTTON='2';}
	else if (button[3]){BUTTON='3';}
	else if (button[4]){BUTTON='4';}
	else if (button[5]){BUTTON='5';}
	else if (button[6]){BUTTON='6';}
	else if (button[7]){BUTTON='7';}
	else if (button[8]){BUTTON='8';}
	else {BUTTON='9';}

	switch(BUTTON)
	{
		//safe mode
		case '0':	j_obj.header=MODESET;
				 	j_obj.data=PANIC;
				 	j_obj.crc8=make_crc8_tabled(j_obj.header, &j_obj.data, 1);								
				 	//term_puts("Entering PANIC mode.....");
					rs232_putchar(j_obj.header);
				 	rs232_putchar(j_obj.data);
				 	rs232_putchar(j_obj.crc8);
					break;

		//panic mode 
		case '1':	j_obj.header=MODESET;
				 	j_obj.data=PANIC;
				 	j_obj.crc8=make_crc8_tabled(j_obj.header, &j_obj.data, 1);								
				 	//term_puts("Entering PANIC mode.....");
					rs232_putchar(j_obj.header);
				 	rs232_putchar(j_obj.data);
				 	rs232_putchar(j_obj.crc8);
					break;

		//manual mode 
		case '2':	j_obj.header=MODESET;
				 	j_obj.data=MANUAL;
				 	j_obj.crc8=make_crc8_tabled(j_obj.header, &j_obj.data, 1);								
				 	//term_puts("Entering Manual mode.....");
					rs232_putchar(j_obj.header);
				 	rs232_putchar(j_obj.data);
				 	rs232_putchar(j_obj.crc8);
					break;
				
		//calibration mode 
		case '3':	j_obj.header=MODESET;
				 	j_obj.data=CALIBRATION_ENTER;
				 	j_obj.crc8=make_crc8_tabled(j_obj.header, &j_obj.data, 1);								
				 	//term_puts("Entering Calibration mode.....");
					rs232_putchar(j_obj.header);
				 	rs232_putchar(j_obj.data);
				 	rs232_putchar(j_obj.crc8);
					break;
				
		//yaw control 
		case '4':	j_obj.header=MODESET;
				 	j_obj.data=YAWCONTROL;
				 	j_obj.crc8=make_crc8_tabled(j_obj.header, &j_obj.data, 1);								
				 	//term_puts("Entering Yaw mode.....");
					rs232_putchar(j_obj.header);
				 	rs232_putchar(j_obj.data);
				 	rs232_putchar(j_obj.crc8);
					break;
				
		//full control
		case '5':	j_obj.header=MODESET;
				 	j_obj.data=FULLCONTROL;
				 	j_obj.crc8=make_crc8_tabled(j_obj.header, &j_obj.data, 1);								
				 	//term_puts("Entering Full mode.....");
					rs232_putchar(j_obj.header);
				 	rs232_putchar(j_obj.data);
				 	rs232_putchar(j_obj.crc8);
					break;
		default: break;			
	}
	
	//compile the command into a packet and send to quadcopter
	make_j_packet();
	
	
}
