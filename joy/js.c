
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "joystick.h"


/* current axis and button readings
 */
int	axis[6];
int new_axis[6];
int	button[12];


/* time
 */
#include <time.h>
#include <assert.h>
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


#define JS_DEV	"/dev/input/js0"

int main (int argc, char **argv)
{
	struct timeb time_buffer;
	unsigned int range = 65535;
	unsigned int new_range = 255;
	unsigned int max = 32767;
	int min = -32768;
	int new_min = -128;
	int 		fd;
	struct js_event js;
	unsigned int	t, i;

	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}

	/* non-blocking mode
	 */
	fcntl(fd, F_SETFL, O_NONBLOCK);


		ftime(&time_buffer);
		unsigned int curr_time= time_buffer.millitm;
		unsigned int fix_time = time_buffer.millitm;

	while (1) {

		/* simulate work
		 */
		//mon_delay_ms(300);
		//t = mon_time_ms();

		if((300 + fix_time)%1000 != curr_time){
			//printf("current time:%d, time:%d\n", curr_time, fix_time);
			ftime(&time_buffer);
			curr_time=time_buffer.millitm;
		}
		
		else{
			ftime(&time_buffer);
			fix_time = time_buffer.millitm;
			curr_time=fix_time;
			/* check up on JS
			 */
			while (read(fd, &js, sizeof(struct js_event)) == 
			       			sizeof(struct js_event))  {

				/* register data
				 */
				// fprintf(stderr,".");
				switch(js.type & ~JS_EVENT_INIT) {
					case JS_EVENT_BUTTON:
						button[js.number] = js.value;
						break;
					case JS_EVENT_AXIS:
						axis[js.number] = js.value;
						break;
				}
			}
			if (errno != EAGAIN) {
				perror("\njs: error reading (EAGAIN)");
				exit (1);
			}

			printf("\n");
			printf("%5d   ",t);
			for (i = 0; i < 6; i++) {
				printf("%6d ",new_axis[i] = (((axis[i] - min) * new_range) / range) + new_min);
			}
			printf(" |  ");
			for (i = 0; i < 12; i++) {
				printf("%d ",button[i]);
			}


			if (button[0])
				break;
		}
		
	}
	printf("\n<exit>\n");

}
