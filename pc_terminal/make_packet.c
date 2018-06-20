//*******************Documentation Section*************************
// This file maps the detected key from keyboard and builds a      
// corresponding packet and sends it to the quadcopter              
// 
// Author: Himanshu Shah                                           
// Date: 03/05/18                                                  
//*****************************************************************

#include<stdio.h>
#include "protocol.h"
#include "../crc.h"

void detect_term_input(char c){

    struct packet p_obj;
    int data_detected=0;

    switch(c)
    {
        case '0':
                 p_obj.header=MODESET;
                 p_obj.data=SAFE;
                 data_detected=1;
                 break;
        case '1':
                 p_obj.header=MODESET;
                 p_obj.data=PANIC;
                 data_detected=1;
                 break;
        case '2': 
                 p_obj.header=MODESET;
                 p_obj.data=MANUAL;
                 data_detected=1;
                 break;
        case '3':
                 p_obj.header=MODESET;
                 p_obj.data=CALIBRATION;
                 data_detected=1;
                 break;
        case '4':
                 p_obj.header=MODESET;
                 p_obj.data=YAWCONTROL;
                 data_detected=1;
                 break;
        case '5':
                 p_obj.header=MODESET;
                 p_obj.data=FULLCONTROL;
                 data_detected=1;
                 break;
        case '6':
                 p_obj.header=MODESET;
                 p_obj.data=RAW;
                 data_detected=1;
                 break;
        case '7':
                 p_obj.header=MODESET;
                 p_obj.data=HEIGHT;
                 data_detected=1;
                 break;
        case '8':
                 p_obj.header=MODESET;
                 p_obj.data=WIRELESS;
                 data_detected=1;
                 break;
        case '9':
                 p_obj.header=MODESET;
                 p_obj.data=DUMPLOGS;
                 data_detected=1;
                 break;
        case 'a':
                 p_obj.header=K_LIFT;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'z':
                 p_obj.header=K_LIFT;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'q':
                 p_obj.header=K_YAW;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'w':
                 p_obj.header=K_YAW;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'u':
                 p_obj.header=K_P;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'j': 
                 p_obj.header=K_P;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'i':
                 p_obj.header=K_P1;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'k': 
                 p_obj.header=K_P1;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'o':
                 p_obj.header=K_P2;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'l': 
                 p_obj.header=K_P2;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'y':
                 p_obj.header=K_HEIGHT;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'h':
                 p_obj.header=K_HEIGHT;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'A':
                 p_obj.header=K_PITCH;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'B':
                 p_obj.header=K_PITCH;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'C':
                 p_obj.header=K_ROLL;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'D':
                 p_obj.header=K_ROLL;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        default: 
                break;
    }
    if(data_detected == 1){
        p_obj.crc8 = make_crc8_tabled(p_obj.header, &p_obj.data, 1);
        rs232_putchar(p_obj.header);
        rs232_putchar(p_obj.data);
        rs232_putchar(p_obj.crc8);
    }

}