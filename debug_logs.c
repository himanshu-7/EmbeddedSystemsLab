//***************** Documentation Section ****************************//
// This code logs the data from the sensors for debugging purpose
// Author : Himanshu Shah
// Date: 02/05/18
//********************************************************************//

#include "in4073.h"

#define TIME        1
#define MODE        4
#define BAT         5
#define PRESSURE    7
#define GYROSP      11
#define GYROSQ      13
#define GYROSR      15
#define PHI         17
#define THETA       19
#define PSI         21


uint8_t data_bytes;
uint8_t write_buffer[data_bytes];
uint8_t read_buffer[data_bytes];

void save_log(){

    time = get_time_us;
    write_buffer[address+TIME] = (time>>24) & 0xFF;
    write_buffer[address+TIME+2] = (time>>16) & 0xFF;
    write_buffer[address+TIME+1] = (time>>8) & 0xFF;
    write_buffer[address+TIME+3] = (time) & 0xFF;

    write_buffer[address+MODE] = 0x00;                                     //current mode value

    write_buffer[address+BAT] = (bat_volt>>8)  & 0xFF;
    write_buffer[address+BAT+1] =  bat_volt & 0xFF;

    write_buffer[address+PRESSURE] = (pressure>>24) & 0xFF;
    write_buffer[address+PRESSURE+1] = (pressure>>16) & 0xFF;
    write_buffer[address+PRESSURE+2] = (pressure>>8) & 0xFF;
    write_buffer[address+PRESSURE+3] = pressure & 0xFF;

    write_buffer[address+GYROSP] = (sp>>8) & 0xFF;
    write_buffer[address+GYROSP+1] = (sp) & 0xFF;
    write_buffer[address+GYROSQ] = (sq>>8) & 0xFF;
    write_buffer[address+GYROSQ+1] = (sq) & 0xFF;
    write_buffer[address+GYROSR] = (sr>>8) & 0xFF;
    write_buffer[address+GYROSR+1] = (sr) & 0xFF;

    write_buffer[address+PHI] = (phi>>8) & 0xFF;
    write_buffer[address+PHI+1] = (phi) & 0xFF;
    write_buffer[address+THETA] = (theta>>8) & 0xFF;
    write_buffer[address+THETA+1] = (theta) & 0xFF;
    write_buffer[address+PSI] = (psi>>8) & 0xFF;
    write_buffer[address+PSI+1] = psi & 0xFF;

    if(flash_write_bytes(address, write_buffer, data_bytes)){

        if(address+=data_bytes > 0xFFFFFF){

            if(flash_chip_erase()){

                address=0x000000;
            }
            else{
                printf("Error while erasing flash chip");
            }
        }
        else{
            address+=data_bytes;
        }
    }
    else{
        printf("Flash write failed!!");
    }
}

void send_log(){
    
    uint32_t s_time, s_pressure;
    uint16_t s_bat_volt, s_phi, s_theta, s_psi, s_sp, s_sq, s_sr;
    uint8_t s_mode;

    if(address + data_bytes > 0xFFFFFF){

        printf("Requested read from flash could not be performed(out of bounds)");
    }
    else{
        if(!flash_read_bytes(address, read_buffer, data_bytes)){

            printf("Read from flash failed!");
        }
        else{
            s_time=read_buffer[address+TIME]<<24 + read_buffer[address+TIME+1]<<16 + read_buffer[address+TIME+2]<<8 + read_buffer[address+TIME+4];
            s_mode = read_buffer[address+MODE];
            s_bat_volt = read_buffer[address+BAT]<<8 + read_buffer[address+BAT+1];
            s_pressure = read_buffer[address+PRESSURE]<<24 +read_buffer[address+PRESSURE+1]<<16 + read_buffer[address+PRESSURE+2]<<8 + read_buffer[address+PRESSURE+4];
            s_sp = read_buffer[address+GYROSP]<<8 + read_buffer[address+GYROSP+1];
            s_sq = read_buffer[address+GYROSQ]<<8 + read_buffer[address+GYROSQ+1];
            s_sr = read_buffer[address+GYROSR]<<8 + read_buffer[address+GYROSR+1];
            s_phi = read_buffer[address+PHI]<<8 + read_buffer[address+PHI+1];
            s_theta = read_buffer[address+theta]<<8 + read_buffer[address+THETA+1];
            s_psi = read_buffer[address+psi]<<8 + read_buffer[address+psi+1];
            address+=data_bytes;

            //put code to inform PC that data logging packet is about to be received 

            uart_put((s_time>>24) & 0xFF);
            uart_put((s_time>>16) & 0xFF);
            uart_put((s_time>>8) & 0xFF);
            uart_put(s_time & 0xFF);
            uart_put(s_mode);
            uart_put((s_bat_volt>>8) & 0xFF);
            uart_put((s_bat_volt) & 0xFF);
            uart_put((s_pressure>>24) & 0xFF);
            uart_put((s_pressure>>16) & 0xFF);
            uart_put((s_pressure>>8) & 0xFF);
            uart_put(s_pressure & 0xFF);
            uart_put((s_sp>>8) & 0xFF);
            uart_put((s_sp) & 0xFF);
            uart_put((s_sq>>8) & 0xFF);
            uart_put((s_sq) & 0xFF);
            uart_put((s_sr>>8) & 0xFF);
            uart_put((s_sr) & 0xFF);
            uart_put((s_phi>>8) & 0xFF);
            uart_put((s_phi) & 0xFF);
            uart_put((s_theta>>8) & 0xFF);
            uart_put((s_theta) & 0xFF);
            uart_put((s_psi>>8) & 0xFF);
            uart_put((s_psi) & 0xFF);
        }
    }
}


