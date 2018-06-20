/*------------------------------------------------------------------
 * logger.c
 * Logging functions to write log data to the on-board flash
 * All code is intended to run on the microcontroller
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/
#include "in4073.h"

#define LOG_WRITE_DEBUG 0
#define LOG_READ_DEBUG  0
#define LOG_TERMINAL    1

uint32_t prev_log_time;
uint32_t write_addr;						
uint32_t addr_before_overflow;				// Last address written to before overflow occurred	
int8_t curr_flash_block;					// 4KB flash block currently being written to
bool log_init_done;
bool log_err;
bool log_err_change;
bool flash_overflow;						// True if flash overflow has been triggered

// Initialize the logfile
bool log_init(){
	prev_log_time = get_time_us();
    write_addr = 0x000000;
    addr_before_overflow = 0x000000;
    curr_flash_block = -1;
    flash_overflow = false;
    log_err = false;
	log_err_change = false;
    if(spi_flash_init()){
        log_init_done = true;
        return log_init_done;
    } else {
        printf("ERROR: SPI Flash initialization failed\n");
        return false;
    }
}

// Write relevant log data to the flash memory at the specified address
// Returns true on success, false on failure
bool log_write(uint32_t addr){
    if(!log_init_done){
        printf("ERROR: Flash not initalized yet!");
        return false;
    }
    // If address out of bounds
    if(addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
        printf("ERROR: Address(%lx) + log size(%x) out of bounds(%x)\n", addr, LOG_ENTRY_SIZE_BYTES, FLASH_ADDR_LIMIT);
        return false;
    } else {
        //Construct log array
        uint8_t array[LOG_ENTRY_SIZE_BYTES];
        /* Time */
        uint32_t t_cur = get_time_us();
        array[0] = t_cur & 0xFF;
        array[1] = (t_cur >> (8*1)) & 0xFF;
        array[2] = (t_cur >> (8*2)) & 0xFF;
        array[3] = (t_cur >> (8*3)) & 0xFF;
        /* State */
        array[4] = QuadState;
        /* Motor values */
        array[5] = ae[0] & 0xFF;
        array[6] = (ae[0] >> 8) & 0xFF;
        array[7] = ae[1] & 0xFF;
        array[8] = (ae[1] >> 8) & 0xFF;
        array[9] = ae[2] & 0xFF;
        array[10] = (ae[2] >> 8) & 0xFF;
        array[11] = ae[3] & 0xFF;
        array[12] = (ae[3] >> 8) & 0xFF;
        /* MPU Data */
        // phi
        array[13] = (phi-phi_o) & 0xFF;
        array[14] = ((phi-phi_o) >> 8) & 0xFF;
        // theta
        array[15] = (theta-theta_o) & 0xFF;
        array[16] = ((theta-theta_o) >> 8) & 0xFF;
        // psi
        array[17] = (psi-psi_o) & 0xFF;
        array[18] = ((psi-psi_o) >> 8) & 0xFF;
        // sp
        array[19] = (sp-sp_o) & 0xFF;
        array[20] = ((sp-sp_o) >> 8) & 0xFF;
        // sq
        array[21] = (sq-sq_o) & 0xFF;
        array[22] = ((sq-sq_o) >> 8) & 0xFF;
        // sr
        array[23] = (sr-sr_o) & 0xFF;
        array[24] = ((sr-sr_o) >> 8) & 0xFF;
        /* Battery voltage */
        array[25] = bat_volt & 0xFF;
        array[26] = (bat_volt >> 8) & 0xFF;

        #if LOG_WRITE_DEBUG
        printf("Log Written: ");
        for(uint8_t i=0; i<LOG_ENTRY_SIZE_BYTES; i++){
            printf("%02X ", array[i]);
        }
        printf("\n");
        #endif

        if(flash_write_bytes(addr, array, LOG_ENTRY_SIZE_BYTES)){
            return true;
        } else {
            printf("ERROR: Flash write failed\n");
            return false;
        }
    }

}

// Push all log data to the PC
// Returns true on success, false on failure
bool log_read_entry(uint32_t addr){
    if(!log_init_done){
        printf("ERROR: Log not initialized\n");
        return false;
    }
    if(addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
        printf("ERROR: Address(%lx) + log size(%x) out of bounds(%x)\n", addr, LOG_ENTRY_SIZE_BYTES, FLASH_ADDR_LIMIT);
        return false;
    }
    // Create data buffer
    uint8_t data_buf[LOG_ENTRY_SIZE_BYTES];
    flash_read_bytes(addr, data_buf, LOG_ENTRY_SIZE_BYTES);
    uint8_t crc = make_crc8_tabled(BIG_PACKET, data_buf, LOG_ENTRY_SIZE_BYTES);
    #if LOG_READ_DEBUG
    printf("Entry(%lx): ", addr);
    #endif

    // Check for valid data
    uint8_t counter = 0;
    for(uint8_t i=0; i < LOG_ENTRY_SIZE_BYTES; i++){
        if(data_buf[i] == 0xFF){
            counter++;
        }
    }
    if(counter == LOG_ENTRY_SIZE_BYTES){
        printf("INFO: Empty flash space found.\n");
        return false;
    }


    #if LOG_TERMINAL
    // Print log entry to PC Terminal
    printf("%02X ", BIG_PACKET);
    for(uint8_t i=0; i < LOG_ENTRY_SIZE_BYTES; i++){
        printf("%02X ", data_buf[i]);
        nrf_delay_us(250);
    }
    printf("CRC: %02X\n", crc);
    nrf_delay_us(250);
    return true;
    #else
    // Send log entry to PC log reader
    struct packet log_p;
    log_p.header = LOG_CHAR;
    log_p.data = data_buf;
    log_p.crc8 = crc;
    rs232_putchar(log_p.header);
    rs232_putchar(log_p.data);
    rs232_putchar(log_p.crc8);
    #endif
}

void log_dump(){
    printf("INFO: Log dump started.\n");
    // Iterate starting from last address that contains data
    int32_t i = write_addr - LOG_ENTRY_SIZE_BYTES;
    // Normal readout
    while(i >= 0){
        if(!log_read_entry(i)){
            printf("ERROR: Log dump aborted early.\n");
            flash_overflow = false;
            break;
        } else {
            i -= LOG_ENTRY_SIZE_BYTES;
        }
    }
    // Overflow readout
    if(flash_overflow){
        // Read back from end of flash to last non-erased address
        i = addr_before_overflow - LOG_ENTRY_SIZE_BYTES;
        uint32_t lower_addr_limit = (curr_flash_block + 1) * 0x1000;
        // Read down to lower address limit
        while(i > lower_addr_limit){
            if(!log_read_entry(i)){
                printf("ERROR: Log dump aborted early.\n");
                break;
            } else {
                i -= LOG_ENTRY_SIZE_BYTES;
            }
        }			
    }
    printf("INFO: Log dump complete.\n");
    return;
}

void logger_main(){
    uint32_t cur_time = get_time_us();
    // Only log when flash is initialized, no logging errors have occured, and log period
    // has expired.
    if((log_init_done) && (!log_err) && (cur_time >= prev_log_time + LOG_PERIOD_US)){
        prev_log_time = cur_time;
        #if LOG_WRITE_DEBUG
        //printf("Logging to address: %lx\n", write_addr);
        #endif
        log_err = !log_write(write_addr);
        write_addr += LOG_ENTRY_SIZE_BYTES;
        // If there is flash overflow
        if(write_addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
            // Wrap around
            addr_before_overflow = write_addr - LOG_ENTRY_SIZE_BYTES;
            write_addr = 0;
            if(flash_overflow == false){
                printf("WARNING: Flash overflow detected: old data will be erased!\n");
            }
            flash_overflow = true;
        }
        // Erase flash sector if necessary
        if(flash_overflow){
            uint8_t requested_block = (uint8_t)floor(write_addr / 0x1000);
            if(curr_flash_block != requested_block){
                #if LOG_READ_DEBUG || LOG_WRITE_DEBUG
                    printf("Switching flash block to %i\n", requested_block);
                #endif
                if(flash_4k_sector_erase(requested_block)){
                    curr_flash_block = requested_block;
                } else {
                    printf("ERROR: 4k Sector erase failed.\n");
                }
            }
        }
    } else if (!log_err_change && log_err){
        printf("ERROR: Logging aborted!\n");
        log_err_change = true;
    }
}