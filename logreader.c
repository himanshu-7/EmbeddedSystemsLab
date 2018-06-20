/*------------------------------------------------------------------
 * logreader.c
 * Log function to read the embedded flash log to a file on the PC
 * This code is intended to run on a PC
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/

#include "in4073.h"
#define LOG_FILENAME "log.txt"

#define INIT_ST     0x0
#define READ_ST     0x1

void send_logdump_req_packet(){
    struct packet req_p;
    req_p.header = MODESET;
    req_p.data = DUMPLOGS;
    req_p.crc8 = make_crc8_tabled(req_p.header, &req_p.data, 1);
    rs232_putchar(req_p.header);
    rs232_putchar(req_p.data);
    rs232_putchar(req_p.crc8);
    return;
}

bool read_log_entry(uint8_t *buffer){
    uint8_t reader_state = INIT_ST;
    uint8_t log_entry[LOG_ENTRY_SIZE_BYTES] = {0};
    uint8_t header = 0;
    uint8_t crc_packet = 0;
    uint8_t crc_check = 0;
    int i = 0;
    while(i < LOG_ENTRY_SIZE_BYTES){
        switch(reader_state){
            case INIT_ST:  
                // Keep looking for header packet  
                if(rx_queue.count > 0){
                    header = dequeue(&rx_queue);
                    if(header == LOG_CHAR){
                        reader_state = READ_ST;
                    } else {
                        printf("WARNING: Received non-log packet while reading log!");
                    }
                }
                break;
                // Read rest of the data
            case READ_ST:
                if(rx_queue.count > 0){
                    log_entry[i] = dequeue(&rx_queue);
                    i++;
                }
                break;
        }
    }
    // Obtain CRC packet
    while(i < LOG_ENTRY_SIZE_BYTES + 1)
    {
        if(rx_queue.count > 0){
            crc_packet = dequeue(&rx_queue);
        }
    }
    crc_check = make_crc8_tabled(header, log_entry, LOG_ENTRY_SIZE_BYTES);
    if(crc_packet == crc_check){
        return true;
    } else {
        printf("WARNING: Log entry CRC mismatch");
        return false;
    }
}

bool read_log_to_file(){
    // Create log file. Overwrite if already exists
    FILE *f = fopen(LOG_FILENAME, "w");
    if(f == NULL){
        printf("ERROR: Could not create log file");      
        return false;
    }
    uint32_t addr = 0x000000;
    bool read_ok = true;
    uint8_t log_entry[LOG_ENTRY_SIZE_BYTES] = {0};
    send_logdump_req_packet();
    // Read log data from flash per entry
    while(((addr + LOG_ENTRY_SIZE_BYTES) <= FLASH_ADDR_LIMIT) && read_ok){
        int i = 0;
        bool success = read_log_entry(log_entry);
        while(i < LOG_ENTRY_SIZE_BYTES){
            if(success){
                fprintf(f, "%c", (char)log_entry[i]);
                i++;
            }
        }
        // Write EOL
        fprintf(f, "%s","\n");
        // Iterate address
        addr += LOG_ENTRY_SIZE_BYTES;
    }
    if (!read_ok){
        printf("ERROR: Log read failed!\n");
        return false;
    }
    return true;
}