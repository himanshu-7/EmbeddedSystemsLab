/*----------------------------------------------------------------------
 * crc.c
 * CRC8 calculation functions for a byte array with arbitrary length
 * make_crc_tabled() uses tables for additional speedup at the cost
 * of memory, while make_crc_nontabled is slower, but has a much 
 * lower memory footprint.
 * 
 * Reference:
 * https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 * Author: Niels Versluis - 4227646
 *--------------------------------------------------------------------*/
#include <stdint.h> 
#include <stdio.h> 
#include "crc.h"

#define PC_BENCH 0

#if PC_BENCH
#include <sys/time.h>
#endif

#define WIDTH (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

// Tabled CRC calculation
uint8_t make_crc8_tabled(uint8_t header, uint8_t data[], uint8_t numDataBytes){
    //printf("Calculating CRC by table... Headerbyte: %02X, inPacketBuffer[0]: %02X, inPacketBufSize: %02X\n", header, data[0], numDataBytes);
    // CRC8-CCIT Lookup table [Poly = 0x07]
    static const uint8_t crc_lookup[256] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
        0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
        0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
        0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
        0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
        0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
        0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
        0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
        0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
        0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
        0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
        0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
        0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
        0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
        0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
        0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
        0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
        0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
        0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
        0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
        0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
        0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
        0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
        0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
        0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
        0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
        0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
        0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
    };
    // Create packet byte array
    uint8_t packet[numDataBytes+1];
    packet[0] = header;
    for(int i=0; i<numDataBytes; i++){
        packet[i+1] = data[i];
    }
    // Calculate CRC
    uint8_t crc = 0;
    for(int j=0; j<numDataBytes+1; ++j){
        crc = crc_lookup[crc ^ packet[j]];
    }
    //printf("CRC by table: %02X\n", crc);
    return crc;
}

// Nontabled CRC calculation 
uint8_t make_crc8_nontabled(uint8_t header, uint8_t data[], uint8_t numDataBytes){
    //printf("Calculating CRC by calc... Headerbyte: %02X, inPacketBuffer[0]: %02X, inPacketBufSize: %02X\n", header, data[0], numDataBytes);
    // Create packet byte array
    uint8_t packet[numDataBytes+1];
    packet[0] = header;
    //printf("packet: %02X", packet[0]);
    for(uint8_t i=0; i<numDataBytes; i++){
        packet[i+1] = data[i];
        //printf(" %02X\n", packet[i+1]);
    }
    //printf("\n");

    // Calculate CRC
    uint8_t crc = 0;

    uint8_t length = numDataBytes + 1;
    //printf("Calcs: %02X, length: %02X\n", crc, length);
    for(uint8_t i=0; i<length; ++i){
        crc ^= (packet[i] << (WIDTH - 8));
        for(uint8_t j=8; j>0; --j){
            if(crc & TOPBIT){
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
        }
        //printf("Calc[%d]: %02X\n", i, crc);
    }
    //printf("CRC by calc: %02X\n", crc);
    return crc;
}

// Benchmark function. Only runs on PC.
#if PC_BENCH
int main(){
    // Test data
    char hdr_test = 0x01;
    uint8_t data_size = 200;
    uint8_t data_test[data_size];
    for(int i=0;i<data_size;i++){
        data_test[i] = 0xAB;
    }
    // Init CRC and timer variables
    uint8_t crc = 0;
    struct timeval before, after;
    // Tabled CRC
    gettimeofday(&before, NULL);
    for(int i=0;i<10000;i++){
        crc = make_crc8_tabled(hdr_test, data_test, data_size);
    }
    gettimeofday(&after, NULL);
    double usec = (double)(after.tv_usec - before.tv_usec) / 1000000 + (double)(after.tv_sec - before.tv_sec);
    printf("Tabled CRC result:    %x, calculated in %f us.\n", crc, usec);
    // Nontabled CRC
    gettimeofday(&before, NULL);
    for(int i=0;i<10000;i++){
        crc = make_crc8_nontabled(hdr_test, data_test, data_size);
    }
    gettimeofday(&after, NULL);
    usec = (double)(after.tv_usec - before.tv_usec) / 1000000 + (double)(after.tv_sec - before.tv_sec);
    printf("Nontabled CRC result: %x, calculated in %f us.\n", crc, usec);
}
#endif