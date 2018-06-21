/*------------------------------------------------------------------
 *  in4073.h -- defines, globals, function prototypes
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#ifndef IN4073_H__
#define IN4073_H__

#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pc_terminal/protocol.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "app_util_platform.h"
#include "crc.h"

#define CONTROLLER 1
#define NON_ZERO_DEBUG	 1
#define BATTERY_CONNECTED 0
#define PACKET_DEBUG 0
#define LOG_DEBUG 0
#define MOTOR_VALUES_DEBUG 1

#define TIMER_PERIOD	4 //50ms=20Hz (MAX 23bit, 4.6h), 4ms = 250Hz
#define MAX_PACKET_SIZE 10
#define CALIBRATION_TIME_US 30000000
#define USB_TIMEOUT_MS (KEEP_ALIVE_TIMEOUT_MS + 500)
#define USB_TIMEOUT_CYCLES (USB_TIMEOUT_MS/TIMER_PERIOD)
#define BLINKLED_HZ 5
#define BLINKLED_CYCLES (500/(TIMER_PERIOD*BLINKLED_HZ))

#define RED			22
#define YELLOW		24
#define GREEN		28
#define BLUE		30
#define INT_PIN		5

uint8_t QuadState;
uint8_t ModeToSet;
uint8_t PreviousMode;
uint32_t CalibrationStartTime;
int8_t LRPY[4];
int16_t k_LRPY[8];
int16_t LRPY16[4];
int32_t filtered_sr;

bool received_data;
bool BlinkLed;
bool USBDisconnected;
bool demo_done;

// Control
int16_t motor[4],ae[4];
void run_filters();
void run_control();

// Timers
void timers_init(void);
uint32_t get_time_us(void);
bool check_timer_flag(void);
void clear_timer_flag(void);

// GPIO
void gpio_init(void);

// Queue
#define QUEUE_SIZE 256
typedef struct {
	uint8_t Data[QUEUE_SIZE];
	uint16_t first,last;
  	uint16_t count; 
} queue;
void init_queue(queue *q);
void enqueue(queue *q, char x);
char dequeue(queue *q);

// UART
#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14
queue rx_queue;
queue tx_queue;
uint32_t last_correct_checksum_time;
void uart_init(void);
void uart_put(uint8_t);

// TWI
#define TWI_SCL	4
#define TWI_SDA	2
void twi_init(void);
bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

// MPU wrapper
int16_t phi_o, theta_o, psi_o; // offsets
int16_t sp_o, sq_o, sr_o; // offsets
int16_t phi, theta, psi, phi_raw, theta_raw, psi_raw;
int16_t sp, sq, sr, sp_raw, sq_raw, sr_raw;
int16_t sax, say, saz;
uint8_t sensor_fifo_count;
void imu_init(bool dmp, uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_dmp_data(void);
void get_raw_sensor_data(void);
bool check_sensor_int_flag(void);
void clear_sensor_int_flag(void);

// Barometer
int32_t pressure;
int32_t temperature;
void read_baro(void);
void baro_init(void);

// ADC
uint16_t bat_volt;
bool low_battery;
void check_battery();
void adc_init(void);
void adc_request_sample(void);

// Flash
bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);
// New addition
bool flash_4k_sector_erase(uint8_t sector_number);

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
volatile bool radio_active;
void ble_init(void);
void ble_send(void);

// Logger
#define FLASH_ADDR_LIMIT   		0x01FFFF    // Maximum flash address
#define LOG_ENTRY_SIZE_BYTES  	27          // Amount of bytes in a log entry
#define LOG_PERIOD_US			10			// Amount of microseconds between logs
bool log_init();
void log_dump();
void logger_main();

#endif // IN4073_H__
