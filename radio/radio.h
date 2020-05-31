#ifndef RADIO_H
#define RADIO_H



void radio_init(void);

void radio_tick(void);

uint8_t * radio_get_data(void);

void radio_clear_data(void);

bool radio_get_crc_status(void);

void radio_clear_crc_status(void);

uint8_t radio_get_rssi(void);

void radio_send_packet(uint8_t * data_ptr);

uint8_t * radio_get_packet(void);




uint32_t read_packet(void);

void send_packet_blocking(void);

#endif