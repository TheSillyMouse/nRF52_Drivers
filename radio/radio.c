#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "nrf52.h"
#include "Utils/utils.h"
#include "nRF52_Drivers/rtc/rtc.h"
#include "nRF52_Drivers/uart/uart.h"
#include "packets/packets.h"
#include "radio.h"

#define PACKET_BASE_ADDRESS_LENGTH  (4UL)
#define PACKET_STATIC_LENGTH        (64UL)
#define PACKET_PAYLOAD_MAXSIZE      (PACKET_STATIC_LENGTH)
#define PACKET_S1_FIELD_SIZE        (0UL)
#define PACKET_S0_FIELD_SIZE        (0UL)
#define PACKET_LENGTH_FIELD_SIZE    (0UL)
#define BASE_ADDRESS_0              (0x01234567UL)
#define BASE_ADDRESS_1              (0x89ABCDEFUL)

#define TX_PACKET_SIZE    128
#define RX_PACKET_SIZE    128


typedef enum
{
    RADIO_STATE_DISABLED    = 0,
    RADIO_STATE_RXRU        = 1,
    RADIO_STATE_RXIDLE      = 2,
    RADIO_STATE_RX          = 3, 
    RADIO_STATE_RXDISABLE   = 4,
    RADIO_STATE_TXRU        = 9,
    RADIO_STATE_TXIDLE      = 10,
    RADIO_STATE_TX          = 11,
    RADIO_STATE_TXDISABLE   = 12,
}RadioState; 

typedef enum
{
    RADIO_BITRATE_NRF_1M    = RADIO_MODE_MODE_Nrf_1Mbit,
    RADIO_BITRATE_NRF_2M    = RADIO_MODE_MODE_Nrf_2Mbit,
    RADIO_BITRATE_NRF_250K  = RADIO_MODE_MODE_Nrf_250Kbit,
    RADIO_BITRATE_BLE_1M    = RADIO_MODE_MODE_Ble_1Mbit,
    RADIO_BITRATE_BLE_2M    = RADIO_MODE_MODE_Ble_2Mbit,
}RadioBitrate;

typedef enum
{
    RADIO_PACKET_LITTLE_ENDIAN = RADIO_PCNF1_ENDIAN_Little,
    RADIO_PACKET_BIG_ENDIAN = RADIO_PCNF1_ENDIAN_Big,
}RadioPacketEndianess; 

typedef enum
{
    RADIO_WHITENING_DISABLED    = RADIO_PCNF1_WHITEEN_Disabled,
    RADIO_WHITENING_ENABLED     = RADIO_PCNF1_WHITEEN_Enabled
}RadioWhitening;

typedef enum
{
    RADIO_CRC_LEN_ONE_BYTE  = RADIO_CRCCNF_LEN_One,
    RADIO_CRC_TWO_BYTES     = RADIO_CRCCNF_LEN_Two,
    RADIO_CRC_THREE_BYTES   = RADIO_CRCCNF_LEN_Three,
}RadioCRCLen;

typedef enum
{
    RADIO_TXPOWER_0dBm      = RADIO_TXPOWER_TXPOWER_0dBm,
    RADIO_TXPOWER_Pos3dBm   = RADIO_TXPOWER_TXPOWER_Pos3dBm,
    RADIO_TXPOWER_Pos4dBm   = RADIO_TXPOWER_TXPOWER_Pos4dBm,
    RADIO_TXPOWER_Neg40dBm  = RADIO_TXPOWER_TXPOWER_Neg40dBm,
    RADIO_TXPOWER_Neg20dBm  = RADIO_TXPOWER_TXPOWER_Neg20dBm,
    RADIO_TXPOWER_Neg16dBm  = RADIO_TXPOWER_TXPOWER_Neg16dBm,
    RADIO_TXPOWER_Neg12dBm  = RADIO_TXPOWER_TXPOWER_Neg12dBm,
    RADIO_TXPOWER_Neg8dBm   = RADIO_TXPOWER_TXPOWER_Neg8dBm,
    RADIO_TXPOWER_Neg4dBm   = RADIO_TXPOWER_TXPOWER_Neg4dBm,
    RADIO_TXPOWER_Neg30dBm  = RADIO_TXPOWER_TXPOWER_Neg30dBm,
}RadioTXPower; 

typedef enum
{
    RADIO_SHORT_READY_START         = RADIO_SHORTS_READY_START_Pos,
    RADIO_SHORT_END_DISABLE         = RADIO_SHORTS_END_DISABLE_Pos,
    RADIO_SHORT_DISABLED_TXEN       = RADIO_SHORTS_DISABLED_TXEN_Pos,
    RADIO_SHORT_DISABLED_RXEN       = RADIO_SHORTS_DISABLED_RXEN_Pos,
    RADIO_SHORT_ADDRESS_RSSISTART   = RADIO_SHORTS_ADDRESS_RSSISTART_Pos,
    RADIO_SHORT_END_START           = RADIO_SHORTS_END_START_Pos,
    RADIO_SHORT_ADDRESS_BCSTART     = RADIO_SHORTS_ADDRESS_BCSTART_Pos,
    RADIO_SHORT_DISABLED_RSSISTOP   = RADIO_SHORTS_DISABLED_RSSISTOP_Pos,
}RadioShorts;


struct packet_config
{
    uint8_t s0_field_size;  	/**< Packet S0 field size in bits. */
    uint8_t s1_field_size;  	/**< Packet S1 field size in bits. */
    uint8_t packet_len;		    /**< Packet length field size in bits. */
    bool    whitening;		    /**< 1 = Enabled, 0 = Disabled. */
    bool    endianess;		    /**< 1 = Big endian, 0 = Little endian. */
    uint8_t base_addr_len;	    /**< Packet base address length field size in bytes. */
    uint8_t static_len;		    /**< Packet static length in bytes. */
    uint8_t payload_max_size;	/**< Packet payload maximum size in bytes. */
};


static RadioState radio_get_state(void);

static void radio_enable(void);

static void radio_disable(void);

static void radio_set_frequency_channel(uint8_t channel_mhz);

static void radio_set_data_rate(uint32_t bitrate);

static void radio_set_base_addr(uint32_t base0, uint32_t base1);

static void radio_set_prefix_addr(uint8_t prefix, uint8_t address);

static void radio_set_short(RadioShorts option);

static void radio_set_packet_ptr(uint32_t * ptr);

//! TASKS functions
static void radio_set_task_rxen(void);
static void radio_set_task_stop(void);

//! EVENTS functions
static bool radio_get_events_ready(void);
static bool radio_get_events_disabled(void);
static bool radio_get_events_end(void);




static volatile uint8_t tx_buffer[TX_PACKET_SIZE];
static volatile uint8_t rx_buffer[RX_PACKET_SIZE];
static volatile uint8_t * rx_data_ptr = NULL;
static volatile uint8_t * tx_data_ptr = NULL;

static bool packet_crc_ok = false;
static uint32_t packet_tx_counter;




void radio_send_packet(uint8_t * data_ptr)
{
    strcpy(tx_buffer, data_ptr);
    tx_data_ptr = tx_buffer;
}

uint8_t * radio_get_packet(void)
{
    if (rx_data_ptr != NULL)
    {
        rx_data_ptr = NULL;
        return rx_buffer;
    }
    else
    {
        return NULL;
    }
    
}

void radio_tick(void)
{
    // Note
    // If there are no packets to be sent
    // turn on receiver and wait for packets

    RadioState state = radio_get_state();
    switch (state)
    {        
        case RADIO_STATE_DISABLED:
        {
            //uart_write_string("RADIO_STATE_DISABLED\r\n", 22);
            // Clear the flags.
            radio_clear_events_ready();
            radio_clear_events_disabled();
            radio_clear_events_end();

            if (tx_data_ptr != NULL)
            {
                radio_set_packet_ptr(tx_data_ptr);
                radio_set_task_txen();
                //art_write_string("Enable txen\r\n", 14);
                tx_data_ptr = NULL;
            }
            else
            {
                //uart_write_string("Enable rxen\r\n", 14);
                radio_set_packet_ptr(rx_buffer);
                // Enable the Receiver.
                radio_set_task_rxen();
            }
            break;
        }

        case RADIO_STATE_RXDISABLE:
        {            
            break;
        }

        case RADIO_STATE_RXRU:
        {
            // If READY_START are shorted
            // radio will jump to RX STATE and not
            // to RXIDLE.
            break;
        }

        case RADIO_STATE_RXIDLE:
        {
            //uart_write_string("RADIO_STATE_RXIDLE\r\n", 20);
            // If we are in RXIDLE STATE it means a packet
            // has been received in the past and actions need
            // to be done.

            // If there is a packet available to be sent we need
            // to go in TX mode and send it.
            if (tx_data_ptr != NULL)
            {
                radio_clear_events_end();
                radio_set_packet_ptr(tx_buffer);
                //uart_write_string("enable txen\r\n", 14);
                radio_set_task_txen();
                tx_data_ptr = NULL;
            }
            else
            {
                rx_data_ptr = NULL;
                if (NRF_RADIO->CRCSTATUS == 1U)
                {
                    rx_data_ptr = rx_buffer;
                }
                radio_clear_events_end();
                radio_set_packet_ptr(rx_buffer);
                radio_set_task_start();
            }       
            break;
        }

        case RADIO_STATE_RX:
        {
            // Receiving packet...
            if (tx_data_ptr != NULL)
            {
                radio_set_task_stop();    
                radio_set_packet_ptr(tx_buffer);            
                //uart_write_string("RX STOP\r\n", 10);
            }
            break;
        }

        case RADIO_STATE_TXRU:
        {
            // Radio is going to send the packet.
            //uart_write_string("RADIO_STATE_TXRU\r\n", 19);
            break;
        }

        case RADIO_STATE_TXIDLE:
        {
            //uart_write_string("RADIO_STATE_TXIDLE\r\n", 20);
            // If we are in TXIDLE STATE it means the radio
            // has transmitted a packet in the past.
            // We have to check if there's another packet to be sent
            // or to go back in receiver mode.
            if (tx_data_ptr != NULL)
            {
                radio_set_task_start();
            }
            else
            {
                radio_set_packet_ptr(rx_buffer);
                radio_set_task_rxen();
            }
            break;
        }

        case RADIO_STATE_TX:
        {
            // Radio is transmitting the packet.
            //uart_write_string("Packet is being sent\r\n", 23);
            break;
        }

        case RADIO_STATE_TXDISABLE:
        {
            break;
        }
    
        default:
        {
            return;
        }
            
    } 

    return;
}

uint8_t * radio_get_data(void)
{
    return rx_data_ptr;
}

void radio_clear_data(void)
{
    rx_data_ptr = NULL;
}

void radio_init(void)
{
    radio_enable();
    radio_set_tx_power(RADIO_TXPOWER_Pos4dBm);
    radio_set_frequency_channel(7); // 2400MHz + 7MHz
    radio_set_data_rate(RADIO_BITRATE_NRF_1M);

    struct packet_config packet_config;
    packet_config.base_addr_len     = PACKET_BASE_ADDRESS_LENGTH;
    packet_config.endianess         = RADIO_PACKET_BIG_ENDIAN;
    packet_config.packet_len        = PACKET_LENGTH_FIELD_SIZE;
    packet_config.payload_max_size  = PACKET_PAYLOAD_MAXSIZE;
    packet_config.static_len        = PACKET_STATIC_LENGTH;
    packet_config.s0_field_size     = PACKET_S0_FIELD_SIZE;
    packet_config.s1_field_size     = PACKET_S1_FIELD_SIZE;
    packet_config.whitening         = RADIO_WHITENING_DISABLED;

    radio_packet_config(&packet_config);

    radio_set_base_addr(BASE_ADDRESS_0, BASE_ADDRESS_1);
    radio_set_prefix_addr(0, 0xC0);
    radio_set_prefix_addr(1, 0xC1);
    radio_set_prefix_addr(2, 0xC2);
    radio_set_prefix_addr(3, 0xC3);
    radio_set_prefix_addr(4, 0xC4);
    radio_set_prefix_addr(5, 0xC5);
    radio_set_prefix_addr(6, 0xC6);
    radio_set_prefix_addr(7, 0xC7);

    radio_set_tx_address(0);
    radio_set_rx_address(1);

    radio_set_crc_len(RADIO_CRC_TWO_BYTES);

    radio_set_short(RADIO_SHORT_READY_START);
    radio_set_short(RADIO_SHORT_ADDRESS_RSSISTART);
    //radio_set_short(RADIO_SHORT_END_DISABLE);

    // struct packet packet;
    // packet.destination_address = 0x1234;
    // packet.sender_address = 0x4321;
    // packet.uid = 0x87;
    // packet.payload_length = sprintf(packet.payload, "Packet: %d", packet_tx_counter++);
    // packet_create(&packet, tx_buffer);
    // tx_data_ptr = tx_buffer;
}


bool radio_get_crc_status(void)
{
    return packet_crc_ok;
}

void radio_clear_crc_status(void)
{
    packet_crc_ok = false;
}


static RadioState radio_get_state(void)
{
    return NRF_RADIO->STATE;
}

void radio_set_tx_power(uint32_t level)
{
    NRF_RADIO->TXPOWER   = (level << RADIO_TXPOWER_TXPOWER_Pos);
}

static void radio_set_frequency_channel(uint8_t channel_mhz)
{
    NRF_RADIO->FREQUENCY = channel_mhz;
}

static void radio_set_data_rate(uint32_t bitrate)
{
    NRF_RADIO->MODE = (bitrate << RADIO_MODE_MODE_Pos);
}

void radio_packet_config(struct packet_config * config)
{   
    // Packet configuration
    NRF_RADIO->PCNF0 = (config->s1_field_size   << RADIO_PCNF0_S1LEN_Pos) |
                       (config->s0_field_size   << RADIO_PCNF0_S0LEN_Pos) |
                       (config->packet_len	<< RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // Packet configuration
    NRF_RADIO->PCNF1 = (config->whitening	    << RADIO_PCNF1_WHITEEN_Pos) |
                       (config->endianess	    << RADIO_PCNF1_ENDIAN_Pos)  |
                       (config->base_addr_len	    << RADIO_PCNF1_BALEN_Pos)   |
                       (config->static_len	    << RADIO_PCNF1_STATLEN_Pos) |
                       (config->payload_max_size    << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"
}

static void radio_set_base_addr(uint32_t base0, uint32_t base1)
{
    NRF_RADIO->BASE0 = bytewise_bitswap(base0);
    NRF_RADIO->BASE1 = bytewise_bitswap(base1);
}

static void radio_set_prefix_addr(uint8_t prefix, uint8_t address)
{
    switch (prefix)
    {
        case 0:
            NRF_RADIO->PREFIX0 |= ((uint32_t)swap_bits(address) << 0);
        case 1:
            NRF_RADIO->PREFIX0 |= ((uint32_t)swap_bits(address) << 8);
        case 2:
            NRF_RADIO->PREFIX0 |= ((uint32_t)swap_bits(address) << 16);
        case 3:
            NRF_RADIO->PREFIX0 |= ((uint32_t)swap_bits(address) << 24);
        case 4:
            NRF_RADIO->PREFIX1 |= ((uint32_t)swap_bits(address) << 0);
        case 5:
            NRF_RADIO->PREFIX1 |= ((uint32_t)swap_bits(address) << 8);
        case 6:
            NRF_RADIO->PREFIX1 |= ((uint32_t)swap_bits(address) << 16);
        case 7:
            NRF_RADIO->PREFIX1 |= ((uint32_t)swap_bits(address) << 24);
        default:
            return;
    }
    
}

static void radio_set_short(RadioShorts option)
{
    NRF_RADIO->SHORTS |= (1UL << option);
}

static void radio_clear_short(RadioShorts option)
{
    NRF_RADIO->SHORTS &= ~(1UL << option);
}

void radio_set_mode_rx(void)
{
}

static void radio_enable(void)
{
    NRF_RADIO->POWER = 1;
}

static void radio_disable(void)
{
    NRF_RADIO->POWER = 0;
}

void radio_set_tx_address(uint8_t logic_addr)
{
    NRF_RADIO->TXADDRESS = logic_addr;
}

void radio_set_rx_address(uint8_t logic_adr)
{
    NRF_RADIO->RXADDRESSES = logic_adr;
}

void radio_set_crc_len(RadioCRCLen crc_len)
{
    NRF_RADIO->CRCCNF = (crc_len << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
    }
    else
    {
    }
}

static void radio_set_packet_ptr(uint32_t * ptr)
{
    NRF_RADIO->PACKETPTR = ptr;
}

void radio_clear_events_ready(void)
{
    NRF_RADIO->EVENTS_READY = 0U;
}

void radio_clear_events_disabled(void)
{
    NRF_RADIO->EVENTS_DISABLED = 0U;
}

void radio_clear_events_end(void)
{
    NRF_RADIO->EVENTS_END  = 0U;
}

void radio_set_task_txen(void)
{
    NRF_RADIO->TASKS_TXEN   = 1;
}

static void radio_set_task_rxen(void)
{
    NRF_RADIO->TASKS_RXEN = 1U;
}

static void radio_set_task_stop(void)
{
    NRF_RADIO->TASKS_STOP = 1U;
}

void radio_set_task_start(void)
{
    NRF_RADIO->TASKS_START = 1U;
}

void radio_set_task_disable(void)
{
    NRF_RADIO->TASKS_DISABLE = 1U;
}

static bool radio_get_events_ready(void)
{
    return NRF_RADIO->EVENTS_READY;
}

static bool radio_get_events_end(void)
{
    return NRF_RADIO->EVENTS_END;
}

static bool radio_get_events_disabled(void)
{
    return NRF_RADIO->EVENTS_DISABLED;
}

uint8_t radio_get_rssi(void)
{
    return NRF_RADIO->RSSISAMPLE;
}



void RADIO_IRQHandler(void)
{
}








void send_packet_blocking(void)
{
    sprintf((uint8_t *)tx_buffer, "Hello world: %lu", packet_tx_counter++);

    NRF_RADIO->PACKETPTR = (uint32_t)&tx_buffer[0];

    // send the packet:
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_TXEN   = 1;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END  = 0U;
    NRF_RADIO->TASKS_START = 1U;

    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
}

uint32_t read_packet(void)
{
    uint32_t result = 0;
    uint32_t packet = 0;

    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;

    // Wait for end of packet or buttons state changed
    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    if (NRF_RADIO->CRCSTATUS == 1U)
    {
        result = packet;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
    return packet;
}




void radio_configure()
{
    // Radio config
    NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->FREQUENCY = 7UL;  // Frequency bin 7, 2407MHz
    NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

    // Radio address config
    NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0xC3) << 24) // Prefix byte of address 3 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC2) << 16) // Prefix byte of address 2 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC1) << 8)  // Prefix byte of address 1 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC0) << 0); // Prefix byte of address 0 converted to nRF24L series format

    NRF_RADIO->PREFIX1 =
        ((uint32_t)swap_bits(0xC7) << 24) // Prefix byte of address 7 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC6) << 16) // Prefix byte of address 6 converted to nRF24L series format
      | ((uint32_t)swap_bits(0xC4) << 0); // Prefix byte of address 4 converted to nRF24L series format

    NRF_RADIO->BASE0 = bytewise_bitswap(0x01234567UL);  // Base address for prefix 0 converted to nRF24L series format
    NRF_RADIO->BASE1 = bytewise_bitswap(0x89ABCDEFUL);  // Base address for prefix 1-7 converted to nRF24L series format

    NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = 0x01UL;  // Enable device address 0 to use to select which addresses to receive

    // Packet configuration
    NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |
                       (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |
                       (PACKET_LENGTH_FIELD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // Packet configuration
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                       (PACKET_BASE_ADDRESS_LENGTH   << RADIO_PCNF1_BALEN_Pos)   |
                       (PACKET_STATIC_LENGTH         << RADIO_PCNF1_STATLEN_Pos) |
                       (PACKET_PAYLOAD_MAXSIZE       << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
    }
    else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
    {
        NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
    }
}