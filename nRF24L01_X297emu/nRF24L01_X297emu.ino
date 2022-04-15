#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"
#include "holystone710.h"

#define MOSI_pin  5  // MOSI - D3
#define SCK_pin   7  // SCK  - D4
#define CE_pin    2  // CE   - D5
#define MISO_pin  6 // MISO - D6
#define CS_pin    3 // CS   - D3

#define MOSI_on PORTC |= _BV(6)  // PC6
#define MOSI_off PORTC &= ~_BV(6)// PC6
#define SCK_on PORTE |= _BV(6)   // PE6
#define SCK_off PORTE &= ~_BV(6) // PE6
#define CE_on PORTD |= _BV(1)    // PD1
#define CE_off PORTD &= ~_BV(1)  // PD1
#define CS_on PORTD |= _BV(0)    // PD0
#define CS_off PORTD &= ~_BV(0)  // PD0

// SPI input
#define  MISO_on (PIND & _BV(7)) // PD7

#define RF_POWER TX_POWER_80mW 

      
uint8_t packet[128];
uint8_t counter=0;

//uint8_t payload_size=24;//+2;//transmitter packets are 24 bytes + 2 crc
uint8_t payload_size=16;//+2;//drone packets are 16 bytes + 2 crc

HS710_TransmitterState ts;

//float in -100 <> +100
uint8_t float_to_byte(float f,uint8_t min_val,uint8_t max_val) {
  return ( (f/100.0+1)/2 )*(max_val-min_val)+min_val;
}

uint8_t set_bit_state(uint8_t d, uint8_t n, bool state) {
  return state ? d&~_BV(n) : d|_BV(n);
}

void setup()
{
    Serial.begin(9600);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    NRF24L01_Reset();
    NRF24L01_Initialize();
    delay(10);
    XN297_SetTXAddr(HS710_tx_addr, 5);
    XN297_SetRXAddr(HS710_tx_addr, 5); //calls setup AW

    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment on all data pipes

    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payload_size); // PAYLOAD SIZE
    
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only 
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);  // 5 bytes address
    
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 56);     // set RF channel
    
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);

    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits
    NRF24L01_Activate(0x73);                         // Activate feature register

    delay(150);
    
    ts = {
      .lCtrl_v=0.0,
      .lCtrl_h=0.0,
      .rCtrl_v=0.0,
      .rCtrl_h=0.0,
      .channel_pulse=false,
      .gpshome_b=false,
      .picture_b=false,
      .highlow_b=false,
      .takeoff_b=false,
      .lock_b=false,
      .gpsen_b=false,
      .rf_channels = {56,62,70,64},
      .rf_channel = 0,
      .time_start = micros(),
      .time_channel_emit_last = 0,
      .time_channel_emit_freq = 7500,
      .time_channel_hop_length = 2, // in terms of emit freq

    };
}

void loop()
{
    /*if (counter++==0) {
      //Send unlock command to HS710
      XN297_SetTxRxMode(TX_EN);
      delayMicroseconds(5);
              NRF24L01_WriteReg(NRF24L01_07_STATUS,0x70);
              NRF24L01_FlushTx();
              NRF24L01_WriteReg(NRF24L01_05_RF_CH,56);
              XN297_WritePayload(unlock_packet, sizeof(unlock_packet)); //(bind packet)
      delay(1);
    }*/
    
    //RX things
    /*XN297_SetTxRxMode(RX_EN);
    packet[0]=0;
    uint32_t timeout = millis()+3;
    while(millis()<timeout) {
        if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from aircraft
            uint8_t crc_pass=XN297_ReadPayload(packet, payload_size+2);
            if (crc_pass) {
              for (int i=0; i<payload_size; i++) {
                Serial.print(packet[i],HEX);
              }
              Serial.println("");
              break;
            } else {
              //NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
              //NRF24L01_FlushRx();
            }
        }
    }*/

    //
    
      //Serial.println("PROCESS");
      HS710_process(&ts);
    //delay(100);
                

}
