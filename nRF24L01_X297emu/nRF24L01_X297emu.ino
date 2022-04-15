#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

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

uint8_t tx_addr[5] = {0x24,0x6a,0x6a,0x1a,0x82};
uint8_t rx_addr[5] = {0x24,0x6a,0x6a,0x1a,0x82}; //set command scrambles it

uint8_t unlock_packet[]= {0xCA,0xb6,0xde,0x24,
                  0x7d,0x7d,0x7d,0x7d,
                  0x20,0x20,0x20,0x78,
                  0xC0,
                  0x36,0x40,0x64,0x40,0x21,
                  0x0,0x0,0x0,
                  0x0,0x0,0x0};

                //WRITE TX 0xa0 (channel 70) TX-> 0x8d.b6.de.24.7d.7d.7d.7d.20.20.20.7f.a0.76.40.4.40.21.0.0.0.0.0.0
                // WRITE TX 0xa0 (channel 70) TX-> 0x8d.b6.de.24.7d.7d.7d.7d.20.20.20.7f.e0.36.40.4.40.21.0.0.0.0.0.0
                  
uint8_t packet[128];
uint8_t counter=0;

uint8_t payload_size=16; //24 is transmitter , 16 is drone

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
    XN297_SetTXAddr(tx_addr, 5);
    XN297_SetRXAddr(tx_addr, 5); //calls setup AW

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
}

void loop()
{
    if (counter++==0) {
      //Send unlock command to HS710
      XN297_SetTxRxMode(TX_EN);
      delayMicroseconds(5);
              NRF24L01_WriteReg(NRF24L01_07_STATUS,0x70);
              NRF24L01_FlushTx();
              NRF24L01_WriteReg(NRF24L01_05_RF_CH,56);
              XN297_WritePayload(unlock_packet, sizeof(unlock_packet)); //(bind packet)
      delay(1);
    }
    
    //RX things
    XN297_SetTxRxMode(RX_EN);
    packet[0]=0;
    uint32_t timeout = millis()+3;
    while(millis()<timeout) {
        if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from aircraft
            XN297_ReadPayload(packet, 24);
            if( packet[9] != 194)
            break;
        }
    }
    if (packet[0]!=0) {
      Serial.println(packet[0]);
    }
                

}
