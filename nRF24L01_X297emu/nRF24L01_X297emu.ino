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

//#define RF_POWER TX_POWER_5mW 
#define RF_POWER TX_POWER_20mW 

#define BIND_ON_BOOT false

      
uint8_t packet[128];
uint8_t counter=0;

//uint8_t payload_size=24;//+2;//transmitter packets are 24 bytes + 2 crc
uint8_t rx_payload_size=16+2;//+2;//drone packets are 16 bytes + 2 crc

HS710_TransmitterState ts;

//preprogrammed routine
unsigned long start_time; // = millis();
int drone_state=0;


//float in -100 <> +100
uint8_t float_to_byte(float f,uint8_t min_val,uint8_t max_val) {
  return ( (f/100.0+1)/2 )*(max_val-min_val)+min_val;
}

uint8_t set_bit_state(uint8_t d, uint8_t n, bool state) {
  return state ? d&~_BV(n) : d|_BV(n);
}

void setup()
{
    start_time=millis();
    Serial.begin(9600);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    NRF24L01_Reset();
    NRF24L01_Initialize();
    delay(10);

    if (BIND_ON_BOOT) {
      XN297_SetTXAddr(binding_HS710_tx_addr, 5);
      XN297_SetRXAddr(binding_HS710_rx_addr, 5); //calls setup AW
    } else {
      XN297_SetTXAddr(HS710_tx_addr, 5);
      XN297_SetRXAddr(HS710_rx_addr, 5); //calls setup AW
    }

    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment on all data pipes

    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, rx_payload_size); // PAYLOAD SIZE
    
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only 
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);  // 5 bytes address
    
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 62); //56);     // set RF channel
    
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);

    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits
    NRF24L01_Activate(0x73);                         // Activate feature register



    XN297_SetTxRxMode(RX_EN);
    
    delay(150);
    
    ts = {
      .bound=!BIND_ON_BOOT,
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
      .binding_rf_channels = {55,68,59,65},
      .rf_channel = 0,
      .time_start = micros(),
      .time_channel_emit_last = 0,
      .time_channel_emit_freq = 7500,
      .time_channel_hop_length = 2, // in terms of emit freq

    };
}

void loop()
{
    /*if (1==0 && counter++==0) {
      //Send unlock command to HS710
      XN297_SetTxRxMode(TX_EN);
      delayMicroseconds(5);
              NRF24L01_WriteReg(NRF24L01_07_STATUS,0x70);
              NRF24L01_FlushTx();

              //hacking binding
              NRF24L01_WriteReg(NRF24L01_05_RF_CH,55);

              memcpy(packet,HS710_drone_bind_packet,16);
              long r=random();
              packet[1]=r&0xFF;
              packet[2]=(r>>4)&0xFF;
              packet[0]=HS710_checksum(packet,16,RX_EN);
              for (int i=0; i<16; i++) {
                Serial.print(packet[i],HEX);
              }
              Serial.println("");
              XN297_WritePayload(packet, 16); 

              //lock rotors
              NRF24L01_WriteReg(NRF24L01_05_RF_CH,56);
              XN297_WritePayload(HS710_unlock_packet, 24);
      //delay(700);
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
    /*if (drone_state==0 && millis()-start_time>(10*1000)) {
      //unlock
      ts.gpsen_b=1;
      ts.lock_b=!ts.lock_b;
      drone_state=1;
    } else if (drone_state==1 && millis()-start_time>(12*1000)) {
      //take off
      ts.takeoff_b=!ts.takeoff_b;
      drone_state=2;
    } else if (drone_state==2 && millis()-start_time>(16*1000)) {
      //take off
      ts.takeoff_b=!ts.takeoff_b;
      drone_state=3;
    }*/

      HS710_process(&ts);
      ts.lCtrl_h=16;

      if (Serial.available()) {
        while (Serial.available()) {
          char r=Serial.read();
          if (r>10) {
            if (r=='l') {
              ts.lock_b=!ts.lock_b;
              Serial.print("LOCK:");
              Serial.println(ts.lock_b);
            }
            if (r=='u') {
              ts.takeoff_b=!ts.takeoff_b;
              Serial.print("TAKEOFF:");
              Serial.println(ts.takeoff_b);
            }
            if (r=='g') {
              ts.gpsen_b=!ts.gpsen_b;
              Serial.print("GPSEN:");
              Serial.println(ts.gpsen_b);
            }
            //Serial.println(r);
          }
        }
      }

}
