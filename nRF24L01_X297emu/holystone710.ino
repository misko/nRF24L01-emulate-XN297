
#include "holystone710.h"

//general transmission
uint8_t HS710_tx_addr[5] = {0x24,0x6a,0x6a,0x1a,0x82};
uint8_t HS710_rx_addr[5] = {0x24,0x6a,0x6a,0x1a,0x82}; //set command scrambles it

uint8_t HS710_drone_rx_packet[64]; //its 24 for debugging
//binding mode
uint8_t binding_HS710_tx_addr[5] = {0x6d,0x6a,0x78,0x52,0x43};
uint8_t binding_HS710_rx_addr[5] = {0x6d,0x6a,0x78,0x52,0x43}; 

uint8_t binding_magic_A[16]={
  0x6c,0x28,0xb3,0x3e,
  0x24,0x9e,0x73,0x9a,
  0x3d,0x94,0xa2,0x55,
  0x3d,0xa6,0x43,0x86
};

uint8_t binding_magic_B[16]={
  0x2d,0x95,0x9c,0xb4,
  0xa9,0x56,0x3e,0xb8,
  0x6a,0x94,0x3d,0x4b,
  0x69,0xc2,0x6b,0x23
};

uint8_t HS710_drone_bind_packet[]= {0xdc,0x00,0x0C,
                  0x0,0x0,0x0,
                  0x0,0x0,0x0,
                  0x0,0x0,0x0,
                  0x0,0xa0,0x0,0x0};
                  
uint8_t HS710_unlock_packet[]= {0xCA,0xb6,0xde,0x24,
                  0x7d,0x7d,0x7d,0x7d,
                  0x20,0x20,0x20,0x78,
                  0xC0,
                  0x36,0x40,0x64,0x40,0x21,
                  0x0,0x0,0x0,
                  0x0,0x0,0x0};
                  
uint8_t HS710_default_packet[]= {0x00,0xb6,0xde,0x24,
                  0x7d,0x7d,0x7d,0x7d,
                  0x20,0x20,0x20,0x7f,
                  0xa0,
                  0x76,0x40,0x04,0x40,0x21,
                  0x0,0x0,0x0,
                  0x0,0x0,0x0};

                //WRITE TX 0xa0 (channel 70) TX-> 0x8d.b6.de.24.7d.7d.7d.7d.20.20.20.7f.a0.76.40.4.40.21.0.0.0.0.0.0
                // WRITE TX 0xa0 (channel 70) TX-> 0x8d.b6.de.24.7d.7d.7d.7d.20.20.20.7f.e0.36.40.4.40.21.0.0.0.0.0.0


uint8_t HS710_checksum(uint8_t * this_packet, uint8_t len, TXRX_State txrx ) {
  if (txrx == TX_EN) {
    uint8_t checksum=HS710_tx_checksum;  
    for (int i=1; i<len; i++) {
      checksum^=this_packet[i];
    }
    return checksum;
  }
  
  uint8_t checksum=HS710_rx_checksum;
  for (int i=1; i<len; i++) {
    checksum+=this_packet[i];
  }
  return checksum&0xff;
  
}

void HS710_packet_from_transmitter_state(HS710_TransmitterState * ts) {
  memcpy(packet,HS710_default_packet,sizeof(HS710_default_packet));

  if (ts->bound) {
  //update joy stick positions
  packet[HS710_LCTRL_V]=float_to_byte(ts->lCtrl_v,0,0xfa);
  packet[HS710_LCTRL_H]=float_to_byte(ts->lCtrl_h,0,0xfa);
  packet[HS710_RCTRL_V]=float_to_byte(ts->rCtrl_v,0,0xfa);
  packet[HS710_RCTRL_H]=float_to_byte(ts->rCtrl_h,0,0xfa);

  packet[HS710_CHANNEL_PULSE_BYTE]=set_bit_state(packet[HS710_CHANNEL_PULSE_BYTE],
                                      HS710_CHANNEL_PULSE_BIT, ts->channel_pulse);
                                      
  packet[HS710_GPSHOME_B_BYTE]=set_bit_state(packet[HS710_GPSHOME_B_BYTE],
                                      HS710_GPSHOME_B_BIT, ts->gpshome_b);
                                      
  packet[HS710_GPSEN_B_BYTE]=set_bit_state(packet[HS710_GPSEN_B_BYTE],
                                      HS710_GPSEN_B_BIT, ts->gpsen_b);
                                      
  packet[HS710_PICTURE_B_BYTE]=set_bit_state(packet[HS710_PICTURE_B_BYTE],
                                      HS710_PICTURE_B_BIT, !ts->picture_b);
                                      
  packet[HS710_HIGHLOW_B_BYTE]=set_bit_state(packet[HS710_HIGHLOW_B_BYTE],
                                      HS710_HIGHLOW_B_BIT, !ts->highlow_b);
                                      
  packet[HS710_TAKEOFF_B_BYTE]=set_bit_state(packet[HS710_TAKEOFF_B_BYTE],
                                      HS710_TAKEOFF_B_BIT, ts->takeoff_b);
                                      
  packet[HS710_LOCK_B_BYTE]=set_bit_state(packet[HS710_LOCK_B_BYTE],
                                      HS710_LOCK_B_BIT, ts->lock_b);
  } else {
    //if we are not bound, send out a binding style packet
    packet[HS710_LCTRL_V]=0;
    packet[HS710_LCTRL_H]=0x7d;
    packet[HS710_RCTRL_V]=0x7d;
    packet[HS710_RCTRL_H]=0x7d;
    
    packet[HS710_CTRL1]=0x0;
    packet[HS710_CTRL2]=0x46;
    packet[HS710_FRONT_DIAL]=0x78;

    
    packet[HS710_CHANNEL_PULSE_BYTE]=set_bit_state(packet[HS710_CHANNEL_PULSE_BYTE],
                                      HS710_CHANNEL_PULSE_BIT, ts->channel_pulse);

    
  }
                                      
  //lastly compute the checksum
  packet[0]=HS710_checksum(packet,HS710_transmitter_payload,TX_EN);
}

void HS710_process(HS710_TransmitterState * ts) {
  bool emit=false;
  
  const unsigned long current_time = micros();
  const uint8_t current_hop = (current_time-ts->time_start)/ts->time_channel_emit_freq;

  uint8_t * channels = ts->bound ? ts->rf_channels : ts->binding_rf_channels;
  uint8_t new_rf_channel = channels[ (current_hop/ts->time_channel_hop_length)%sizeof(ts->rf_channels) ];

  //set channel pulse
  if (new_rf_channel!=ts->rf_channel) {
     //need to hop channel
     ts->channel_pulse=true;
     ts->rf_channel=new_rf_channel;
  } else {
    ts->channel_pulse=false;
  }
  
  if (ts->time_channel_emit_last!=current_hop) {
    ts->time_channel_emit_last=current_hop;
    emit=true;
  }


  if (emit) {
    HS710_packet_from_transmitter_state(ts);

    
    XN297_SetTxRxMode(TX_EN);
    delayMicroseconds(5);
    NRF24L01_WriteReg(NRF24L01_07_STATUS,0x70);
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH,ts->rf_channel);
    //NRF24L01_WriteReg(NRF24L01_05_RF_CH,65);
    XN297_WritePayload(packet, HS710_transmitter_payload); //(bind packet)
    delay(1);

    //DEBUG
    /*if (1==1) {
      Serial.print(micros());
      Serial.print(" ");
      Serial.print(ts->rf_channel);
      Serial.print(" ");
      for (int i=0; i<24; i++) {
        Serial.print(packet[i] < 16 ? "0" : "");
        Serial.print(packet[i],HEX);
      }
      Serial.println("");
      
    }*/
    XN297_SetTxRxMode(RX_EN);
  }


    uint8_t rx_waiting=XN297_RX_waiting();
    if (rx_waiting>0) {
      //listen for a reply?
      for (int i=0; i<24; i++) {
        HS710_drone_rx_packet[i]=0;
      }
      
      const uint8_t pass=XN297_ReadPayload(HS710_drone_rx_packet,16+2);//must be expected length + 2 (CRC)
  
      if (pass>0) {
        if (1==0) {
          Serial.print(ts->rf_channel);
          Serial.print(" RX ");
          Serial.print(pass>0 ? "PASS" : "FAIL");
          for (int i=0; i<16; i++) {
            Serial.print(HS710_drone_rx_packet[i] < 16 ? "0" : ""); 
            Serial.print(HS710_drone_rx_packet[i],HEX);
          }
          Serial.println("");
        }

    
        if (!ts->bound) {
              //figure out binding address
              const uint8_t binding_sum = HS710_drone_rx_packet[1]+HS710_drone_rx_packet[2];
              const uint8_t bindA = binding_magic_A[binding_sum&0xF];
              const uint8_t bindB = binding_magic_B[(binding_sum>>4)&0xF];
              
              const uint8_t HS710_bound_addr[5]={ 
                bindA,HS710_drone_rx_packet[1],bindB,HS710_drone_rx_packet[2],0x82
              };
              memcpy(HS710_tx_addr,HS710_bound_addr,5);
              memcpy(HS710_rx_addr,HS710_bound_addr,5);
              
              XN297_SetTXAddr(HS710_tx_addr, 5);
              XN297_SetRXAddr(HS710_rx_addr, 5); //calls setup AW
      
              ts->bound=true;
              for (int i=0; i<5; i++) {
                Serial.print(HS710_tx_addr[i] < 16 ? "0" : ""); 
                Serial.print(HS710_tx_addr[i],HEX);
              }
              Serial.println("");
        }
      }
      //
      
      NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
      NRF24L01_FlushRx();

     
    }
  
}
