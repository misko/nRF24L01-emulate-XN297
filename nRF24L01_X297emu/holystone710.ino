
#include "holystone710.h"

uint8_t HS710_tx_addr[5] = {0x24,0x6a,0x6a,0x1a,0x82};
uint8_t HS710_rx_addr[5] = {0x24,0x6a,0x6a,0x1a,0x82}; //set command scrambles it

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
  uint8_t checksum=txrx == TX_EN ? HS710_tx_checksum : HS710_rx_checksum;
  for (int i=1; i<len; i++) {
    checksum^=this_packet[i];
  }
  return checksum;
}

void HS710_packet_from_transmitter_state(HS710_TransmitterState * ts) {
  memcpy(packet,HS710_default_packet,sizeof(HS710_default_packet));

  //update joy stick positions
  packet[HS710_LCTRL_V]=float_to_byte(ts->lCtrl_v,0,0xfa);
  packet[HS710_LCTRL_H]=float_to_byte(ts->lCtrl_h,0,0xfa);
  packet[HS710_RCTRL_V]=float_to_byte(ts->rCtrl_v,0,0xfa);
  packet[HS710_RCTRL_H]=float_to_byte(ts->rCtrl_h,0,0xfa);

  packet[HS710_CHANNEL_PULSE_BYTE]=set_bit_state(packet[HS710_CHANNEL_PULSE_BYTE],
                                      HS710_CHANNEL_PULSE_BIT, ts->channel_pulse);
                                      
  packet[HS710_GPSHOME_B_BYTE]=set_bit_state(packet[HS710_GPSHOME_B_BYTE],
                                      HS710_GPSHOME_B_BIT, ts->gpshome_b);
                                      
  packet[HS710_PICTURE_B_BYTE]=set_bit_state(packet[HS710_PICTURE_B_BYTE],
                                      HS710_PICTURE_B_BIT, !ts->picture_b);
                                      
  packet[HS710_HIGHLOW_B_BYTE]=set_bit_state(packet[HS710_HIGHLOW_B_BYTE],
                                      HS710_HIGHLOW_B_BIT, !ts->highlow_b);
                                      
  packet[HS710_TAKEOFF_B_BYTE]=set_bit_state(packet[HS710_TAKEOFF_B_BYTE],
                                      HS710_TAKEOFF_B_BIT, ts->takeoff_b);
                                      
  packet[HS710_LOCK_B_BYTE]=set_bit_state(packet[HS710_LOCK_B_BYTE],
                                      HS710_LOCK_B_BIT, ts->lock_b);
                                      
  //lastly compute the checksum
  packet[0]=HS710_checksum(packet,HS710_transmitter_payload,TX_EN);
}

void HS710_process(HS710_TransmitterState * ts) {
  bool emit=false;
  
  const unsigned long current_time = micros();
  const uint8_t current_hop = (current_time-ts->time_start)/ts->time_channel_emit_freq;

  uint8_t new_rf_channel = ts->rf_channels[ (current_hop/ts->time_channel_hop_length)%sizeof(ts->rf_channels) ];

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
    XN297_WritePayload(packet, HS710_transmitter_payload); //(bind packet)
    //delay(1);
    
    /*Serial.print(micros());
    Serial.print(" ");
    Serial.print(ts->rf_channel);
    Serial.print(" ");
    for (int i=0; i<24; i++) {
      Serial.print(packet[i],HEX);
    }
    Serial.println("");*/
  }
}
