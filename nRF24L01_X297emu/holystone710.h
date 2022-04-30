
#ifndef _HOLYSTONE710_H_
#define _HOLYSTONE710_H_
extern uint8_t HS710_tx_addr[5];
extern uint8_t HS710_rx_addr[5];
extern uint8_t HS710_drone_bind_packet[];
extern uint8_t HS710_unlock_packet[];


#define HS710_rx_checksum  0x6d
#define HS710_tx_checksum  0x6d

#define HS710_transmitter_payload 24
#define HS710_drone_payload 16

enum HS710_TrasmitterPacketBytes {
  HS710_CHECKSUM = 0,
  HS710_UNKN01 = 1,
  HS710_UNKN02 = 2,
  HS710_UNKN03 = 3,
  HS710_LCTRL_V = 4,
  HS710_LCTRL_H = 5,
  HS710_RCTRL_V = 6,
  HS710_RCTRL_H = 7,
  HS710_RCTRL_H_JERK = 8,
  HS710_RCTRL_V_JERK = 9,
  HS710_LCTRL_VH_JERK = 10,
  HS710_FRONT_DIAL = 11,
  HS710_CTRL1 = 12,
  HS710_CTRL2 = 13,
  HS710_CTRL3 = 14,
  HS710_UNKN04 = 15,
  HS710_UNKN05 = 16,
  HS710_UNKN06 = 17,
  HS710_UNKN07 = 18,
  HS710_UNKN08 = 19,
  HS710_UNKN09 = 20,
  HS710_UNKN10 = 21,
  HS710_UNKN11 = 22,
  HS710_UNKN12 = 23,
};

#define HS710_CHANNEL_PULSE_BYTE HS710_CTRL1
#define HS710_CHANNEL_PULSE_BIT 6

#define HS710_GPSHOME_B_BYTE HS710_CTRL1
#define HS710_GPSHOME_B_BIT 5

#define HS710_PICTURE_B_BYTE HS710_CTRL1
#define HS710_PICTURE_B_BIT 0

#define HS710_HIGHLOW_B_BYTE HS710_CTRL1
#define HS710_HIGHLOW_B_BIT 1

#define HS710_TAKEOFF_B_BYTE HS710_CTRL2
#define HS710_TAKEOFF_B_BIT 4

#define HS710_LOCK_B_BYTE HS710_CTRL2
#define HS710_LOCK_B_BIT 6

#define HS710_GPSEN_B_BYTE HS710_CTRL3
#define HS710_GPSEN_B_BIT 6


typedef struct HS710_TransmitterState{
  float lCtrl_v;
  float lCtrl_h;
  float rCtrl_v;
  float rCtrl_h;
  bool channel_pulse;
  bool gpshome_b;
  bool picture_b;
  bool highlow_b;
  bool takeoff_b;
  bool lock_b;
  bool gpsen_b;

  //Frequency hoping
  uint8_t rf_channels[4]; 
  uint8_t rf_channel; 
  
  unsigned long time_start;
  unsigned long time_channel_emit_last;
  unsigned long time_channel_emit_freq;
  unsigned long time_channel_hop_length;
  
} HS710_TransmitterState;

#endif
