#pragma once
#ifndef XBEE_H
#define XBEE_H

#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include "SafeVector.h" 


#define SERIAL_BUFFER_SIZE 128  // Originally 128, then 1028. Changed to 64 to keep tx_size within range
template <typename R24> int ID_Ind(char id, R24 *p_r24);

// SERIAL COM GENERAL
const uint16_t expectedSerialBufferSize = 1028;
const int tx_sizeMaxSend = SERIAL_BUFFER_SIZE;
const int rx_sizeMaxSend = SERIAL_BUFFER_SIZE;
int cnt_bytesRead = 0;
int cnt_bytesDiscarded = 0;
uint32_t cnt_timeoutRX = 0;
uint32_t cnt_overflowRX = 0;

// Define buffer sizes
const uint16_t buffMed = 50;
const uint16_t buffLrg = 250;
const uint16_t buffMax = buffLrg + 100;
const uint16_t buffTerm = 4;

// Define Queue size
const int SQ_Capacity = 10;
const int SQ_MsgBytes = 18;





// COM INSTANCE ID
namespace COM
{
  enum ID {
    r2c, // robot to C#
    c2r, // C# to robot
    r2a, // robot to CheetahDue
    a2r  // CheetahDue to robot
  };
  const char str_list_id[6][buffMed] =
  { { "r2c" },{ "c2r" },{ "r2a" },{ "a2r" } };
}

const char _cs_id_list[2] =
{
  'h', // setup handshake
  '\0'
};

const char _ard_id_list[9] =
{
  'h', // setup handshake
  'n', // ping test packets
  't', // hardware test
  'q', // quit/reset
  'r', // reward
  's', // sound cond [0, 1, 2]
  'p', // pid mode [0, 1]
  'b', // bull mode [0, 1]
  '\0'
};

const uint16_t _pack_range[2] = { 1, UINT16_MAX - 1 };

//UNION FOR SERIAL COMS
union UNION_SERIAL {
  byte b[4]; // (byte) 1 byte
  char c[4]; // (char) 1 byte
  uint16_t i16[2]; // (uint16_t) 2 byte
  uint32_t i32; // (uint32_t) 4 byte
  float f; // (float) 4 byte
};
UNION_SERIAL U;

// FEEDERDUE OUTGOING SERIAL
template <typename HW>
struct R2_COM
{
  HW &hwSerial;
  const COM::ID comID;
  const int lng;
  const char head;
  const char foot;
  VEC<char> id;
  VEC<uint16_t> packRange;
  VEC<uint16_t> packArr;
  VEC<uint16_t> packConfArr;
  uint16_t packInd;
  uint32_t packSentAll;
  uint32_t packRcvdAll;
  VEC<float> dat1;
  VEC<float> dat2;
  VEC<float> dat3;
  VEC<byte> flagArr;
  VEC<uint32_t> t_sentArr;
  VEC<uint32_t> t_queuedArr;
  VEC<bool> do_rcvCheckArr;
  VEC<int> cnt_repeatArr;
  uint32_t cnt_repeat;
  uint32_t t_sent; // (ms)
  int dt_sent; // (ms)
  int dt_minSentRcvd; // (ms) 
  int resendMax;
  int dt_resend; // (ms)
  byte SQ_Queue[SQ_Capacity][SQ_MsgBytes];
  int SQ_StoreInd;
  int SQ_ReadInd;
  R2_COM(HW &_hwSerial, const COM::ID _comID, const char _head, const char _foot, const char *_id, const uint16_t *_packRange, int _dt_minSentRcvd = 0, int _resendMax = 0, int _dt_resend = 0) :
    hwSerial(_hwSerial),
    comID(_comID),
    lng(strlen(_id)),
    head(_head),
    foot(_foot),
    id(lng, __LINE__, _id),
    packRange(2, __LINE__, _packRange),
    packArr(lng, __LINE__),
    packConfArr(lng, __LINE__),
    packInd(_packRange[0] - 1),
    packSentAll(0),
    packRcvdAll(0),
    dat1(lng, __LINE__),
    dat2(lng, __LINE__),
    dat3(lng, __LINE__),
    flagArr(lng, __LINE__),
    t_sentArr(lng, __LINE__),
    t_queuedArr(lng, __LINE__),
    do_rcvCheckArr(lng, __LINE__),
    cnt_repeatArr(lng, __LINE__),
    cnt_repeat(0),
    t_sent(0),
    dt_sent(0),
    dt_minSentRcvd(_dt_minSentRcvd),
    resendMax(_resendMax),
    dt_resend(_dt_resend),
    SQ_Queue(),
    SQ_StoreInd(0),
    SQ_ReadInd(0)
  {}
};

// Initialize Outgoing packet (r2a and r2c) structs
R2_COM<HardwareSerial> r2c(Serial1, COM::ID::r2c, '<', '>', _cs_id_list, _pack_range, 5, 5, 250);
R2_COM<HardwareSerial> r2a(Serial3, COM::ID::r2a, '{', '}', _ard_id_list, _pack_range, 5, 5, 100);

// FEEDERDUE INCOMING SERIAL
template <typename HW>
struct R4_COM
{
  HW &hwSerial;
  const COM::ID comID;
  const int lng;
  const char head;
  const char foot;
  VEC<char> id;
  VEC<uint16_t> packRange;
  VEC<uint16_t> packArr;
  VEC<uint16_t> packConfArr;
  uint16_t packInd;
  uint32_t packSentAll;
  uint32_t packRcvdAll;
  VEC<float> dat;
  uint32_t cnt_repeat;
  uint32_t cnt_dropped;
  char idNew;
  bool is_new;
  uint32_t t_rcvd; // (ms)
  int dt_rcvd; // (ms)
  int dt_minSentRcvd; // (ms) 
  R4_COM(HW &_hwSerial, const COM::ID _comID, const char _head, const char _foot, const char *_id, const uint16_t *_packRange, int _dt_minSentRcvd = 0) :
    hwSerial(_hwSerial),
    comID(_comID),
    lng(strlen(_id)),
    head(_head),
    foot(_foot),
    id(lng, __LINE__, _id),
    packRange(2, __LINE__, _packRange),
    packArr(lng, __LINE__),
    packConfArr(lng, __LINE__),
    packInd(_packRange[0] - 1),
    packSentAll(0),
    packRcvdAll(0),
    dat(3, __LINE__),
    cnt_repeat(0),
    cnt_dropped(0),
    idNew('\0'),
    is_new(false),
    t_rcvd(0),
    dt_rcvd(0),
    dt_minSentRcvd(_dt_minSentRcvd)
  {}
};

// Initialize Incoming packet (a2r and c2r) structs
R4_COM<HardwareSerial> c2r(Serial1, COM::ID::c2r, '<', '>', _cs_id_list, _pack_range, 5);
R4_COM<HardwareSerial> a2r(Serial3, COM::ID::a2r, '{', '}', _ard_id_list, _pack_range, 5);

// GET ID INDEX
template <typename R24> int ID_Ind(char id, R24 *p_r24)
{

  // Local vars
  static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

  // Return -1 if not found
  int ind = -1;
  for (int i = 0; i < p_r24->lng; i++)
  {

    if (id == p_r24->id[i])
    {
      ind = i;
    }
  }

  // Print warning if not found
  if (ind == -1)
  {
    Serial.println("INDEX NOT FOUND");
  }

  return ind;

}

// Define XBEE Functions
// QueuePacket, SendPacket, GetSerial, and WaitBuffRead are declared in Main.cpp due to dependencies
inline void QueuePacket(R2_COM<HardwareSerial> *p_r2, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf, bool is_conf, bool is_done, bool is_resend);
inline bool SendPacket(R2_COM<HardwareSerial> *p_r2);
inline bool GetSetByteBit(byte * b_set, int bit, bool do_set);
inline void GetSerial(R4_COM<HardwareSerial> *p_r4);
inline byte WaitBuffRead(R4_COM<HardwareSerial> *p_r4, char mtch='\0');


#endif

