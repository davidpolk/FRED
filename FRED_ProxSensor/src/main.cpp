//############################ Start of Main ############################



#include "Button.h"
#include "Pump.h"
#include "LCD.h"
#include "Logger.h"
#include "PID.h"
#include "XBEE.h"
#include "Reward.h"
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "L6474.h"
#include <SPI.h>
#include <math.h>


// Arduino Pins
const int proxSensor = 21;
const int proxSensor2 = 20;
const int LED = 53;


//#############  Variables  ##########

// Misc
int proxSensorVal;
int prox2SensorVal;
int motorSpeed;
String packetMessage;
byte sesMsg;
byte sesCond;
byte sesTask;
byte sesSound;
double sesSetpointHeadDist;
int pumpOpenScale;
int bullDel;
int bullSpeed;
int cnt_logBytesStored;
int rewDuration = 0; // (ms) 

// PID
const float setPointHead = 60;
const float feedDist = 72;
const float guardDist = 4.5;
float PIDsetPoint;

// Position Variables
String ratPosRadStr;
String robPosRadStr;;
double RatPos_cm;
double RobPos_cm;
double RatPos_rad;
double RobPos_rad;
float lastRatPosition = -1;
double feedTrackPastDist = 0; // (cm) 
const double feedHeadPastDist = 15; // (cm)
byte cnt_move;
float moveTarg;
double posDiff;
int packNum = 0;



// Flags
bool isRatStopped;
bool isHandshakeDone1 = false;
bool isHandshakeDone2 = false;
bool isTestPacketDone = false;
bool is_ManualSes;
bool is_ForageTask;
int cnt_rew = 0; 
bool is_RatOnTrack;
bool do_LogSend;
bool is_streamStarted = false;
bool isSetupComplete = false; 
bool isInitialMoveDone = false;
bool isFirstMove = false;
bool isMoveForwardPressed = false; 
bool isMoveBackwardPressed = false; 
bool isEtohPressed = false;
bool isFeedPressed = false;
bool isHalted;
bool isMoving = false;
bool isLEDon = true;
bool isMoveReady = false;
bool isRewardPending = false;
bool isRobotInFeedPosition = false;


// Time Variables
int ratStopTime;
int seconds;
int pumpTime;
float csTimeMs;
char ctimeStr[11]; // 00:00:00:00
float logTimerStartMs = 0;
String csTimeStr;
String timeStr;

//Object Initialization
L6474 myL6474; 

//Function Declarations
void MyFlagInterruptHandler(void);
void feed(int rewDuration);
void sysTest();
void extendFeederPlate(L6474 myL6474);
void retractFeederPlate(L6474 myL6474);
void stopRobot();
void stopRobotSlow();
void checkMove();


// create 4 states for switch debounce
typedef enum stateType_enum {
  wait_press,
  debounce_press,
  wait_release,
  debounce_release
} stateType;

// Initialize states.  Remember to use volatile
// set pushbutton state to initial position of wait_press
// make state variable volatile type since it can be changed in the ISR
//



volatile stateType pbstate = wait_press;

void setup() {
  Serial.begin(9600); 
  Serial1.begin(57600); 
  r2c.hwSerial.begin(57600);   //Begin Serial Communication (XBee)
  SPI.begin();
  sei(); // Enable global interrupts.
  Serial.println("Setup...");

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // HIGH = 1. Proximity sensor should always be 1 until it senses someone within the 10 cm range then it should be 0
  int proxSensorState = HIGH;
  int prox2SensorState = HIGH;
  //Setup the peripheral devices
   
  myL6474.Begin(1, 0);
  myL6474.AttachFlagInterrupt(MyFlagInterruptHandler);
  ButtonSetup();
  LCDSetup();
  PumpSetup();
  SDCardSetup();
  // MotorsSetup(); 
  // L6470_setup();
  
  myL6474.SetMaxSpeed(0,4000);
  myL6474.SetMinSpeed(0,1000);
  myL6474.SetAcceleration(0, 1000);
  myL6474.SetDeceleration(0, 3300);
  myL6474.SelectStepMode(0, L6474_STEP_SEL_1_4, 1);

  myL6474.SetMaxSpeed(1,3000);
  myL6474.SetMinSpeed(1,2000);
  myL6474.SetAcceleration(1, 1000);
  myL6474.SetDeceleration(1, 1000);
   
  // myL6474.SetMaxSpeed(1,5000);
  // myL6474.SetMinSpeed(1,4000);
  // myL6474.SetAcceleration(1,10);
  // myL6474.SetDeceleration(1,10);
  myL6474.SetHome(0, 0);
  myL6474.SetHome(0, 1);
  // myL6474.SetHome(1, 0);

  //sysTest(); 

  Serial.println("Setup Complete!");
  writeLCD("SysTest Complete!", 0);

  attachInterrupt(digitalPinToInterrupt(proxSensor), checkMove, CHANGE);
  attachInterrupt(digitalPinToInterrupt(proxSensor2), checkMove, CHANGE);

  attachInterrupt(digitalPinToInterrupt(19), stopRobot, RISING);

}

void loop() {
  // Check Proiximity Sensor Values
  // This should be 1 if rat not within 10 cm. Helpful with debugging
  


  if (proxSensorVal == 0 || prox2SensorVal == 0 ){
    myL6474.HardStop(0, 1);
  }

 /// **** DELETE; SD Card Module not used **** ///
  writeToSDCard(timeStr, "Test Message", "test.txt");
  delay(10);


  // Get Data from C# and process command
  GetSerial(&c2r);

  //checkMove();

  if (isRewardPending == true && isRobotInFeedPosition == true){
    myL6474.HardStop(0, 1);
    ProcRewCmd(0, 0, 0);
    isRewardPending = false;
  }
  
  // Get Data from CheetahDue
  // GetSerial(&a2r);

  // Send Data to C#
  //SendPacket(&r2c);

  // Send Data to CheetahDue
  //SendPacket(&r2a);

  
  
//    INITIAL HANDSHAKE    //
  // Check if timeSync handshake has sent to the C#
  // if not send it to the C#
  // if (isHandshakeDone1 == false) {
  //   QueuePacket(&r2c, 'h', 1, 0, 0, 0, true, false, false, false);
  //   delay(1);
  //   SendPacket(&r2c);
    
  //   // Log the current millisecond time since program start,
  //   // used for time sync and logging.
  //   logTimerStartMs = millis();
  // }
  // Check if confirmation handshake has been sent to C#, 
  // if not send it to the C#
  // if (isHandshakeDone2 == false) {
  //   QueuePacket(&r2c, 'h', 2, 0, 0, 0, true, false, false, false);
  //   delay(1);
  //   SendPacket(&r2c);
  // }
  // Check if hardware test packet has been sent to C#,
  // if not send it to the C#
  // if (isTestPacketDone == false && isHandshakeDone2) {
  //   QueuePacket(&r2c, 'T', 0, 0, 0, 0, true, true, true, false);
  //   delay(1);
  //   SendPacket(&r2c);
  //   isTestPacketDone = true;
  // }


  // if (isTestPacketDone == true) {
  //   if ((millis() % 5000) <= 100){
  //     if (isSetupComplete == true){
  //     // QueuePacket(&r2c, 'K', 0, 0, 0, 0, false, true, false, false);
  //     delay(1);
  //     //SendPacket(&r2c);
  //     // isSetupComplete = false;
  //     }
  //   } 
  // }

/////         BUTTON HANDLING           ////

  // Pump Ethanol using button on robot
  if (!(PINC & (1 << PINC5))) {
    Serial.println("B7 pressed");
    clearLCD();
    writeLCD("Priming ethanol motor", 0);
    Serial.println("Button Pressed");
    digitalWrite(5, HIGH);
    clearLCD();
    isFeedPressed = true;
  }
  else {  
    if (isFeedPressed == true){
      digitalWrite(5, LOW);
       Serial.println("B7  NOT pressed");
      isFeedPressed = false;
    }
  }



//  Pump Food using button on robot
  if (!(PINC & (1 << PINC3))) {
    clearLCD();
    writeLCD("Priming Feeding Pump", 0);
    digitalWrite(4, HIGH);
    clearLCD();
    isEtohPressed = true;
  }

else {
  if (isEtohPressed == true){
    digitalWrite(4, LOW);
    isEtohPressed = false;
  }
}

//   Do Feeding Routine using button on robot
  if (!(PINC & (1 << PINC1))) {
    clearLCD();
    writeLCD("Feeding Routine", 0);
    feed(2000);
    clearLCD();
  }

  

// Move Forward using button on robot
if (!(PINC & (1 << PINC4)) ){
  Serial.println("Button Pressed");

    Serial.println("Move Forward");
    clearLCD();
    writeLCD("Moving Forward...", 0);
    myL6474.Run(0, FORWARD, 1);
    // delay(2000);
    clearLCD();
    isMoveForwardPressed = true;
  }
while (!(PINC & (1 << PINC4)) ){

   //Run this asynchronously
}
if (isMoveForwardPressed == true){
  clearLCD();
  myL6474.SoftStop(0);
  isMoveForwardPressed = false; 
  myL6474.HardStop(0, 1);
}


// Move Backward using button on robot
if (!(PINC & (1 << PINC2)) ){
  Serial.println("Button Pressed");

    Serial.println("Move Backward");
    clearLCD();
    writeLCD("Moving Backward...", 0);
    myL6474.Run(0, BACKWARD, 1);
    // delay(2000);
    clearLCD();
    isMoveBackwardPressed = true;
  }
while (!(PINC & (1 << PINC2)) ){

   //Run this asynchronously
}
if (isMoveBackwardPressed == true){
  clearLCD();
  myL6474.SoftStop(0);
  isMoveBackwardPressed = false; 
  myL6474.HardStop(0, 1);
}
  
// Turn LED On/Off using button on robot
if (!(PINC & (1 << PINC0)) ){
Serial.println("Button Pressed");

  if (isLEDon == true){
    Serial.println("Turning LED Off");
    clearLCD();
    writeLCD("Turning LED Off...", 0);
    digitalWrite(LED, LOW);
    clearLCD();
    isLEDon = false;
  }
  else{
    Serial.println("Turning LED Off");
    clearLCD();
    writeLCD("Turning LED Off...", 0);
    digitalWrite(LED, HIGH);
    clearLCD();
    isLEDon = true;
  }
  
 
}

// BUTTON DEBOUNCE  (Smoothes out button presses) //  NEEDED?
  // Case statements for different button states
  switch (pbstate) {
  
    // button not pressed yet
    case wait_press:
    break;
  
    // button just preseed
    case debounce_press:
    delay(2);
    pbstate = wait_release;
    break;
  
    // button unreleased yet
    case wait_release:

    break;
  
    // button justt released
    case debounce_release:
    Serial.println("Feeding...");
    clearLCD();
    writeLCD("Feeding...", 0);
    // feederPlate();
    // delay(10);
    // QueuePacket(&r2c, 'h', 1, 0, 0, 0, true, false, false, false);
    // delay(10);
    // pumpFood(400);
    delay(2);
    Serial.println("End");
    clearLCD();
    delay(2);
    timeStr = getCurrentTime(logTimerStartMs);
    writeToSDCard(timeStr, "Test Message", "testFile3.txt");
    pbstate = wait_press;
    break;
  }
}



// // Implement an Pin Change Interrupt which handles the switch being pressed and released.

ISR(PCINT0_vect) {

  // if the interrupt was triggered when state was waiting for press then we are
  // going debounce the press action  so set pbstate to debounce press

  if (pbstate == wait_press) {
    pbstate = debounce_press;
  } else if (pbstate == wait_release) {
    Serial.println("Release");
    pbstate = debounce_release;
  }
}



// Test all robot components individually; Used at startup  
void sysTest(){
    // Test LCD
    writeLCD("System Test", 0);

    //Test Feeding Pump
    writeLCD("Testing Feeding Pump", 1);
    pumpFood(1000);

    //Test Ethanol Pump
    delay(2000);
    clearLine(1);
    writeLCD("Testing Ethanol Pump", 1);
    pumpEtoh();

    
    // Test Movement Motor
    clearLine(1);
    writeLCD("Testing Movement", 1);
    myL6474.Run(0, FORWARD, 1);
    delay(2000);
    myL6474.SoftStop(0);
    delay(2000);
    Serial.println("Motor 2 Setup");
    myL6474.Run(0, BACKWARD, 1);
    delay(2000);
    myL6474.HardStop(0, 1);
    delay(1000);

    // Test Feeder Plate Motor
    delay(2000);
    clearLine(1);
    writeLCD("Testing Feeder Plate", 1);
    extendFeederPlate(myL6474);
    delay(2000);
    retractFeederPlate(myL6474);
    delay(2000);

    clearLCD();
}


//  Feeding routine  //
/* 
    1.Extends Feeder plate 
    2. Pumps food for time given by rewDuration
    3. Wait for rat to eat (20 Seconds)
    4. Retracts feeder plate 
*/
void feed(int rewDuration){
    cnt_rew++;
    Serial.println(rewDuration);
    delay(10);
    extendFeederPlate(myL6474);
    delay(200);
    pumpFood(rewDuration);
    //QueuePacket(&r2a, 'r', 400, cnt_rew, 0, 0, false, false, false, false);
    delay(1);
    //SendPacket(&r2a);
    // for(int i=0; i<2; i++){
    //   QueuePacket(&r2c, 'R', 1, 2, 3, 100, 1, 1, 1, 1);
    //   delay(5);
    //   SendPacket(&r2c);
    //   delay(5);
    // }
    delay(7000); // update to 20 seconds delay
    retractFeederPlate(myL6474);
    for (int i=0; i<5; i++){
      QueuePacket(&r2c, 'R', 9, 9, 9, packNum+1, false, true, true, true);
      delay(5);
      SendPacket(&r2c);
    }
}



void extendFeederPlate(L6474 myL6474){
  //myL6474.Run(0, FORWARD, 1);
  myL6474.SetMaxSpeed(0,2000);
  myL6474.SetMinSpeed(0,1000);
  // myL6474.SetAcceleration(0,1000);
  // myL6474.SetDeceleration(0,1000);
  myL6474.GoTo(0, -1700, 0);
  delay(750); //220ms delay = ~90 degree motor turn
  myL6474.HardStop(0, 0);
  myL6474.SetMaxSpeed(0,5000);
  myL6474.SetMinSpeed(0,4000);
  myL6474.SetAcceleration(0,1000);
  myL6474.SetDeceleration(0,1000);
}

void retractFeederPlate(L6474 myL6474){
  myL6474.SetMaxSpeed(0,2000);
  myL6474.SetMinSpeed(0,1000);
  // myL6474.SetAcceleration(0,1000);
  // myL6474.SetDeceleration(0,1000);
  myL6474.GoTo(0, 1500, 0);
  delay(750); //220ms delay = ~90 degree motor turn
  myL6474.HardStop(0, 0);
  myL6474.SetMaxSpeed(0,5000);
  myL6474.SetMinSpeed(0,4000);
  // myL6474.SetAcceleration(0,1000);
  // myL6474.SetDeceleration(0,1000);
}


///////////////////////////////// Serial Communication ////////////////////////////////////////////////////

// STORE PACKET DATA TO BE SENT
void QueuePacket(R2_COM<HardwareSerial> *p_r2, char id, float dat1, float dat2,
                 float dat3, uint16_t pack, bool do_conf, bool is_conf,
                 bool is_done = false, bool is_resend = false) {

  /*
  STORE DATA TO SEND
  FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer
  */

  //   Serial.println("QUEUE PACKET");
  
  // Local vars
  static char buff_lrg[buffLrg] = {0};
  buff_lrg[0] = '\0';
  float _dat[3] = {dat1, dat2, dat3};
  VEC<float> dat(3, __LINE__, _dat);
  byte flag_byte = 0;
  int id_ind = 0;
  uint16_t tx_size = 0;
  uint16_t rx_size = 0;
  R4_COM<HardwareSerial> *p_r4;

  // Set pointer to R4_COM struct
  if (p_r2->comID == COM::ID::r2c) {
    p_r4 = &c2r;
  } else if (p_r2->comID == COM::ID::r2a) {
    p_r4 = &a2r;
  }

  // Set flag_byte
  GetSetByteBit(&flag_byte, 0, do_conf);
  GetSetByteBit(&flag_byte, 1, is_conf);
  GetSetByteBit(&flag_byte, 2, is_done);
  GetSetByteBit(&flag_byte, 3, is_resend);

  // Get buffers
  tx_size = SERIAL_BUFFER_SIZE - 1 - p_r2->hwSerial.availableForWrite();
  rx_size = p_r2->hwSerial.available();

  // Update sendQueue ind
  p_r2->SQ_StoreInd++;

  // Check if ind should roll over
  if (p_r2->SQ_StoreInd == SQ_Capacity) {
    p_r2->SQ_StoreInd = 0;
  }

  // Check if overfloweed
  if (p_r2->SQ_Queue[p_r2->SQ_StoreInd][0] != '\0') {

    // Get list of empty entries
    for (int i = 0; i < SQ_Capacity; i++) {
      buff_lrg[i] = p_r2->SQ_Queue[i][0] == '\0' ? '0' : '1';
    }
    buff_lrg[SQ_Capacity] = '\0';

    // Set queue back
    p_r2->SQ_StoreInd =
        p_r2->SQ_StoreInd - 1 >= 0 ? p_r2->SQ_StoreInd - 1 : SQ_Capacity - 1;

    return;
  }

  // Incriment packet ind if originating from robot
  if (pack == 0) {

    // Incriment packet
    p_r2->packInd++;

    // Reset packet if out of range
    if (p_r2->packInd > p_r2->packRange[1]) {

      // Set to lowest range value
      p_r2->packInd = p_r2->packRange[0];
    }

    // Copy packet number
    pack = p_r2->packInd;
  }

  // Create byte packet
  int b_ind = 0;
  // Store header
  p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = p_r2->head;
  // Store mesage id
  p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = id;
  // Store mesage data

  for (int i = 0; i < 3; i++) {
    U.f = dat[i];
    p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[0];
    p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[1];
    p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[2];
    p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[3];
  }

  // Store packet number
  U.f = 0.0f;
  U.i16[0] = pack;
  p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[0];
  p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[1];
  // Store flag_byte flag
  p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = flag_byte;
  // Store footer
  p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = p_r2->foot;

  // Store time
  // id_ind = ID_Ind<R2_COM<HardwareSerial>>(id, p_r2);
  p_r2->t_queuedArr[id_ind] = millis();
}

// SEND SERIAL PACKET DATA
bool SendPacket(R2_COM<HardwareSerial> *p_r2) {

  /*
  STORE DATA TO SEND
  FORMAT IN:  [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer,
  [9]targ FORMAT OUT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte,
  [8]footer
  */

  // Local vars
  static char buff_lrg[buffLrg] = {0};
  buff_lrg[0] = '\0';
  static char buff_lrg_2[buffLrg] = {0};
  buff_lrg_2[0] = '\0';
  static char buff_lrg_3[buffLrg] = {0};
  buff_lrg_3[0] = '\0';
  static char buff_lrg_4[buffLrg] = {0};
  buff_lrg_4[0] = '\0';
  int dt_rcvd = 0;
  int dt_queue = 0;
  char id = '\0';
  VEC<float> dat(3, __LINE__);
  bool do_conf = false;
  bool is_conf = false;
  bool is_resend = false;
  bool is_repeat = false;
  byte flag_byte = 0;
  uint16_t pack = 0;
  uint16_t pack_last = 0;
  uint16_t tx_size = 0;
  uint16_t rx_size = 0;
  int id_ind = 0;
  R4_COM<HardwareSerial> *p_r4;
  R4_COM<HardwareSerial> *p_r4o;
  R2_COM<HardwareSerial> *p_r2o;

  Serial.println("Send Packet");

  // Set pointer to R4_COM struct
  if (p_r2->comID == COM::ID::r2c) {
    p_r4 = &c2r;
    p_r2o = &r2a;
    p_r4o = &a2r;
  } else if (p_r2->comID == COM::ID::r2a) {
    p_r4 = &a2r;
    p_r2o = &r2c;
    p_r4o = &c2r;
  }

  // Bail if nothing in queue
  if (p_r2->SQ_ReadInd == p_r2->SQ_StoreInd &&
      p_r2->SQ_Queue[p_r2->SQ_StoreInd][0] == '\0') {
      Serial.println("Bail if nothing in queue");
    return false;
  }

  // Get buffer
  tx_size = SERIAL_BUFFER_SIZE - 1 - p_r2->hwSerial.availableForWrite();
  rx_size = p_r2->hwSerial.available();

  Serial.print("tx_size: ");
  Serial.println(tx_size);
  Serial.print("rx_size: ");
  Serial.println(rx_size);

  // Bail if buffer or time inadequate
  if (tx_size > tx_sizeMaxSend || rx_size > rx_sizeMaxSend ||
      millis() < p_r2->t_sent + p_r2->dt_minSentRcvd) {
        Serial.println("Bail if buffer or time inadequate");
    return true;
  }

  // Add small delay if just recieved
  else if (millis() < p_r4->t_rcvd + p_r4->dt_minSentRcvd) {
    delayMicroseconds(500);
  }

  // Serial.println("SENDING PACKET");

  // Incriment send ind
  p_r2->SQ_ReadInd++;

  // Check if ind should roll over
  if (p_r2->SQ_ReadInd == SQ_Capacity) {
    p_r2->SQ_ReadInd = 0;
  }

  // Send
  p_r2->hwSerial.write(p_r2->SQ_Queue[p_r2->SQ_ReadInd], SQ_MsgBytes);
  // Serial.println("Data sent");

  // Get buffers
  tx_size = SERIAL_BUFFER_SIZE - 1 - p_r2->hwSerial.availableForWrite();
  rx_size = p_r2->hwSerial.available();

  // pull out packet data
  int b_ind = 1;
  // id
  id = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
  // dat
  for (int i = 0; i < 3; i++) {
    U.f = 0;
    U.b[0] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
    U.b[1] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
    U.b[2] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
    U.b[3] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
    dat[i] = U.f;
  }
  // pack
  U.f = 0.0f;
  U.b[0] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
  U.b[1] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
  pack = U.i16[0];
  // conf flag
  flag_byte = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
  do_conf = GetSetByteBit(&flag_byte, 0, false);
  is_conf = GetSetByteBit(&flag_byte, 1, false);
  is_resend = GetSetByteBit(&flag_byte, 3, false);

  // Set entry to null
  p_r2->SQ_Queue[p_r2->SQ_ReadInd][0] = '\0';

  // Get id ind
  // id_ind = ID_Ind<R2_COM<HardwareSerial>>(id, p_r2);

  // Set flags for recieve confirmation
  if (do_conf) {
    p_r2->do_rcvCheckArr[id_ind] = true;
  }

  // Get last pack
  pack_last = is_conf ? p_r2->packConfArr[id_ind] : p_r2->packArr[id_ind];

  // Flag resent pack
  is_repeat = pack == pack_last;

  // Incriment repeat count
  p_r2->cnt_repeat += is_repeat || is_resend ? 1 : 0;

  // Incriment total sent packet count
  p_r2->packSentAll++;

  // Update packet history
  if (!is_conf)
    p_r2->packArr[id_ind] = pack;
  else
    p_r2->packConfArr[id_ind] = pack;

  // Update dt stuff
  p_r2->dt_sent = p_r2->t_sent > 0 ? millis() - p_r2->t_sent : 0;
  p_r2->t_sent = millis();
  dt_rcvd = p_r4->t_rcvd > 0 ? millis() - p_r4->t_rcvd : 0;
  dt_queue = millis() - p_r2->t_queuedArr[id_ind];

  // Update other struct info
  p_r2->dat1[id_ind] = dat[0];
  p_r2->dat2[id_ind] = dat[1];
  p_r2->dat3[id_ind] = dat[2];
  p_r2->flagArr[id_ind] = flag_byte;
  p_r2->t_sentArr[id_ind] = p_r2->t_sent;

   Serial.println("Done sending data");

  return true;
}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(R4_COM<HardwareSerial> *p_r4, char mtch = '\0') {

  // Local vars
  static char buff_lrg[buffLrg] = {0};
  buff_lrg[0] = '\0';
  static char buff_lrg_2[buffLrg] = {0};
  buff_lrg_2[0] = '\0';
  static int timeout = 100;
  uint32_t t_timeout = millis() + timeout;
  bool is_overflowed = false;
  byte b = 0;

  // Get total data in buffers now
  uint16_t rx_size_start = p_r4->hwSerial.available();
  // Serial.print("rx size");
  // Serial.println(rx_size_start);

  // Check for overflow
  is_overflowed = rx_size_start >= SERIAL_BUFFER_SIZE - 1;

  // Wait for at least 1 byte
  while (p_r4->hwSerial.available() < 1 && millis() < t_timeout)
    ;

  // Get any byte
  if (!is_overflowed && mtch == '\0') {

    if (p_r4->hwSerial.available() > 0) {

      b = p_r4->hwSerial.read();
      cnt_bytesRead++;

      return b;
    }
  }

  // Find specific byte
  while (b != mtch && millis() < t_timeout && !is_overflowed) {

    // Check new data
    if (p_r4->hwSerial.available() > 0) {

      b = p_r4->hwSerial.read();
      cnt_bytesRead++;

      // check match was found
      if (b == mtch) {

        // Return byte
        return b;
      }

      // Otherwise add to discard count
      else {
        cnt_bytesDiscarded++;
      }

      // Check for overflow
      is_overflowed = !is_overflowed
                          ? p_r4->hwSerial.available() >= SERIAL_BUFFER_SIZE - 1
                          : is_overflowed;
    }
  }

  // Check if buffer flooded
  if (is_overflowed) {

    // DUMP IT ALL
    while (p_r4->hwSerial.available() > 0) {
      if (p_r4->hwSerial.available() > 0) {
        p_r4->hwSerial.read();
        cnt_bytesRead++;
      }
    }
  }

  // Get buffer
  uint16_t tx_size =
      SERIAL_BUFFER_SIZE - 1 - p_r4->hwSerial.availableForWrite();
  uint16_t rx_size = p_r4->hwSerial.available();

  // // Store current info
  // Debug.sprintf_safe(buffLrg, buff_lrg_2, " from=%s buff_lrg=\'%s\' b_read=%d
  // b_dump=%d rx_start=%d rx_now=%d tx_now=%d dt_chk=%d",
  // 	COM::str_list_id[p_r4->comID], Debug.FormatSpecialChars(b),
  // cnt_bytesRead, cnt_bytesDiscarded, rx_size_start, rx_size, tx_size,
  // (millis() - t_timeout) + timeout);

  // Buffer flooded
  if (is_overflowed) {

    // Incriment count
    cnt_overflowRX++;

    // Print first 5 messages
    if (cnt_overflowRX < 1000) {
      Serial.println("OVERFLOW!!");
      // Debug.sprintf_safe(buffLrg, buff_lrg, "BUFFER OVERFLOWED: cnt=%d",
      // cnt_overflowRX);
    }

    // TEMP Run error hold imediately
    // Debug.RunErrorHold("RUNNING ERROR HOLD", "BUFF OVERFLOW");

  }

  // Timed out
  else if (millis() > t_timeout) {

    // Incriment count
    cnt_timeoutRX++;

    // Print only every 10th message after first 10
    if (cnt_timeoutRX < 10 || cnt_timeoutRX % 10 == 0) {

      // Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT: cnt=%d",
      // cnt_timeoutRX);
    }
    // Return zero
    else {
      Serial.println("TIMEOUT!!");
      return 0;
    }
  }

  // Byte not found
  else if (mtch != '\0') {
    Serial.println("Char not found");
    // Debug.sprintf_safe(buffLrg, buff_lrg, "CHAR \'%c\' NOT FOUND:",
    // mtch);
  }

  // Failed for unknown reason
  else {
    Serial.println("FAILED FOR UNKNOWN REASON:");
    // Debug.sprintf_safe(buffLrg, buff_lrg, "FAILED FOR UNKNOWN REASON:");
  }

  // Compbine strings
  // Debug.strcat_safe(buffLrg, strlen(buff_lrg), buff_lrg, strlen(buff_lrg_2),
  // buff_lrg_2);

  // Log error
  // Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

  // Return 0
  // DB_FUN_END(m_ind);
  return 0;
}

void GetSerial(R4_COM<HardwareSerial> *p_r4) {
  //Serial.println("Get serial");

  /*
  PARSE DATA FROM CS
  FORMAT: [0]head, [1]id, [2:5]dat[0], [6:9]dat[1], [10:13]dat[1], [14:15]pack,
  [16]flag_byte, [17]footer, [18]targ
  */

  // Local vars
  static char buff_lrg[buffLrg] = {0};
  buff_lrg[0] = '\0';
  static char buff_lrg_2[buffLrg] = {0};
  buff_lrg_2[0] = '\0';
  static char buff_lrg_3[buffLrg] = {0};
  buff_lrg_3[0] = '\0';
  static char buff_lrg_4[buffLrg] = {0};
  buff_lrg_4[0] = '\0';
  uint32_t t_start = millis();
  int dt_parse = 0;
  int dt_sent = 0;
  uint16_t tx_size = 0;
  uint16_t rx_size = 0;
  byte b = 0;
  char head = ' ';
  char id = ' ';
  VEC<float> dat(3, __LINE__);
  int r4_ind = 0;
  int r2_ind = 0;
  uint16_t pack = 0;
  char foot = ' ';
  bool do_conf = false;
  bool is_conf = false;
  bool is_resend = false;
  bool do_resend = false;
  bool is_repeat = false;
  bool is_missed = false;
  bool is_dropped = false;
  int n_missed = 0;
  byte flag_byte = 0;
  R2_COM<HardwareSerial> *p_r2;
  R2_COM<HardwareSerial> *p_r2o;
  R4_COM<HardwareSerial> *p_r4o;

  // Set pointer to R2_COM struct
  if (p_r4->comID == COM::ID::c2r) {
    p_r2 = &r2c;
    p_r2o = &r2a;
    p_r4o = &a2r;
    
  } else if (p_r4->comID == COM::ID::a2r) {
    p_r2 = &r2a;
    p_r2o = &r2c;
    p_r4o = &c2r;

  }

  // Reset vars
  cnt_bytesRead = 0;
  cnt_bytesDiscarded = 0;
  p_r4->is_new = false;
  p_r4->idNew = ' ';

  // Bail if no new input
  if (p_r4->hwSerial.available() == 0) {
    Serial.println("HW Serial not Available");
    return;
  }
  // else {
  //   Serial.println("Data is Available");
  // }

  // Serial.println("Buffer redings: ");
  // for (int i = 0; i < 3; i++) {
  //   Serial.println(WaitBuffRead(p_r4));
  // }

  // Dump data till p_msg header byte is reached
  b = WaitBuffRead(p_r4, p_r4->head);
  if (b == 0) {
    return;
  }

  // Store header
  head = b;
  // Serial.print("Header: ");
  // Serial.println(head);
  // Serial.print(head);

  // Get id
  id = WaitBuffRead(p_r4);
  // Serial.print(id);

  // Parse data
  for (int i = 0; i < 3; i++) {
    U.f = 0.0f;
    U.b[0] = WaitBuffRead(p_r4);
    U.b[1] = WaitBuffRead(p_r4);
    U.b[2] = WaitBuffRead(p_r4);
    U.b[3] = WaitBuffRead(p_r4);
    dat[i] = U.f;

  //   Serial.println("Raw Data");
  //   Serial.println(U.b[0]);
  //   Serial.println(U.b[1]);
  //   Serial.println(U.b[2]);
  //  Serial.println(U.b[3]);
  //  Serial.println(dat[i]);
  }

  // Get packet num
  U.f = 0.0f;
  U.b[0] = WaitBuffRead(p_r4);
  U.b[1] = WaitBuffRead(p_r4);
  pack = U.i16[0];

  // Serial.println("Incoming Data");
  // Serial.println(id);
  // Serial.println(c2r.dat[0]);
  // Serial.println(c2r.dat[1]);
  // Serial.println(c2r.dat[2]);
    Serial.println("ID:  " + id);
  switch (id) {
  case ('h'):
    Serial.println("Case h");
    // Serial.println(c2r.dat[0]);
    // Serial.println(c2r.dat[1]);
    // Serial.println(c2r.dat[2]);

    if (c2r.dat[0] == 1) {
      isHandshakeDone1 = true;
      QueuePacket(&r2c, 'h', 1, 0, 0, 0, false, true, true, true);  
      delay(1);
      SendPacket(&r2c);
    }

    if (c2r.dat[0] == 2) {
      isHandshakeDone2 = true;
      QueuePacket(&r2c, 'h', 2, 0, 0, 0, false, true, true, true);
      delay(1);
      LCDBacklightOff();
    }

    break;

  case ('T'):
    Serial.println("Case T");
    QueuePacket(&r2c, 'T', 0, 0, 0, 0, true, true, true, false);
    delay(1);
    SendPacket(&r2c);
    break;

  case ('K'):
    Serial.println("Case K");
    break;

  case ('I'):
    Serial.println("Case I");
    is_RatOnTrack = c2r.dat[0] == 1 ? true : false;
    isInitialMoveDone = true;
    break;
  
  case ('B'):
    Serial.println("Case B");
    bullDel = c2r.dat[0];
		bullSpeed = c2r.dat[1];
    QueuePacket(&r2a, 'b', 1, 0, 0, 0, false, false);
    QueuePacket(&r2a, 'b', 0, 0, 0, 0, false, false);
    isSetupComplete = true;
    break;
    
  case ('H'):
    myL6474.HardStop(0, 1);
    QueuePacket(&r2c, 'H', c2r.dat[0], c2r.dat[1], c2r.dat[2], packNum, false, true, true, true);
    delay(5);
    SendPacket(&r2c);
    Serial.println("Case H");
    
    if (dat[1] == 1){
      isHalted = true;
      
    }
    else if (dat[1] == 2){
      isHalted = false;
    }
    //myL6474.HardStop(0, 1);
    isMoving = false;
    break;
  case ('M'):
    QueuePacket(&r2c, 'M', c2r.dat[0], c2r.dat[1], c2r.dat[2], packNum, false, true, true, true);
     delay(5);
     SendPacket(&r2c);
    Serial.println("Case M");
    // delay(1000);
    Serial.println(c2r.dat[0]);
    Serial.println(c2r.dat[1]);
    Serial.println(c2r.dat[2]);
    cnt_move = (byte)c2r.dat[0];
		moveTarg = c2r.dat[1];

  if ((proxSensorVal != 0) && pack != packNum){
    Serial.println("Move Packet");
     
    if (isHalted == false){
      Serial.println("Move Forward");
      clearLCD();
      writeLCD("Moving Forward...", 0);
      myL6474.Run(0, FORWARD, 1);
      delay(2000);
      isMoving = true;
    }

  }
  

    // if (cnt_move == 0 || cnt_move == 1){
    //   QueuePacket(&r2c, 'M', 0, 0, 0, c2r.packArr[ID_Ind<R4_COM<HardwareSerial>>('M', &c2r)], true, false, true);
    //   delay(1);
    //   SendPacket(&r2c);
    // }

    // delay(10);
   
    // else{
    //     myL6474.HardStop(0,0);
    // }

    // isInitialMoveDone = true;
    if (cnt_move == 1){
      isFirstMove = true;
    }
    else if (cnt_move == 0){
      isFirstMove = false;
    }
    
    break;

  case ('P'):
    Serial.println("Case P");
    // Serial.println(c2r.dat[0]);
    // Serial.println(c2r.dat[1]);
    // Serial.println(c2r.dat[2]);

    // if (isInitialMoveDone == true){
      if (c2r.dat[0] == 0) {
        RatPos_cm = c2r.dat[1];
        RatPos_rad = RatPos_cm / ((140 * PI) / (2 * PI));
      }
      else {
        RobPos_cm = c2r.dat[1];
        RobPos_rad = RobPos_cm / ((140 * PI) / (2 * PI));
      }
      if (!is_streamStarted){ 
        is_streamStarted = true;
        QueuePacket(&r2c, 'K', 1, 0, 0, 0, true, false);
        delay(1);
        SendPacket(&r2c);
      }
      else{
        posDiff = (abs(RatPos_cm - RobPos_cm));
        if(posDiff > 55){
          

          if (myL6474.GetShieldState(0) == INACTIVE && isHalted == false){
            myL6474.Run(0, FORWARD, 1);
            pumpEtoh();

          }
          
        }
        else{
          myL6474.HardStop(0, 1);
        }
      }

      // if (isInitialMoveDone == true){

      //   motorSpeed = GetPID(moveTarg, RobPos_rad);
      //   myL6474.Run(0, FORWARD, 0);
      //   delay(motorSpeed);
      //   myL6474.HardStop(0, 0);

      // }
      
      
    // }

    break;

  case ('U'):
  // Send number of log bytes being sent
			QueuePacket(&r2c, 'U', cnt_logBytesStored, 0, 0, 0, true, false);
      delay(1);
      SendPacket(&r2c);

  break;

  case ('L'):
    Serial.println("Case L");
    do_LogSend = c2r.dat[0] == 1 ? true : false;

    if (!do_LogSend)
		{

			// Log

			// Send number of log bytes being sent
			QueuePacket(&r2c, 'U', cnt_logBytesStored, 0, 0, 0, true, false);
      delay(1);
      SendPacket(&r2c);

			
		}
    break;

  case ('Q'):
    Serial.println("Case Q");
    QueuePacket(&r2a, 'q', 0, 0, 0, 0, true, false);
    delay(1);
    SendPacket(&r2a);
    break;

case ('S'):
    Serial.println("Case S");
    QueuePacket(&r2c, 'S', 0, 0, 0, 0, true, true);
    delay(1);
    SendPacket(&r2c);
    // Store message data
		sesMsg = (byte)c2r.dat[0];

		// Store info from first 'S' packet
		if (sesMsg == 1)
		{
			sesCond = (byte)c2r.dat[1];
			sesTask = (byte)c2r.dat[2];
		}

		// Store info from second 'S' packet
		if (sesMsg == 2)
		{
			sesSound = (byte)c2r.dat[1];
			sesSetpointHeadDist = c2r.dat[2];
		}

		// Handle first 'S' packet
		if (sesMsg == 1)
		{

			// Reset flags
			is_ManualSes = false;
			is_ForageTask = false;

			// Handle Manual session
			if (sesCond == 1)
			{
				// Turn on LCD light
				LCDoff();
				// Set flag
				is_ManualSes = true;
			}

			// Handle Behavior session
			if (sesCond == 2){}

			// Handle Implant session
			if (sesCond == 3){}

			// Handle Track task
			if (sesTask == 1)
			{
				// Set rew led min
				// rewLEDduty[0] = 0;

				// Change reward solonoid on scale
				pumpOpenScale = 1;
			}

			// Handle Forage task
			if (sesTask == 2)
			{

				// Set rew led forage min
				// rewLEDduty[0] = 2;

				// Set to min
				// analogWrite(pin.LED_REW_C, rewLEDduty[0]);
				// analogWrite(pin.LED_REW_R, rewLEDduty[0]);

				// Change reward solonoid on scale
				pumpOpenScale = 0.5;

				// Set flag
				is_ForageTask = true;
			}

		}

		// Handle second 'S' packet
		if (sesMsg == 2)
		{

			// Handle no sound condition
			if (sesSound == 0)
			{

				// No sound
				QueuePacket(&r2a, 's', 0, 0, 0, 0, false, false);
        delay(1);
        SendPacket(&r2a);
			}

			// Handle white noise only condition
			if (sesSound == 1)
			{

				// Use white noise only
				QueuePacket(&r2a, 's', 1, 0, 0, 0, false, false);
        delay(1);
        SendPacket(&r2a);
			}

			// Handle white noise and reward tone condition
			if (sesSound == 2)
			{
				// Use white and reward noise
				QueuePacket(&r2a, 's', 2, 0, 0, 0, false, false, false);
        delay(1);
        SendPacket(&r2a);
			}

			// Compute and store pid setpoint
			PIDsetPoint = setPointHead + sesSetpointHeadDist;

			// Update tracker feeder pass distance
			// feedTrackPastDist = feedDist + feedHeadPastDist + sesSetpointHeadDist;

			// Log set setpoint

			// Log feed pass distance
		

		}
    break;

  case ('R'):
    QueuePacket(&r2c, 'R', c2r.dat[0], c2r.dat[1], c2r.dat[2], packNum, false, true, true, true);
    delay(5);
    SendPacket(&r2c);
    Serial.print("dat[0]: ");
    Serial.println(dat[0]);
    Serial.print("dat[1]: ");
    Serial.println(dat[1]);
    Serial.print("dat[2]: ");
    Serial.println(dat[2]);

    isRewardPending = true;

    checkMove();
    // pumpFood(400);
   // myL6474.HardStop(0, 1);
   
    
    // delay(100);
    // for(int i; i<100; i++){
      
    //   delay(100);
    // }


    //ProcRewCmd(dat[0], dat[1], dat[2]);
  
    // RunReward();
    // feed(rewDuration);
    // Serial.print(rewDuration);
    // delay(10);
    // QueuePacket(&r2c, 'O', 0, 0, 0, 0, false, true);
    // SendPacket(&r2c);
    
    // Serial.println(c2r.dat[0]);
    // Serial.println(c2r.dat[1]);
    // Serial.println(c2r.dat[2]);
    
    break;
  }

  
  packNum = pack;

  Serial.print("pack Num");
  Serial.print(pack);

  // Get confirmation flag
  U.f = 0.0f;
  U.b[0] = WaitBuffRead(p_r4);
  do_conf = U.i16[0];

  U.f = 0.0f;
  U.b[0] = WaitBuffRead(p_r4);
  is_conf = U.i16[0];

  // U.b[0] = WaitBuffRead(p_r4);
  // do_resend = U.b[0];

  // U.b[0] = WaitBuffRead(p_r4);
  // is_resend = U.b[0];

  // flag_byte = U.b[0];
  // do_conf = GetSetByteBit(&flag_byte, 0, false);
  // is_conf = GetSetByteBit(&flag_byte, 1, false);
  // do_resend = GetSetByteBit(&flag_byte, 2, false);
  // is_resend = GetSetByteBit(&flag_byte, 3, false);

  // Serial.print("Do Conf ");
  // Serial.println(do_conf);
  // Serial.print("is_conf ");
  // Serial.println(is_conf);
  // Serial.print("do_resend ");
  // Serial.println(do_resend);
  // Serial.print("is_resend ");
  // Serial.println(is_resend);

  // Get footer
  foot = WaitBuffRead(p_r4);

  // Serial.println(foot);

  // Get total data in buffers
  rx_size = p_r4->hwSerial.available();
  tx_size = SERIAL_BUFFER_SIZE - 1 - p_r4->hwSerial.availableForWrite();

  // Compute parce and sent dt
  dt_parse = millis() - t_start;
  dt_sent = p_r2->t_sent > 0 ? millis() - p_r2->t_sent : 0;

  // Check for dropped packet
  if (foot != p_r4->foot) {

    // Incriment dropped count
    p_r4->cnt_dropped++;

    // Set flag
    is_dropped = true;
  }

  // Footer found so process packet
  else {

    // Send confirmation
    if (do_conf) {
      Serial.println("Do Confirm");
      QueuePacket(p_r2, id, dat[0], dat[1], dat[2], pack, false, true);
      delay(5);
      SendPacket(p_r2);
    }
    else{
      Serial.println("Dont confirm");
    }

    // Get id ind
    r4_ind = ID_Ind<R4_COM<HardwareSerial>>(id, p_r4);
    r2_ind = ID_Ind<R2_COM<HardwareSerial>>(id, p_r2);

    // // Set coms started flag
    // if (p_r4->comID == COM::ID::c2r && !is_ComsStarted)
    // {
    // 	is_ComsStarted = true;
    // }

    // Reset check
    p_r2->do_rcvCheckArr[r2_ind] = false;
    p_r2->cnt_repeatArr[r2_ind] = 0;

    // Update times
    p_r4->dt_rcvd = p_r4->t_rcvd > 0 ? millis() - p_r4->t_rcvd : 0;
    p_r4->t_rcvd = millis();

    // Get last pack
    uint16_t pack_last;
    if (!is_conf)
      pack_last = p_r4->packArr[r4_ind];
    else
      pack_last = p_r4->packConfArr[r4_ind];

    // Flag repeat pack
    is_repeat = pack == pack_last;

    // Incriment repeat count
    p_r4->cnt_repeat += is_repeat || is_resend ? 1 : 0;

    // Update packet history
    if (!is_conf)
      p_r4->packArr[r4_ind] = pack;
    else
      p_r4->packConfArr[r4_ind] = pack;

    // Update data
    p_r4->dat[0] = dat[0];
    p_r4->dat[1] = dat[1];
    p_r4->dat[2] = dat[2];

    // Update for new packets
    if (!is_conf && !is_repeat) {

      // Get pack diff accounting for packet rollover
      int pack_diff =
          abs(pack - p_r4->packInd) < (p_r4->packRange[1] - p_r4->packRange[0])
              ? pack - p_r4->packInd
              : pack - (p_r4->packRange[0] - 1);

      // Update dropped packets
      n_missed = pack_diff - 1;
      p_r4->cnt_dropped += n_missed > 0 ? n_missed : 0;

      // flag missed packets
      if (n_missed > 0) {
        is_missed = true;
      }

      // Update packets sent
      p_r4->packSentAll += pack_diff;

      // Update packet ind
      p_r4->packInd = pack;

      // Incriment packets recieved
      p_r4->packRcvdAll++;

      // Update new message info
      p_r4->idNew = id;
      p_r4->is_new = true;
    }
  }
}


//////////////////////////////////// Reward Handling ///////////////////////////////////////////////

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>






// VARIABLES
	int durationDefault = 400; // (ms) 400 
	const int _zoneRewDurs[9] = { // (ms)
		100,
		182,
		284,
		368,
		400,
		368,
		284,
		182,
		100
	};
	const int _zoneLocs[9] = { // (deg)
		20,
		15,
		10,
		5,
		0,
		-5,
		-10,
		-15,
		-20,
	};


	const VEC<int> zoneRewDurs;
	const VEC<int> zoneLocs;

	static const int zoneLng =
		sizeof(_zoneLocs) / sizeof(_zoneLocs[0]);

	VEC<double> zoneBoundCumMin;
	VEC<double> zoneBoundCumMax;
	VEC<int> zoneOccTim;
	VEC<int> zoneOccCnt;
	VEC<double> zoneBoundCumRewarded;
	int cnt_cmd = 0;
	uint32_t t_nowZoneCheck = 0;
	uint32_t t_lastZoneCheck = 0;
	int rewDelay = 0; // (ms)
	float solOpenScale = 1;
	int zoneMin = 0;
	int zoneMax = 0;
	uint32_t t_rewStr = 0;
	uint32_t t_rewEnd = 0;
	uint32_t t_closeSol = 0;
	uint32_t t_retractArm = 0;
	uint32_t t_moveArmStr = 0;
	double goalPosCum = 0;
	bool isRewarding = false;
	bool isZoneTriggered = false;
	bool isAllZonePassed = false;
	bool is_ekfNew = false;
	int zoneInd = 0;
	int zoneRewarded = 0;
	int occRewarded = 0;
	int lapN = 0;
	uint32_t armMoveTimeout = 5000;
	bool do_ArmMove = false;
	bool do_ExtendArm = false;
	bool do_RetractArm = false;
	bool do_TimedRetract = false;
	bool isArmExtended = false;
    bool v_doStepTimer;
    byte v_stepTarg; 
    bool v_isArmMoveDone;
    char v_stepDir;
	const int dt_step_high = 500; // (us)
	const int dt_step_low = 500; // (us)
	bool isArmStpOn = false;
    const int dt_rewBlock = 30000; // (ms)
    const int rewZoneWidth = 5; // (deg)

	enum REWMODE {
		BUTTON,
		NOW,
		CUE,
		FREE,
	};
	const char *p_str_list_rewMode[4] =
	{  "BUTTON" , "NOW" , "CUE" , "FREE"  };
	REWMODE rewMode = BUTTON;
	enum MICROSTEP {
		FULL,
		HALF,
		QUARTER,
		EIGHTH,
		SIXTEENTH
	};
	const char *p_str_microstep[5] =
	{  "FULL" , "HALF" , "QUARTER" , "EIGHTH" , "SIXTEENTH"  };
	const MICROSTEP ezExtMicroStep = QUARTER;
	const MICROSTEP ezRetMicroStep = QUARTER;

	
void checkMove(){
  //  if (isLEDon == true){
  //   Serial.println("Turning LED Off");
  //   clearLCD();
  //   writeLCD("Turning LED Off...", 0);
  //   digitalWrite(LED, LOW);
  //   clearLCD();
  //   isLEDon = false;
  // }
  // else{
  //   Serial.println("Turning LED Off");
  //   clearLCD();
  //   writeLCD("Turning LED Off...", 0);
  //   digitalWrite(LED, HIGH);
  //   clearLCD();
  //   isLEDon = true;
  // }

  proxSensorVal = digitalRead(proxSensor);
  prox2SensorVal = digitalRead(proxSensor2);

  // if (proxSensorVal == 0) {
  //   digitalWrite(LED, HIGH);
  // }
  // else{
  //   digitalWrite(LED, LOW);
  // }

  if (proxSensorVal == 0){
    isRobotInFeedPosition = true;
  }
  else {
    isRobotInFeedPosition = false;
  }

  // If both sensor are activated, stop robot
  if (proxSensorVal == 0 && prox2SensorVal == 0){
    myL6474.HardStop(0, 1);
    isMoveReady = false;
  }
  // If only front sensor is activated, get ready to move forward
  else if (proxSensorVal == 0 && prox2SensorVal == 1){
    isMoveReady = true;
  }
  else if (proxSensorVal == 1 && prox2SensorVal == 0){
    isMoveReady = false;
  }
  // If both sensor are inactive, determine whether rat is in front or behind sensors
  else if (proxSensorVal == 1 && prox2SensorVal == 1){
    // If rat is in front of robot, move forward
    if (isMoveReady == true){
      myL6474.Run(0, FORWARD, 1);
      delay(1000);
    }
  }
}

void stopRobot(){
  myL6474.HardStop(0, 1);
}

void stopRobotSlow(){
  myL6474.SoftStop(0);
}

void ProcRewCmd(byte cmd_type, float cmd_goal, int cmd_zone_delay)
{

	// NOTE: arg2 = reward delay or zone ind or reward duration

  rewDuration = 400;
  feed(rewDuration);
	// Local vars
	// int cmd_zone_ind = -1;
	// int cmd_delay = -1;

	// // Store mode
	// rewMode =
	// 	cmd_type == 0 ? BUTTON :
	// 	cmd_type == 1 ? NOW :
	// 	cmd_type == 2 ? CUE :
	// 	FREE;

	// // Update counts
	// if (rewMode != BUTTON)
	// {
  //   // rewDuration = 400;
  //   QueuePacket(&r2c, 'R', c2r.dat[0], c2r.dat[1], c2r.dat[2], 0, false, true, false, false);
  //   delay(1);
  //   SendPacket(&r2c);
	// 	cnt_cmd++;

  //   // feed(rewDuration);
	// }
	// cnt_rew++;

	// // Format string

	// // Handle zone/delay arg
	// if (rewMode == NOW || rewMode == CUE)
	// {
  //   QueuePacket(&r2c, 'R', c2r.dat[0], c2r.dat[1], c2r.dat[2], 0, false, true, false, false);
  //   delay(1);
  //   SendPacket(&r2c);
	// 	// Set to zero based index
	// 	cmd_zone_ind = cmd_zone_delay - 1;

  //   // feed(rewDuration);
	// }
	// else if (rewMode == FREE)
	// {
	// 	cmd_delay = cmd_zone_delay;
	// }

	// // Setup "BUTTON" reward
	// if (rewMode == BUTTON)
	// {
  //   QueuePacket(&r2c, 'R', c2r.dat[0], c2r.dat[1], c2r.dat[2], 0, false, true, false, false);
  //   delay(1);
  //   SendPacket(&r2c);
	// 	// Set duration to default
	// 	SetZoneDur(400);
  //   RunReward();
	// }

	// // Setup "NOW" reward
	// else if (rewMode == NOW)
	// {

	// 	// Set duration
	// 	// SetZoneDur(cmd_zone_ind);
  //   rewDuration = 400;
  //   feed(rewDuration);
  //   // RunReward();
	// }

	// // Setup "CUE" reward
	// else if (rewMode == CUE)
	// {

	// 	// Include specified zone
	// 	zoneMin = cmd_zone_ind;
	// 	zoneMax = cmd_zone_ind;

	// 	// Set zone bounds
	// 	SetZoneBounds(cmd_goal);

	// 	// Set delay to zero
	// 	rewDelay = 0;
	// }

	// // Setup "FREE" reward
	// else if (rewMode == FREE)
	// {

	// 	// Include all zones
	// 	zoneMin = 0;
	// 	zoneMax = zoneLng - 1;

	// 	// Set zone bounds
	// 	SetZoneBounds(cmd_goal);

	// 	// Store reward delay time in ms
	// 	rewDelay = cmd_delay * 1000;
	// }

	// // Log

}

bool RunReward()
{


	// Local vars
	bool reward_done = false;

	// Bail if rewarding
	if (isRewarding)
	{

		return reward_done;
	}

	// Zone not triggered yet
	if (!isZoneTriggered)
	{

		// Check each zone
		if (CheckZoneBounds())
		{

      delay(rewDelay);
			// Start reward
			feed(rewDuration);

			// Print message
		

			// Set done flag
			reward_done = true;

		}
	}

	// Check if rat passed all bounds
	if (isAllZonePassed &&
		!isZoneTriggered)
	{

		// Print reward missed

		// Send missed reward msg
		QueuePacket(&r2c, 'Z', cnt_rew, 0, zoneInd + 1, 0, true, false, false, false);
    delay(1);
    SendPacket(&r2c);

		// Decriment reward count
		cnt_rew--;

		// Reset flags
		RewardReset(reward_done);

		// Set done flag
		reward_done = true;

	}

	// Return flag
	return reward_done;
}




void SetZoneDur(int zone_ind)
{
	

	// Local vars
	

	// Set zone ind
	if (zone_ind != -1)
	{
		zoneInd = zone_ind;
	}

	// Find default ind
	else {
		for (int i = 0; i < zoneLng; i++)
		{
			zoneInd = zoneRewDurs[i] == durationDefault ? i : zoneInd;
		}
	}

	// Set duration
	rewDuration = zoneRewDurs[zoneInd];

	// Log

	
}

void SetZoneBounds(float cmd_goal)
{
	

	// Local vars
	
	int diam = 0;
	int pos_int = 0;
	double pos_cum = 0;
	double dist_center_cm = 0;
	double dist_start_cm = 0;
	double dist_end_cm = 0;

	// Compute laps
	diam = (int)(140 * PI * 100);
	pos_int = (int)(RatPos_cm * 100);
	lapN = round(RatPos_cm / (140 * PI) - (float)(pos_int % diam) / diam);
	// Check if rat 'ahead' of rew pos
	pos_cum = (double)(pos_int % diam) / 100;
	// Add lap
	lapN = pos_cum > cmd_goal ? lapN + 1 : lapN;

	// Compute reward center
	goalPosCum = cmd_goal + lapN*(140 * PI);

	// Compute bounds for each zone
	for (int i = zoneMin; i <= zoneMax; i++)
	{
		// Get zone width with overlap for center bins
		int zone_bnd_start = zoneMin == zoneMax || i == zoneMin ?
			rewZoneWidth / 2 : rewZoneWidth;
		int zone_bnd_end = zoneMin == zoneMax || i == zoneMax ?
			rewZoneWidth / 2 : rewZoneWidth;

		// Compute zone bounds
		dist_center_cm = -1 * zoneLocs[i] * ((140 * PI) / 360);
		dist_start_cm = dist_center_cm - (zone_bnd_start * ((140 * PI) / 360));
		dist_end_cm = dist_center_cm + (zone_bnd_end * ((140 * PI) / 360));

		// Store in array
		zoneBoundCumMin[i] = goalPosCum + dist_start_cm;
		zoneBoundCumMax[i] = goalPosCum + dist_end_cm;
	}

	// Print message
	

	
}

bool CheckZoneBounds()
{
	

	// Run only if reward not already triggered
	if (isZoneTriggered)
	{
		
		return isZoneTriggered;
	}

	// Bail if pos data not new
	if (!is_ekfNew)
	{
		
		return isZoneTriggered;
	}

	// Reset flag
	is_ekfNew = false;

	// Check if all bounds passed
	if (RatPos_cm > zoneBoundCumMax[zoneMax] + 5)
	{
		isAllZonePassed = true;
		
		return isZoneTriggered;
	}

	// Bail if first bound not reached
	if (RatPos_cm < zoneBoundCumMin[zoneMin])
	{
		
		return isZoneTriggered;
	}

	// Check if rat in any bounds
	for (int i = zoneMin; i <= zoneMax; i++)
	{
		if (
			RatPos_cm > zoneBoundCumMin[i] &&
			RatPos_cm < zoneBoundCumMax[i]
			)
		{

			// Update timers
			t_lastZoneCheck = t_lastZoneCheck == 0 ? millis() : t_lastZoneCheck;
			t_nowZoneCheck = millis();

			// Store occupancy time
			zoneOccTim[i] += t_nowZoneCheck - t_lastZoneCheck;
			zoneOccCnt[i]++;
			t_lastZoneCheck = t_nowZoneCheck;

			// Check if occ thresh passed
			if (zoneOccTim[i] >= rewDelay)
			{

				// REWARD at this pos
				SetZoneDur(i);

				// Store reward info for debugging
				zoneRewarded = zoneLocs[i] * -1;
				zoneBoundCumRewarded[0] = zoneBoundCumMin[i];
				zoneBoundCumRewarded[1] = zoneBoundCumMax[i];
				occRewarded = zoneOccTim[i];

				// Set flag
				isZoneTriggered = true;
			}
		}
	}

	
	return isZoneTriggered;
}



void RewardReset(bool was_rewarded)
{
	

	// Local vars

	// Log event
	

	
	// Reset flags etc
	isRewarding = false;
	isZoneTriggered = false;
	isAllZonePassed = false;
	is_ekfNew = false;

	// Reset occ time
	for (int i = 0; i < zoneLng; i++)
	{
		zoneOccTim[i] = 0;
		zoneOccCnt[i] = 0;
	}
	t_nowZoneCheck = 0;
	t_lastZoneCheck = 0;

	
}


////////////////////////////// Auxiliary Motor Functions ///////////////////////////////////////////

void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the L6474 command GET_STATUS */
  uint16_t statusRegister = myL6474.CmdGetStatus(0, 0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
  {
    // HIZ state
  }

  /* Check direction bit */
  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
  {
    // Forward direction is set
  }  
  else
  {
    // Backward direction is set
  }  

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L6474 */
  /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
  {
       // Command received by SPI can't be performed
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
  }    

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached * 
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
  {
    //overcurrent detection 
  }      
}
