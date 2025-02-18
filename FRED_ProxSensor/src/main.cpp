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


//#############  Variables  ##########

// Arduino Pins
const int proxSensor = 21;
const int proxSensor2 = 20;
const int LED = 53;

// Misc
int proxSensorVal;
int prox2SensorVal;
int bullSpeed = 1000;
int rewDurationMs = 0; // (ms) 
int packNum = 0;
int normalRobotMaxSpeed = 4000;
int normalRobotMinSpeed = 1000;
int normalRobotAcceleration = 500;

// Flags
bool isMoveForwardPressed = false; 
bool isMoveBackwardPressed = false; 
bool isEtohPressed = false;
bool isFeedPressed = false;
bool isHalted = false;
bool isMovePack = false;
bool isLEDon = true;
bool isMoveReady = false;
bool isMoving = false;
bool isRewardPending = false;
bool isRobotInFeedPosition = false;
bool isRobotBulldozing = false;
bool isRewarding = false;
bool isMoveInterrupted = false;
bool isSendingPacket = false;
bool isSessionStarted = false;

//Object Initialization
L6474 motors; 

//Function Declarations
void sysTest();
void feedRoutine(int rewDuration);
void extendFeederPlate(L6474 motors);
void retractFeederPlate(L6474 motors);
void stopRobot();
void checkMove();
void checkPreviousMove();
void getMotorStatus(void);


// create 4 states for button debounce
typedef enum stateType_enum {
  wait_press,
  debounce_press,
  wait_release,
  debounce_release
} stateType;

// Initialize states.  Remember to use volatile
// set pushbutton state to initial position of wait_press
// make state variable volatile type since it can be changed in the ISR
volatile stateType pbstate = wait_press;



void setup() {
  Serial.begin(9600);          //Begin Serial Monitor (for debugging)
  Serial1.begin(57600);        //  ???????
  r2c.hwSerial.begin(57600);   //Begin Serial Communication (XBee)
  SPI.begin();                 //Begin SPI Communication

  sei(); // Enable global interrupts.

  //Serial.println("Setup...");   *Used for debugging

  // Enable and turn on robot's LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  //Setup the peripheral devices 
  ButtonSetup();
  LCDSetup();
  PumpSetup();


  // Tnitialize motors and set parameters
  motors.Begin(1, 0);

  // Attach the flag interrupt handler for check motor status
  motors.AttachFlagInterrupt(getMotorStatus);  
 
  motors.SetMaxSpeed(0, normalRobotMaxSpeed);
  motors.SetMinSpeed(0, normalRobotMinSpeed);
  motors.SetAcceleration(0, normalRobotAcceleration);
  motors.SetDeceleration(0, 3300);
  motors.SelectStepMode(0, L6474_STEP_SEL_1_4, 1);

  motors.SetMaxSpeed(1,3000);
  motors.SetMinSpeed(1,2000);
  motors.SetAcceleration(1, 1000);
  motors.SetDeceleration(1, 1000);

  motors.SetHome(0, 0);
  motors.SetHome(0, 1);

  // Test all components individually
  sysTest(); 

  Serial.println("Setup Complete!");
  writeLCD("SysTest Complete!", 0);

  // Add proximity sensor interrupts
  attachInterrupt(digitalPinToInterrupt(proxSensor), checkMove, CHANGE);
  attachInterrupt(digitalPinToInterrupt(proxSensor2), checkMove, CHANGE);

  // Add interrupt for when robot receives XBEE packet
  attachInterrupt(digitalPinToInterrupt(19), stopRobot, RISING);

}

void loop() {
  // Check Proiximity Sensor Values
  // If activated and not bulldozibg, stop the robot 
  // if ((proxSensorVal == 0 || prox2SensorVal == 0 ) && isRobotBulldozing == false){
  //   motors.HardStop(0, 1);
  // }

  // Get Data from C# and process command  
  GetSerial(&c2r);   //** Put in pin 19 interrupt?  **//


  // Pump Ethanol every 10 seconds once a session has started 
  if (isSessionStarted == true && isHalted == false && isRewarding == false){
    if (millis() % 60000 < 100){
      pumpEtoh();
    }
    checkPreviousMove();
  }


  // Check if robot is in position to feed  rat and reward is pending. If so, dispense reward
  if (isRewardPending == true && isRobotInFeedPosition == true){
    motors.HardStop(0, 0);
    isMoving = false;
    writeLCD("Rewarding...", 0);
    delay(10);
    feedRoutine(rewDurationMs);
    // Serial.print("Rewarded for ");     *Used for debugging
    // Serial.println(rewDurationMs);
    isRewardPending = false;
    clearLCD();
  }

  // If robot has received bulldoze command, start bulldozing
  if (isRobotBulldozing == true)
  {

    // Set motor speed and acceleration for bulldozing
	  motors.SetMaxSpeed(0, bullSpeed);
  	motors.SetMinSpeed(0,500);
  	motors.SetAcceleration(0, 100);

    writeLCD("Bulldozing", 0);
    delay(10);

    isMoving = true;
    motors.Run(0, FORWARD, 1);
    delay(7000);
    motors.HardStop(0, 1);
    isMoving = false;

    delay(10);
    clearLCD();
    
    //writeLCD("Stopped Bulldozing", 0);
    // delay(1000);
    // clearLCD();

    isRobotBulldozing = false;

    //writeLCD("Sending Stop Bulldoze Packet", 0);
    for (int i = 0; i < 10; i++)
    {
      QueuePacket(&r2c, 'B', 1, 0, 0, packNum, false, true, true, true);
      delay(1);
      SendPacket(&r2c);
    }

    delay(10);
    
    // Set motor speed and acceleration back to normal
	  motors.SetMaxSpeed(0, normalRobotMaxSpeed);
    motors.SetMinSpeed(0, normalRobotMinSpeed);
    motors.SetAcceleration(0, normalRobotAcceleration);
	  
    motors.HardStop(0, 1);
    isMoving = false;

    // Check to see if robot needs to move forward after bulldozing
    checkPreviousMove();
   }
  


/////         BUTTON HANDLING           ////

  // Pump Ethanol using button on robot
  if (!(PINC & (1 << PINC5))) {
    writeLCD("Pumping:", 0);
    writeLCD("Ethanol motor", 1);
    // Serial.println("Ethanol Button Pressed");   *Used for debugging
    digitalWrite(5, HIGH);
    isEtohPressed = true;
  }
  else {  
    if (isEtohPressed == true){
      digitalWrite(5, LOW);
      isEtohPressed = false;
      clearLCD();
    }
  }



//  Pump Food using button on robot
  if (!(PINC & (1 << PINC3))) {
    writeLCD("Pumping:", 0);
    writeLCD("Food motor", 1);
    // Serial.println("Food button pressed");  *Used for debugging
    digitalWrite(4, HIGH);
    isFeedPressed = true;
  }

else {
  if (isFeedPressed == true){
    digitalWrite(4, LOW);
    clearLCD();
    isFeedPressed = false;
  }
}

//   Do Feeding Routine using button on robot
  if (!(PINC & (1 << PINC1))) {
    writeLCD("Feeding Routine", 0);
    // Serial.println("Feeding Routine button pressed");  *Used for debugging
    feedRoutine(900);   
    clearLCD();
  }

  

// Move Forward using button on robot
if (!(PINC & (1 << PINC4)) ){
  writeLCD("Moving Forward...", 0);
  // Serial.println("Move Forward button pressed");  *Used for debugging
  motors.SetMaxSpeed(0, normalRobotMaxSpeed);
  motors.SetMinSpeed(0, normalRobotMinSpeed);
  motors.SetAcceleration(0, normalRobotAcceleration);

  isMoving = true;
  motors.Run(0, FORWARD, 1);
  isMoveForwardPressed = true;
  }
while (!(PINC & (1 << PINC4)) ){
   //Run this while button is pressed 
}
if (isMoveForwardPressed == true){
  clearLCD();
  motors.SoftStop(0);
  isMoveForwardPressed = false; 
  motors.HardStop(0, 1);
  isMoving = false;
}


// Move Backward using button on robot
if (!(PINC & (1 << PINC2)) ){
  writeLCD("Moving Backward...", 0);
  // Serial.println("Move Backward button pressed");  *Used for debugging
  motors.SetMaxSpeed(0, normalRobotMaxSpeed);
  motors.SetMinSpeed(0, normalRobotMinSpeed);
  motors.SetAcceleration(0, normalRobotAcceleration);
  isMoving = true;
  motors.Run(0, BACKWARD, 1);
  isMoveBackwardPressed = true;
}
while (!(PINC & (1 << PINC2)) ){
    //Run this while button is pressed
}
if (isMoveBackwardPressed == true){
  clearLCD();
  motors.SoftStop(0);
  isMoveBackwardPressed = false; 
  motors.HardStop(0, 1);
  isMoving = false;
}
  
// Turn LCD Backlight On/Off using button on robot
if (!(PINC & (1 << PINC0)) ){
Serial.println("Button Pressed");

  if (isLEDon == true){
    // Serial.println("Turning LCD Backlight Off");  *Used for debugging
    writeLCD("Turning LCD Backlight", 0);
    writeLCD("On", 1);
    LCDBacklightOn();
    clearLCD();
    isLEDon = false;
  }
  else{
    // Serial.println("Turning LCD Backlight Off");  *Used for debugging
    writeLCD("Turning LCD Backlight", 0);
    writeLCD("Off", 1);
    LCDBacklightOff();
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
  
    // button not released yet
    case wait_release:

    break;
  
    // button just released
    case debounce_release:
      delay(2);
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
    //Serial.println("Release");
    pbstate = debounce_release;
  }
}



// Test all robot components individually; Used at startup  
void sysTest(){
    // Test LCD
    writeLCD("System Test", 0);

    //Test Feeding Pump
    writeLCD("Testing Feeding Pump", 1);
    pumpFood(500);

    //Test Ethanol Pump
    delay(1500);
    clearLine(1);
    writeLCD("Testing Ethanol Pump", 1);
    pumpEtoh();

    
    // Test Movement Motor
    clearLine(1);
    writeLCD("Testing Movement", 1);
    motors.Run(0, FORWARD, 1);
    delay(2000);
    motors.SoftStop(0);
    delay(2000);
    motors.Run(0, BACKWARD, 1);
    delay(2000);
    motors.HardStop(0, 1);
    delay(1000);

    // Test Feeder Plate Motor
    delay(2000);
    clearLine(1);
    writeLCD("Testing Feeder Plate", 1);
    extendFeederPlate(motors);
    delay(2000);
    retractFeederPlate(motors);
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
void feedRoutine(int rewDuration){
    isRewarding = true;
    Serial.println(rewDuration);
    delay(10);
    extendFeederPlate(motors);
    delay(200);
    pumpFood(rewDuration);
    
    delay(7000); // update to 20 seconds delay

    retractFeederPlate(motors);
    for (int i=0; i<10; i++){
      QueuePacket(&r2c, 'R', 9, 9, 9, packNum+1, false, true, true, true);
      delay(5);
      SendPacket(&r2c);
    }

    isRewarding = false;
    checkPreviousMove();
}



void extendFeederPlate(L6474 motors){

  // Stop motor before moving
  motors.HardStop(0, 1);
  isMoving = false;
  delay(100);

  // Set motor speed and acceleration for extending feeder plate
  motors.SetMaxSpeed(0,2000);
  motors.SetMinSpeed(0,2000);
  motors.SetAcceleration(0,1000);
  motors.SetDeceleration(0,1000);

  // Move motor to extend feeder plate
  motors.GoTo(0, -1700, 0);
  delay(750); //750ms delay = ~90 degree motor turn
  motors.HardStop(0, 0);

  // Set motor speed and acceleration back to normal
  motors.SetMaxSpeed(0, normalRobotMaxSpeed);
  motors.SetMinSpeed(0, normalRobotMinSpeed);
  motors.SetAcceleration(0, normalRobotAcceleration);
  motors.SetDeceleration(0,1000);
}

void retractFeederPlate(L6474 motors){

  // Stop motor before moving
  motors.HardStop(0, 1);
  isMoving = false;
  delay(100);

  // Set motor speed and acceleration for retracting feeder plate
  motors.SetMaxSpeed(0,2000);
  motors.SetMinSpeed(0,2000);
  motors.SetAcceleration(0,1000);
  motors.SetDeceleration(0,1000);

  // Move motor to retract feeder plate
  motors.GoTo(0, 1500, 0);
  delay(750); //220ms delay = ~90 degree motor turn
  motors.HardStop(0, 0);

  // Set motor speed and acceleration back to normal
  motors.SetMaxSpeed(0, normalRobotMaxSpeed);
  motors.SetMinSpeed(0, normalRobotMinSpeed);
  motors.SetAcceleration(0, normalRobotAcceleration);
  motors.SetDeceleration(0,1000);
}


// Function used to check if robot needs to move based on proximity sensor values
  /*
    - If either or both sensors are activated, robot will stop.
    - If only front sensor is activated, robot will get ready to move forward. 
        Else, robot will not be ready to move
    - If both sensors are inactive, and robot is ready to move forward, robot will move forward
  */
void checkMove(){

  clearLCD();

  proxSensorVal = digitalRead(proxSensor);
  prox2SensorVal = digitalRead(proxSensor2);


  if (proxSensorVal == 0){
    isRobotInFeedPosition = true;
  }
  else {
    isRobotInFeedPosition = false;
  }

  // If both sensor are activated, stop robot
  if (proxSensorVal == 0 && prox2SensorVal == 0 && isRobotBulldozing == false){
    if (isRobotBulldozing == false) {
      motors.HardStop(0, 1);
      isMoving = false;
    }
    isMoveReady = false;
    isMovePack = false;
  }
  // If only front sensor is activated, get ready to move forward
  else if (proxSensorVal == 0 && prox2SensorVal == 1){
    if (isRobotBulldozing == false) {
      motors.HardStop(0, 1);
      isMoving = false;
    }
    isMoveReady = true;
    isMovePack = false;
  }
  else if (proxSensorVal == 1 && prox2SensorVal == 0){
    if (isRobotBulldozing == false) {
      motors.HardStop(0, 1);
      isMoving = false;
    }
    isMoveReady = false;
    isMovePack = false;
  }
  // If both sensor are inactive, determine whether rat is in front or behind sensors
  else if (proxSensorVal == 1 && prox2SensorVal == 1){
    // If rat is in front of robot, move forward
    if (isMoveReady == true && isHalted == false && isRewarding == false && isRobotBulldozing == false){
      motors.SetMaxSpeed(0, normalRobotMaxSpeed);
      motors.SetMinSpeed(0, normalRobotMinSpeed);
      motors.SetAcceleration(0, normalRobotAcceleration);
      delay(100);
      isMoving = true;
      motors.Run(0, FORWARD, 1);
      delay(100);
    }
  }
}


// Function used to check if robot needs to move after processing other tasks
// Prevents robot from standing still if rat runs away while robot is busy
void checkPreviousMove(){

  proxSensorVal = digitalRead(proxSensor);
  prox2SensorVal = digitalRead(proxSensor2);

  if (isMoveInterrupted == true && proxSensorVal == 1 && prox2SensorVal == 1 && isMoveReady == true){
    motors.SetMaxSpeed(0, normalRobotMaxSpeed);
    motors.SetMinSpeed(0, normalRobotMinSpeed);
    motors.SetAcceleration(0, normalRobotAcceleration);
    delay(100);
    isMoving = true;
    motors.Run(0, FORWARD, 1);
    delay(100);
    isMoveInterrupted = false;
  }

}


// Function used to stop motor in order to process packet
// If the robot is actively sendinga packet, bulldozing, or moving, the robot will not stop
void stopRobot(){
  if (isSendingPacket == false && isRobotBulldozing == false && isMovePack == false){
    motors.HardStop(0, 1);
    isMoving = false;
    isMoveInterrupted = true;
  }
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

  isSendingPacket = true;

  // Serial.println("Sending Packet");   *Used for debugging
  // clearLCD(); 
  // writeLCD("Sending Packet", 0);
  // delay (1000);
  // clearLCD();


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

  // Serial.println("Send Packet");

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

  // Bail if buffer or time inadequate
  if (tx_size > tx_sizeMaxSend || rx_size > rx_sizeMaxSend ||
      millis() < p_r2->t_sent + p_r2->dt_minSentRcvd) {
      // Serial.println("Bail if buffer or time inadequate");
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

  // Serial.println("Done sending data");

  isSendingPacket = false;

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
      // Serial.println("OVERFLOW!!");
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

  return 0;
}

void GetSerial(R4_COM<HardwareSerial> *p_r4) {

  //Serial.println("Get serial");  *Used for debugging
  // writeLCD("Get Serial", 0);
  // delay(1500);
  // clearLCD();

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
   // Serial.println("HW Serial not Available");  *Used for debugging
    return;
  }
  // else {
  //   Serial.println("Data is Available");  *Used for debugging
  // }


  // Dump data till p_msg header byte is reached
  b = WaitBuffRead(p_r4, p_r4->head);
  if (b == 0) {
    return;
  }

  // Store header
  head = b;

  // Serial.print("Header: ");  *Used for debugging
  // Serial.println(head);

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

    //   Serial.println("Raw Data");  *Used for debugging
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
  

  Serial.println("ID:  " + id);
  
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


    // Send confirmation
    if (do_conf) {
      // writeLCD("Sending Confirmation Confirm", 0);  *Used for debugging
      // delay(1000);
      // clearLCD();
      // Serial.println("Do Confirm");
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
    is_repeat = pack < pack_last || pack == pack_last;

    // Incriment repeat count
    p_r4->cnt_repeat += is_repeat || is_resend ? 1 : 0;

    

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


  if (is_repeat == false){

    // Update packet history
    if (!is_conf) {
      p_r4->packArr[r4_ind] = pack;
    }
    else {
      p_r4->packConfArr[r4_ind] = pack;
    }

    switch (id) {
      case ('h'): /*     Handshake Packet      */
      
        // writeLCD("Received 'h' Packet", 0);  *Used for debugging
        // delay(1500); 
        // clearLCD();
        //  Serial.println("Case h");

        if (dat[0] == (float)1) {   // Handshake packet send when GUI is opened
          
          //Send confirmation packet
          QueuePacket(&r2c, 'h', 1, 0, 0, 0, false, true, true, true);  
          delay(1);
          SendPacket(&r2c);  
        }
    
        if (dat[0] == (float)2) {   // Handshake packet send when Session is started

          //Send confirmation packet
          QueuePacket(&r2c, 'h', 2, 0, 0, 0, false, true, true, true);
          delay(1);
          SendPacket(&r2c);  

          // Turn off LCD backlight
          LCDBacklightOff();
        }

        break;
      
      case ('B'):  /*    Bulldoze Packet      */

        // writeLCD("Received 'B' Packet", 0); *Used for debugging
        // delay (1000);
        // clearLCD();
        // Serial.println("Case B");

        // If Data[0] is greater than 0, start robot bulldozing
        if (isRobotBulldozing == false && dat[0] > (float)0){ 
          //bullSpeed = (float)dat[1];
          isRobotBulldozing = true;
        }
        // If Data[0] is 0, stop robot bulldozing 
        else if (dat[0] == (float)0){
          isRobotBulldozing = false;
          motors.HardStop(0, 1);      
          isMoving = false;
        }

        break;
        
      case ('H'):  /*    Halt Packet   */

        // writeLCD("Received 'H' Packet", 0);  *Used for debugging
        // delay(1500);
        // clearLCD();
        // Serial.println("Case H");

        // Stop Robot
        motors.HardStop(0, 1);
        isMoving = false;

        // Send confirmation packet
        QueuePacket(&r2c, 'H', c2r.dat[0], c2r.dat[1], c2r.dat[2], packNum, false, true, true, true);
        delay(5);
        SendPacket(&r2c);
        
        // If Data[0] is 1, halt robot
        if (dat[0] == 1){
          isHalted = true;
          writeLCD("Halted", 0);
          
        }
        // If Data[0] is 2, resume robot
        else if (dat[0] == 2){
          isHalted = false;
          clearLCD();
        }
        break;


      case ('M'):  /*   Move Packet    */

        // writeLCD("Received 'M' Packet", 0);  *Used for debugging
        // delay(1500);
        // clearLCD();
        // Serial.println("Case M");

        // Send confirmation packet
        QueuePacket(&r2c, 'M', c2r.dat[0], c2r.dat[1], c2r.dat[2], packNum, false, true, true, true);
        delay(5);
        SendPacket(&r2c);
        
        delay(10);

        // If this is first 'M' packet, indicate start session
        if (isSessionStarted == false){
          isSessionStarted = true;
        }
        
        isMovePack = true;

        // Serial.println("Move Forward");   *Used for debugging
        writeLCD("Moving Forward...", 0); 

        // Move robot forward
        motors.SetMaxSpeed(0, normalRobotMaxSpeed);
        motors.SetMinSpeed(0, normalRobotMinSpeed);
        motors.SetAcceleration(0, normalRobotAcceleration);
        delay(100);
        isMoving = true;
        motors.Run(0, FORWARD, 1);
        delay(10000);

        
        break;
        
      case ('R'):  /*    Reward Packet   */

        //   writeLCD("Received 'R' Packet", 0);  *Used for debugging
        //  delay(1500);
        //   clearLCD();
        // Serial.println("Case R");

        // Send confirmation packet
        QueuePacket(&r2c, 'R', c2r.dat[0], c2r.dat[1], c2r.dat[2], packNum, false, true, true, true);
        delay(5);
        SendPacket(&r2c);

      //  Millileters of liquid dispensed  -  Time of pump active in seconds Conversion
      /* 
            1ml      -  900ms    
            0.75ml   -  675ms    
            0.6ml    -  540ms    
            0.425ml  -  410ms    
            0.25ml   -  250ms    
      */
      
      // Set time for feeding pump to be active based on data received  
        if (dat[0] == (float)1){
          rewDurationMs = 250;
        }
        else if (dat[0] == (float)2){
          rewDurationMs = 410;
        }
        else if (dat[0]  == (float)3){
          rewDurationMs = 540;
        }
        else if (dat[0]  == (float)4){
          rewDurationMs = 675;
        }
        else if (dat[0]  == (float)5){
          rewDurationMs = 900;
        }

        // Check what kind of Reward packet it is
        if (dat[1] == (float)1){         // If packet came from pressing Reward button on GUI
          feedRoutine(rewDurationMs);    // Reward now
        }
        else if (dat[1] == (float)0){    // If packet was sent automatically
          isRewardPending = true;        // Wait until robot is in position
        }
        
        

        // Check if robot needs to move after rewarding
        checkPreviousMove();
      
        break;

        case ('F'):    /* Finalization/End session packet*/
          motors.HardStop(0, 1);
          isMoving = false;

          // Reset all variables
          bullSpeed = 0;
          rewDurationMs = 0; // (ms) 
          packNum = 0;
          isMoveForwardPressed = false; 
          isMoveBackwardPressed = false; 
          isEtohPressed = false;
          isFeedPressed = false;
          isHalted = false;
          isMovePack = false;
          isLEDon = true;
          isMoveReady = false;
          isMoving = false;
          isRewardPending = false;
          isRobotInFeedPosition = false;
          isRobotBulldozing = false;
          isRewarding = false;
          isMoveInterrupted = false;
          isSendingPacket = false;
          isSessionStarted = false;

          p_r4->packArr[r4_ind] = 0;

          LCDBacklightOn();
          clearLCD();
          writeLCD("Session Ended", 0);
          delay(1000);
          clearLCD();
        break;

    }
  }
  
}

////////////////////////////// Auxiliary Motor Functions ///////////////////////////////////////////

void getMotorStatus(void)
{
  /* Get the value of the status register via the L6474 command GET_STATUS */
  uint16_t statusRegister = motors.CmdGetStatus(0, 0);

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


