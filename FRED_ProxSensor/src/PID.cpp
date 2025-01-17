#include <Arduino.h>
#include <SPI.h>
// #include "PIDLoop.h"
#include <stdlib.h>
#include <math.h>
// #include <PID_v1.h>


using namespace std;


double milliLast=0;
double errorLast=0;
double runSpeed=0.0;
double velUpdate=0.0;
double kC = 1;
double Pc = 3;
double dT = 0;
double kP =0;
double kI =0;
double kD =0;
double error=0;
double integral=0;
double derivative=0;
double p_term=0;
double i_term=0;
double d_term=0;

//Determine motor speed based on the difference in postion between rat and robot
double getPIDSpeed(double RatPos, double RobPos, double *errorLast, double *milliLast){
  
  // Compute error 
  kP = 0.6*kC;
  kD = (2*kP) /Pc;
  kI = kP*Pc / (8);


  dT = (millis() - (*milliLast))/1000;
  *milliLast = millis();


  error = RatPos - RobPos;
  integral = error;
  derivative = error - *errorLast;

  p_term = kP*error;
  i_term = kI*dT*integral;
  d_term = kD / dT*derivative;

  velUpdate = p_term + i_term + d_term;
  runSpeed = velUpdate;

  *errorLast = error;

  // Serial.print("Derivative: ");
  // delay(5);
  // Serial.println(derivative);
  // delay(5);
  // Serial.print("Integral: ");
  // delay(5);
  // Serial.println(integral);
  // delay(5);
  // Serial.print("D: ");
  // delay(5);
  // Serial.println(d_term);
  // delay(5);
  // Serial.print("dT: ");
  // delay(5);
  // Serial.println(dT);
  // delay(5);

  return runSpeed;
}


double GetPID(double RatPos, double RobPos){
  double RobPos_corrected = RobPos;

  // rad1: robot
  // rad2: rat
  // when rad2 negative, add 2pi to rad2: rad2[i] +2pi, 
  // rat always less than robot, rad1 > rad2,
  // if rad1 < rad2, then add 2pi to rad1: rad1[i] +2pi
  // seems redundant here, but no negatives in real data,
  // only +2pi will be rad1 when rad2(rat) crosses 0 threshold into 2pi

  if (RobPos < RatPos){
      RobPos_corrected = RobPos + (2*M_PI);
  }

  runSpeed = getPIDSpeed(RobPos_corrected, RatPos, &errorLast, &milliLast);

  // Serial.print("Radian Angles: ");
  // Serial.print(rad1[i]);
  // delay(5);
  // Serial.print("    ");
  // delay(5);
  // Serial.println(rad2[i]);
  // delay(5);


  // Serial.print("PID Error: ");
  // Serial.println(errorLast);
  // delay(5);
  // Serial.print("Motor Speed: ");
  // delay(5);
  // Serial.println(runSpeed);
  // delay(5);
  // Serial.println();
  // delay(5);

  return runSpeed;

}

// Function to determine if rat is stopped 
bool isStopped(float ratPos, float *lastRatPosition, int *ratStopTime){
  //Initialize lastRatPos once
  if (*lastRatPosition == -1){
      *lastRatPosition = ratPos;
  }

  // Determine if rat is stopped
  if (abs(*lastRatPosition-ratPos) < 3){
    if (millis() >= *ratStopTime){
      return true;
    }
    else{
      return false; 
    }
  }

  else {
      *ratStopTime = millis() + 5000;
  }
}

// Determine food amount depending on rat's position relative to cue
// Currently uses placeholders 1,2,3,4
// returned int is time feeding pump will run
// every 100ms = .05ml up to 400ms at .2ml
int rewardAmount(float ratPos, float cuePos){
  int posDiff = abs(ratPos - cuePos);

  if (posDiff <= 1){
    return 400;
  }
  else if (posDiff > 1 && posDiff <= 2){
    return 300;
  }
  else if (posDiff > 2 && posDiff <= 3){
    return 200;
  }
  else if (posDiff > 3 && posDiff <= 4){
    return 100;
  }

}

