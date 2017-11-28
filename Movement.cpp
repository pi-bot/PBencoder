/*
 * Movement.c
 *
 * Created: 25/02/2016 20:35:56
 *  Author: Olly						changed by Jason for PiBot v8.0 10/05/2016
 */


#include <Arduino.h>
#include "PID.h"
#include "Movement.h"
#include "Encoder.h"
#include "elapsedMillis.h"


int L_MotorPins[2] = {6,10};
int R_MotorPins[2] = {5,9};
int En_MotorPin = 7;  			// not an enable only a mode pin on PiBot v8.0

int P_pos =8;
float I_pos =0.08;
float D_pos =0.3;

bool at_position = false;

bool R_prev_dir;
bool L_prev_dir;

int dir;


bool Rotate = false;
bool Move = false;

// Position controllers

double Kp_Pos=0, Ki_Pos=0, Kd_Pos=0;

bool L_MotorEn = false;
bool R_MotorEn = false;

double L_TargPos, L_CurrPos, L_OutputPos, OutputL, L_diff_delta;
double R_TargPos, R_CurrPos, R_OutputPos, OutputR, R_diff_delta;

PID L_PID_Pos(&L_CurrPos, &L_OutputPos, &L_TargPos, Kp_Pos, Ki_Pos, Kd_Pos, DIRECT);
PID R_PID_Pos(&R_CurrPos, &R_OutputPos, &R_TargPos, Kp_Pos, Ki_Pos, Kd_Pos, DIRECT);




// Differential control
double Kp_Diff=0, Ki_Diff=0, Kd_Diff=0;

double TargDiff=0, CurrDiff, OutputDiff;


PID PID_Diff(&CurrDiff, &OutputDiff, &TargDiff, Kp_Diff, Ki_Diff, Kd_Diff, DIRECT);


elapsedMillis timeElapsed_L;
elapsedMillis timeElapsed_R;
bool time_toggle_L = 0;
bool time_toggle_R = 0;


void movePos(int deltaPos, int speed)
{


//  if(speed<50){
//    speed = 50;
//  }
//  else if(speed>150){
//    speed = 150;
//  }
   speed = 220;


  L_PID_Pos.SetOutputLimits(-speed,speed);
  L_ENC.reset();
  L_TargPos = DIST_TO_CLICKS(deltaPos);//L_CurrPos + DIST_TO_CLICKS(deltaPos);

  R_PID_Pos.SetOutputLimits(-speed,speed);
  R_ENC.reset();
  R_TargPos = DIST_TO_CLICKS(deltaPos);//R_CurrPos + DIST_TO_CLICKS(deltaPos);

  dir = deltaPos;

  L_PID_Pos.SetTunings(P_pos, I_pos, D_pos);
  R_PID_Pos.SetTunings(P_pos, I_pos, D_pos);
  PID_Diff.SetTunings(P_pos, I_pos, D_pos);

  L_MotorEn = true;
  R_MotorEn = true;

  at_position = false;
}

int getDistanceLeft(void){
  return CLICKS_TO_DIST(-L_ENC.read());
}

int getDistanceRight(void){
  return CLICKS_TO_DIST(-R_ENC.read());
}

void moveRotate(double deltaAngle, int speed)
{
// if(speed<80){
//   speed = 80;
// }
//  else if(speed>100){
   speed = 240;
//  }
  L_PID_Pos.SetOutputLimits(-speed,speed);
  L_ENC.reset();
  L_TargPos = - ANGLE_TO_CLICKS(deltaAngle);

  R_PID_Pos.SetOutputLimits(-speed,speed);
  R_ENC.reset();
  R_TargPos = ANGLE_TO_CLICKS(deltaAngle);

  dir = deltaAngle;

  L_PID_Pos.SetTunings(9, 0, 0.05);
  R_PID_Pos.SetTunings(9, 0, 0.05);
  PID_Diff.SetTunings(0, 0, 0);

  L_MotorEn = true;
  R_MotorEn = true;

  at_position = false;
}

void initPID(void)
{
	L_PID_Pos.SetMode(AUTOMATIC);
	R_PID_Pos.SetMode(AUTOMATIC);
	L_PID_Pos.SetSampleTime(1);
	R_PID_Pos.SetSampleTime(1);
	L_PID_Pos.SetOutputLimits(-MAX_SPEED,MAX_SPEED);
	R_PID_Pos.SetOutputLimits(-MAX_SPEED,MAX_SPEED);

	PID_Diff.SetMode(AUTOMATIC);
	PID_Diff.SetSampleTime(1);
	PID_Diff.SetOutputLimits(-10,10);



}

void initMotors(void)
{
	for(int i=0;i<2;i++)
	{
		pinMode(L_MotorPins[i],OUTPUT);
		pinMode(R_MotorPins[i],OUTPUT);
	}
	pinMode(En_MotorPin,OUTPUT);
	digitalWrite(En_MotorPin,HIGH);
}


void motorControl(signed int speed, int* pins)
{


	speed = constrain(speed,-MAX_SPEED,MAX_SPEED);

	if(speed==0 && pins[0]==6){
            analogWrite(10,speed);
        }
        else if(speed==0 && pins[0]==5){
            analogWrite(9,speed);
        }
        else if(speed>0 && pins[0]==6){

            digitalWrite(6,HIGH);
            analogWrite(10,speed);

         }
         else if(speed>0 && pins[0]==5){

            digitalWrite(5,LOW);
            analogWrite(9,speed);

         }
         else if(speed<0 && pins[0]==6){

           speed = abs(speed);
           digitalWrite(6,LOW);
           analogWrite(10,speed);

         }
         else if(speed<0 && pins[0]==5){
            speed = abs(speed);
            digitalWrite(5,HIGH);
            analogWrite(9,speed);
         }

}



void calcPID(void)
{

	L_CurrPos = -L_ENC.read();
	R_CurrPos = -R_ENC.read();

        if(L_TargPos>L_CurrPos){
          L_diff_delta = (L_TargPos-L_CurrPos)+1;
        }
        else{
          L_diff_delta = (L_CurrPos-L_TargPos)+1;
        }
        if(R_TargPos>R_CurrPos){
          R_diff_delta = (R_TargPos-R_CurrPos)+1;
        }
        else{
          R_diff_delta = (R_CurrPos-R_TargPos)+1;
        }

        if(dir>0){
          CurrDiff = R_diff_delta - L_diff_delta;
        }
        else{
          CurrDiff = L_diff_delta - R_diff_delta;
        }

        PID_Diff.Compute();

	if(L_PID_Pos.Compute() && L_MotorEn && R_PID_Pos.Compute() && R_MotorEn)
	{
          if(OutputDiff>0 && R_OutputPos>0 && L_OutputPos>0 || OutputDiff>0 && R_OutputPos<0 && L_OutputPos>0){
//            Serial.print("1");
            motorControl(R_OutputPos,R_MotorPins);
            motorControl(L_OutputPos-OutputDiff,L_MotorPins);
            OutputR =  R_OutputPos;
            OutputL =  L_OutputPos-OutputDiff;
          }

          else if(OutputDiff<0 && R_OutputPos>0 && L_OutputPos>0 || OutputDiff<0 && R_OutputPos>0 && L_OutputPos<0){
//            Serial.print("2");
            motorControl(R_OutputPos+OutputDiff,R_MotorPins);
            motorControl(L_OutputPos,L_MotorPins);
            OutputR =  R_OutputPos+OutputDiff;
            OutputL =  L_OutputPos;
          }

          else if(OutputDiff>0 && R_OutputPos<0 && L_OutputPos<0 || OutputDiff>0 && R_OutputPos>0 && L_OutputPos<0){
//            Serial.print("3");
            motorControl(R_OutputPos,R_MotorPins);
            motorControl(L_OutputPos+OutputDiff,L_MotorPins);
            OutputR =  R_OutputPos;
            OutputL =  L_OutputPos+OutputDiff;
          }

          else if(OutputDiff<0 && R_OutputPos<0 && L_OutputPos<0 || OutputDiff<0 && R_OutputPos<0 && L_OutputPos>0){
//            Serial.print("4");
            motorControl(R_OutputPos-OutputDiff,R_MotorPins);
            motorControl(L_OutputPos,L_MotorPins);
            OutputR =  R_OutputPos-OutputDiff;
            OutputL =  L_OutputPos;
          }

          else{
//            Serial.print("5");
            motorControl(R_OutputPos,R_MotorPins);
            motorControl(L_OutputPos,L_MotorPins);
            OutputR =  R_OutputPos;
            OutputL =  L_OutputPos;
          }
//              Serial.print("L");
//              Serial.print(L_CurrPos);
//              Serial.print("LT");
//              Serial.print(L_TargPos);
//              Serial.print(" L1");
//              Serial.print(L_OutputPos);
//              Serial.print(" Lo");
//              Serial.print(OutputL);
//              Serial.print(" ");
//              Serial.print("  R");
//              Serial.print(R_CurrPos);
//              Serial.print(" RT");
//              Serial.print(R_TargPos);
//              Serial.print(" R1");
//              Serial.print(R_OutputPos);
//              Serial.print(" Ro");
//              Serial.print(OutputR);
//              Serial.print(" CD");
//              Serial.print(CurrDiff);
//              Serial.print(" TD");
//              Serial.print(TargDiff);
//              Serial.print(" oD");
//              Serial.println(getDistanceLeft());

	}
  checkPosReached();
}



void disableControlLoop(void)
{
  L_MotorEn = false;
  R_MotorEn = false;
  motorControl(0,L_MotorPins);
  motorControl(0,R_MotorPins);
  L_TargPos = L_ENC.read();
  R_TargPos = R_ENC.read();
  Move = false;
  Rotate = false;
}



void checkPosReached(void)
{
    if(L_reachedPos()&& R_reachedPos()&& L_MotorEn==true)
  {
//    Serial.println("Reached position");
    disableControlLoop();
    at_position = true;
  }
}

bool get_at_position(void){
  return at_position;
}


bool L_reachedPos(void)
{
   if(abs(L_TargPos-L_CurrPos)<=POS_TOLERANCE && time_toggle_L == 0){
    timeElapsed_L = 0;
    time_toggle_L = 1;
    return 0;
   }
   else if (abs(L_TargPos-L_CurrPos)<=POS_TOLERANCE && time_toggle_L ==1 && timeElapsed_L>250){
     time_toggle_L = 0;
     return 1;
   }
   else
    return 0;
}


bool R_reachedPos(void)
{
  if(abs(R_TargPos-R_CurrPos)<=POS_TOLERANCE && time_toggle_R == 0){
    timeElapsed_R = 0;
    time_toggle_R = 1;
    return 0;
   }
   else if (abs(R_TargPos-R_CurrPos)<=POS_TOLERANCE && time_toggle_R ==1 && timeElapsed_R>250){
     time_toggle_R = 0;
     return 1;
   }
   else
    return 0;
}

bool get_movement(void){
  if(L_MotorEn == false && R_MotorEn == false){

    return 0;

  }
  else{
    return 1;
  }

}
