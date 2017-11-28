/*
 PBencoder.cpp
 Created: 25/11/2017 
 Author: Harry Gee 
 copyright Agilic Ltd
 */ 

#include <Arduino.h>
#include "PBncoder.h"
#include "Movement.h"


// INTERRUPT HANDLERS
void R_EncHandler(void)
{       
	if(R_ENC.dirPinState())
		R_ENC.increment();
	else
		R_ENC.decrement();
        //Serial.print("  ");
        //Serial.println(R_ENC.read());
}

void L_EncHandler(void)
{
	if(L_ENC.dirPinState())
		L_ENC.decrement();
	else
		L_ENC.increment();
        //Serial.println(L_ENC.read());
}

// ENCODER class
ENCODER::ENCODER()
{
	
}

void ENCODER::init(int interruptPin, int dirPin, void (*interruptHandler)(void))
{
	_interruptNum = digitalPinToInterrupt(interruptPin);
	_dirPin = dirPin;
	
	attachInterrupt(_interruptNum,interruptHandler,RISING);
	pinMode(_dirPin,INPUT);
}

double ENCODER::read(void)
{
	return posCount;
}

void ENCODER::write(double val)
{
	posCount = val;
}

void ENCODER::increment(void)
{
	posCount++;
}

void ENCODER::decrement(void)
{
	posCount--;
}

bool ENCODER::dirPinState(void)
{
	return (bool) digitalRead(_dirPin);
}

void ENCODER::reset(void)
{
	posCount = 0;
}

double ENCODER::readVelocity(void)
{
	return tksPerPeriod;
}



void L_calcVel(void)
{
	static double prevPos = 0;
	static double prevTime = 0;
	double currPos, currTime;
	double calc;
	
	currTime = millis();
	currPos = L_ENC.read();
	
	// handle 0/period
	calc = (currPos - prevPos);
	if(calc == 0)
		L_ENC.tksPerPeriod = 0;
	else
		L_ENC.tksPerPeriod = (double) (((currPos - prevPos) / (currTime - prevTime))) * 1000;
		
	prevPos = currPos;
	prevTime = currTime;
}

void R_calcVel(void)
{
	static double prevPos = 0;
	static double prevTime = 0;
	double currPos, currTime;
	double calc;
	
	currTime = millis();
	currPos = R_ENC.read();
	
	// handle 0/period
	calc = (currPos - prevPos);
	if(calc == 0)
		R_ENC.tksPerPeriod = 0;
	else
		R_ENC.tksPerPeriod = (double) (((currPos - prevPos) / (currTime - prevTime))) * 1000;
		
	prevPos = currPos;
	prevTime = currTime;
}
