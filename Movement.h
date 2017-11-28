/*
 * Movement.h
 *
 * Created: 25/02/2016 20:36:13
 *  Author: Olly
 *  Tolerance was 5
 */

#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#define MAX_SPEED	255


// DEFAULTS
#define POS_TOLERANCE 9

#define CLICKS_PER_CM   9.15
#define CLICKS_PER_DEG	1.06

#define CLICKS_TO_DIST(clicks) ((clicks)/(CLICKS_PER_CM))
#define DIST_TO_CLICKS(dist) ((dist)*(CLICKS_PER_CM))

#define CLICKS_TO_ANGLE(clicks) ((clicks)/(CLICKS_PER_DEG))
#define ANGLE_TO_CLICKS(rot) ((rot)*(CLICKS_PER_DEG))
//#define ANGLE_TO_CLICKS(rot) ((double)(rot)*(((float) 1.3) - (((float)(rot))/((float)1125))))

void movePos(int deltaPos, int speed);
void moveRotate(double deltaAngle, int speed);

void initPID(void);
void calcPID(void);
void target_diff(void);
void initMotors(void);
void motorControl(int speed, int* pins);


void checkPosReached(void);
bool L_reachedPos(void);
bool R_reachedPos(void);
bool get_movement(void);
void disableControlLoop(void);

int getDistanceLeft(void);
int getDistanceRight(void);

bool get_at_position(void);

extern double L_TargPos, L_CurrPos, L_OutputPos;
extern double R_TargPos, R_CurrPos, R_OutputPos;

extern bool L_MotorEn;
extern bool R_MotorEn;


extern int L_MotorPins[2];
extern int R_MotorPins[2];


#endif
