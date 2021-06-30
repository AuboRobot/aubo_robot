#ifndef OTGNEWSLIB_H
#define OTGNEWSLIB_H

#include "OTGStep1.h"
#include "OTGStep2.h"
#include "OTGVelocity.h"

typedef struct
{
    MotionState ms[20];
    char step1_dof_index;
    bool condChanged;
    int freedomNum;
    char reserved;
}MotionInfo_S;

extern MotionInfo_S motionInfo;


void otgNewDebugInfo();
double getLeftMoveTime();

bool otgPostionModeParameterUpdate(JointTrajectoryInput jti);
bool otgVelocityModeParameterUpdate(JointTrajectoryInput jti);

int otgPositionModeResult(int numberOfTimes, JointTrajectoryOutput &jto);
int otgVelocityModeResult(int numberOfTimes, JointTrajectoryOutput &jto);

#endif // OTGNEWSLIB_H
