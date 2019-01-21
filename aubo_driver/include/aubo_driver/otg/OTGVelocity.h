#ifndef OTGVELOCITY_H
#define OTGVELOCITY_H
#include "OTGStep1.h"
#include "OTGStep2.h"
#include "OtgType3Com.h"

bool Velocity_Solution_PosTrap(MotionState &ms);
bool Velocity_Solution_PosTri(MotionState &ms);
bool Velocity_Solution_PosLinHldNegLin(MotionState &ms);
bool Velocity_Solution_NegLinHldNegLin(MotionState &ms);

bool VelocityStep1(MotionState &ms);
bool VelocityStep2(MotionState &ms);

const VelocitySolution VelocitySolutionFunc[] =
{
    Velocity_Solution_PosTrap,
    Velocity_Solution_PosTri,
    Velocity_Solution_PosLinHldNegLin,
    Velocity_Solution_NegLinHldNegLin
};

#endif // OTGVELOCITY_H
