#ifndef EQUATIONSOLUTION_H
#define EQUATIONSOLUTION_H

#include "OtgType3Com.h"

bool Step1_PosLinNegTrap(MotionState &ms, bool inverseFlag);

bool Step1_PosLinNegTri(MotionState &ms, bool inverseFlag);

bool Step1_PosTrapNegTri(MotionState &ms, bool inverseFlag);

bool Step1_PosTrapZeroNegTri(MotionState &ms, bool inverseFlag);

bool Step1_PosTriZeroNegTri(MotionState &ms, bool inverseFlag);

bool Step1_PosTrapZeroNegTrap(MotionState &ms, bool inverseFlag);

bool Step1_PosTriZeroNegTrap(MotionState &ms, bool inverseFlag);

bool Step1_PosTrapNegTrap(MotionState &ms, bool inverseFlag);

bool Step1_PosTriNegTrap(MotionState &ms, bool inverseFlag);

bool Step1_PosTriNegTri(MotionState &ms, bool inverseFlag);

#endif // EQUATIONSOLUTION_H
