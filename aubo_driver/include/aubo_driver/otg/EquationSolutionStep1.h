/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
