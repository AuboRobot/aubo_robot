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
#ifndef OTGSTEP1_H
#define OTGSTEP1_H
#include "OtgType3Com.h"
#include "EquationSolutionStep1.h"

bool Decision_1A__001(const MotionState &ms);
bool Decision_1A__002(const MotionState &ms);
bool Decision_1A__003(const MotionState &ms);
bool Decision_1A__004(const MotionState &ms);
bool Decision_1A__005(const MotionState &ms);
bool Decision_1A__006(const MotionState &ms);
bool Decision_1A__007(const MotionState &ms);
bool Decision_1A__008(const MotionState &ms);
bool Decision_1A__009(const MotionState &ms);
bool Decision_1A__010(const MotionState &ms);
bool Decision_1A__011(const MotionState &ms);
bool Decision_1A__012(const MotionState &ms);
bool Decision_1A__013(const MotionState &ms);
bool Decision_1A__014(const MotionState &ms);
bool Decision_1A__015(const MotionState &ms);
bool Decision_1A__016(const MotionState &ms);
bool Decision_1A__017(const MotionState &ms);
bool Decision_1A__018(const MotionState &ms);
bool Decision_1A__019(const MotionState &ms);
bool Decision_1A__020(const MotionState &ms);
bool Decision_1A__021(const MotionState &ms);
bool Decision_1A__022(const MotionState &ms);
bool Decision_1A__023(const MotionState &ms);
bool Decision_1A__024(const MotionState &ms);
bool Decision_1A__025(const MotionState &ms);
bool Decision_1A__026(const MotionState &ms);
bool Decision_1A__027(const MotionState &ms);
bool Decision_1A__028(const MotionState &ms);
bool Decision_1A__029(const MotionState &ms);
bool Decision_1P__001(const MotionState &ms);
bool Decision_1P__002(MotionState &ms);
bool Decision_1Q__001(const MotionState &ms);
bool Decision_1Q__002(const MotionState &ms);
bool Decision_1A__030(const MotionState &ms);
bool Decision_1A__031(const MotionState &ms);
bool Decision_1A__032(MotionState &ms);
bool Decision_1A__033(const MotionState &ms);
bool Decision_1A__034(const MotionState &ms);
bool Decision_1A__035(const MotionState &ms);
bool Decision_1A__036(const MotionState &ms);
bool Decision_1A__037(const MotionState &ms);
bool Decision_1A__038(const MotionState &ms);
bool Decision_1A__039(const MotionState &ms);
bool Decision_1A__040(const MotionState &ms);
bool Decision_1A__041(const MotionState &ms);
bool Decision_1A__042(const MotionState &ms);
bool Decision_1A__043(const MotionState &ms);
bool Decision_1A__044(const MotionState &ms);
bool Decision_1A__045(const MotionState &ms);
bool Decision_1A__046(const MotionState &ms);
bool Decision_1A__047(const MotionState &ms);
bool Decision_1A__048(const MotionState &ms);
bool Decision_1A__049(const MotionState &ms);

bool step1Start(MotionState &ms);
extern const decisionFuncPtr Step1DecisionFuncPtr[54];

extern const int DecisionStep1[108];

#endif // OTGSTEP1_H
