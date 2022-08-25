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
#ifndef OTGSTEP2_H
#define OTGSTEP2_H

#include "OtgType3Com.h"

//invalid nodes
#define Decision_2__022 Decision_2__000
#define Decision_2__035 Decision_2__000
#define Decision_2__039 Decision_2__000
#define Decision_2__045 Decision_2__000

//repeated nodes
#define Decision_2__040 Decision_2__030
#define Decision_2__042 Decision_2__096
#define Decision_2__062 Decision_2__070
#define Decision_2__063 Decision_2__071
#define Decision_2__064 Decision_2__072
#define Decision_2__092 Decision_2__037
#define Decision_2__093 Decision_2__038

bool Decision_2__001(MotionState &ms);
bool Decision_2__002(MotionState &ms);
bool Decision_2__003(MotionState &ms);
bool Decision_2__004(MotionState &ms);
bool Decision_2__005(MotionState &ms);
bool Decision_2__006(MotionState &ms);
bool Decision_2__007(MotionState &ms);
bool Decision_2__008(MotionState &ms);
bool Decision_2__009(MotionState &ms);
bool Decision_2__010(MotionState &ms);
bool Decision_2__011(MotionState &ms);
bool Decision_2__012(MotionState &ms);
bool Decision_2__013(MotionState &ms);
bool Decision_2__014(MotionState &ms);
bool Decision_2__015(MotionState &ms);
bool Decision_2__016(MotionState &ms);
bool Decision_2__017(MotionState &ms);
bool Decision_2__018(MotionState &ms);
bool Decision_2__019(MotionState &ms);
bool Decision_2__020(MotionState &ms);
bool Decision_2__021(MotionState &ms);
bool Decision_2__023(MotionState &ms);
bool Decision_2__024(MotionState &ms);
bool Decision_2__025(MotionState &ms);
bool Decision_2__026(MotionState &ms);
bool Decision_2__027(MotionState &ms);
bool Decision_2__028(MotionState &ms);
bool Decision_2__029(MotionState &ms);
bool Decision_2__030(MotionState &ms);
bool Decision_2__031(MotionState &ms);
bool Decision_2__032(MotionState &ms);
bool Decision_2__033(MotionState &ms);
bool Decision_2__034(MotionState &ms);
bool Decision_2__036(MotionState &ms);
bool Decision_2__037(MotionState &ms);
bool Decision_2__038(MotionState &ms);
bool Decision_2__041(MotionState &ms);
bool Decision_2__043(MotionState &ms);
bool Decision_2__044(MotionState &ms);
bool Decision_2__046(MotionState &ms);
bool Decision_2__047(MotionState &ms);
bool Decision_2__048(MotionState &ms);
bool Decision_2__049(MotionState &ms);
bool Decision_2__050(MotionState &ms);
bool Decision_2__051(MotionState &ms);
bool Decision_2__052(MotionState &ms);
bool Decision_2__053(MotionState &ms);
bool Decision_2__054(MotionState &ms);
bool Decision_2__055(MotionState &ms);
bool Decision_2__056(MotionState &ms);
bool Decision_2__057(MotionState &ms);
bool Decision_2__058(MotionState &ms);
bool Decision_2__059(MotionState &ms);
bool Decision_2__060(MotionState &ms);
bool Decision_2__061(MotionState &ms);
bool Decision_2__065(MotionState &ms);
bool Decision_2__066(MotionState &ms);
bool Decision_2__067(MotionState &ms);
bool Decision_2__068(MotionState &ms);
bool Decision_2__069(MotionState &ms);
bool Decision_2__070(MotionState &ms);
bool Decision_2__071(MotionState &ms);
bool Decision_2__072(MotionState &ms);
bool Decision_2__073(MotionState &ms);
bool Decision_2__074(MotionState &ms);
bool Decision_2__075(MotionState &ms);
bool Decision_2__076(MotionState &ms);
bool Decision_2__077(MotionState &ms);
bool Decision_2__078(MotionState &ms);
bool Decision_2__079(MotionState &ms);
bool Decision_2__080(MotionState &ms);
bool Decision_2__081(MotionState &ms);
bool Decision_2__082(MotionState &ms);
bool Decision_2__083(MotionState &ms);
bool Decision_2__084(MotionState &ms);
bool Decision_2__085(MotionState &ms);
bool Decision_2__086(MotionState &ms);
bool Decision_2__087(MotionState &ms);
bool Decision_2__088(MotionState &ms);
bool Decision_2__089(MotionState &ms);
bool Decision_2__090(MotionState &ms);
bool Decision_2__091(MotionState &ms);
bool Decision_2__094(MotionState &ms);
bool Decision_2__095(MotionState &ms);
bool Decision_2__096(MotionState &ms);
bool Decision_2__097(MotionState &ms);


void VToVMax_PosTrapStep(MotionState &ms);
void VToVMin_NegTrapStep(MotionState &ms);
void VToVMin_NegTriStep(MotionState &ms);
void VToVMax_PosTriStep(MotionState &ms);

typedef double otgDataType;
typedef struct
{
    bool PTrapZPTriA1Flag;
    bool PTriZPTrapA1Flag;
    char PTriZPTriA1Flag;
    char reserved;
    otgDataType PTrapZPTriA1;
    otgDataType PTriZPTrapA1;
    otgDataType PTriZPTriA1;
} pxzpx_info_S;

typedef struct
{
    otgDataType PTriHNLA1; //saved at 43,72 ... for PtHNL.
    otgDataType PTriHNLA2; //saved at 44y,73y...for PTHNL.
    otgDataType PLHPTriA1;
    otgDataType PLHPTriA2;
    otgDataType NLHPTriA1;
    otgDataType NLHPTriA2;
} hld_lin_info_S;

extern pxzpx_info_S pxzpx_info;
extern hld_lin_info_S hld_lin_info;

bool step2Start(MotionState &ms);
extern const decisionFuncPtr Step2DecisionFuncPtr[98];
extern const int DecisionStep2[];
#endif // OTGSTEP2_H
