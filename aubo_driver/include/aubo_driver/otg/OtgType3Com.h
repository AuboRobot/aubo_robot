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
#ifndef OTG_TYPE3_COM_H
#define OTG_TYPE3_COM_H
//*******************************************************************************************
// Include files
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

//#define STEP2_TEST

#define VARIANTB_NODES_NUM 9

#define SEG_NUM 13

#define OTG_INFINITY ((double)1.0e100)

#define RML_INPUT_VALUE_EPSILON ((double)1.0e-10)

#define RML_VALID_SOLUTION_EPSILON ((double)1.0e-10)

#define MAXIMAL_NO_OF_POLYNOMIALS  7

#define RML_MAX_EXECUTION_TIME ((double)1e10)

#define RML_ADDITIONAL_RELATIVE_POSITION_ERROR_IN_CASE_OF_EQUALITY ((double)1e-7)

#define RML_ADDITIONAL_ABSOLUTE_POSITION_ERROR_IN_CASE_OF_EQUALITY ((double)1e-7)

#define POSITIVE_ZERO ((double)1.0e-50)

#define ABSOLUTE_PHASE_SYNC_EPSILON ((double)1.0e-6)

#define RELATIVE_PHASE_SYNC_EPSILON ((double)1.0e-3)

#define RML_INFINITY ((double)1.0e100)

#define RML_POSITION_EXTREMS_TIME_EPSILON ((double)1.0e-4)

#define PHASE_SYNC_COLLINEARITY_REL_EPSILON ((double)1.0e-2)

#define Sign(A) ( ((double)(A) < 0.0)?(-1):(1) )

#define FSign(A) ( ((double)(A) < 0.0)?(-1.0):(1.0) )

#define pow2(A)                         ((A)*(A))

#define Equation_Solution_EPSILON ((double)1.0e-5)

inline double RMLSqrt(const double &Value)
{
    return( ( Value <= 0.0 ) ? ( POSITIVE_ZERO ) : ( sqrt( Value ) ) );
}


#define IsInputEpsilonEqual(A,B) ((bool)((fabs((double)A - (double)B) <= RML_INPUT_VALUE_EPSILON)?(true):(false)))


//  ----------------------------------------------------------
inline bool IsEpsilonEquality(      const double &Value1
                                ,   const double &Value2
                                ,   const double &Epsilon)
{
    return(fabs(Value1 - Value2) <= Epsilon);
}

enum Step1_Profile 
{
    Step1_Undefined=0,
    Step1_Profile_PosLinHldNegLin=1,
    Step1_Profile_PosTrapZeroNegTri=2,
    Step1_Profile_NegTrapZeroPosTri=3,
    Step1_Profile_PosTriZeroNegTri=4,
    Step1_Profile_NegTriZeroPosTri=5,
    Step1_Profile_PosTrapZeroNegTrap=6,
    Step1_Profile_NegTrapZeroPosTrap=7,
    Step1_Profile_PosTriZeroNegTrap=8,
    Step1_Profile_NegTriZeroPosTrap=9,
    Step1_Profile_PosTrapNegTri=10,
    Step1_Profile_NegTrapPosTri=11,
    Step1_Profile_PosTrapNegTrap=12,
    Step1_Profile_NegTrapPosTrap=13,
    Step1_Profile_PosTriNegTrap=14,
    Step1_Profile_NegTriPosTrap=15,
    Step1_Profile_PosTriNegTri=16,
    Step1_Profile_NegTriPosTri=17,
    Step1_Profile_NegLinPosTri=18,
    Step1_Profile_PosLinNegTri=19,
    Step1_Profile_NegLinPosTrap=20,
    Step1_Profile_PosLinNegTrap=21
    };

enum Step2_Profile 
{    
    Step2_Profile_PosTrapZeroNegTri = 0,
    Step2_Profile_NegTrapZeroPosTri,
    Step2_Profile_PosTriZeroNegTri,
    Step2_Profile_NegTriZeroPosTri,
    Step2_Profile_PosTrapZeroNegTrap,
    Step2_Profile_NegTrapZeroPosTrap,
    Step2_Profile_PosTriZeroNegTrap,
    Step2_Profile_NegTriZeroPosTrap,
    Step2_Profile_PosTrapZeroPosTri,
    Step2_Profile_NegTrapZeroNegTri,
    Step2_Profile_PosTriZeroPosTri, //10
    Step2_Profile_NegTriZeroNegTri,
    Step2_Profile_PosTriZeroPosTrap,
    Step2_Profile_NegTriZeroNegTrap,
    Step2_Profile_PosTrapZeroPosTrap,
    Step2_Profile_NegTrapZeroNegTrap,
    Step2_Profile_PosTrapHldNegLin,
    Step2_Profile_NegTrapHldPosLin,
    Step2_Profile_PosTriHldNegLin,
    Step2_Profile_NegTriHldPosLin,
    Step2_Profile_NegLinHldPosTrap, //20
    Step2_Profile_PosLinHldNegTrap,
    Step2_Profile_NegLinHldPosTri,
    Step2_Profile_PosLinHldNegTri,
    Step2_Profile_PosLinHldPosTri,
    Step2_Profile_NegLinHldNegTri,
    Step2_Profile_PosLinHldPosTrap,
    Step2_Profile_NegLinHldNegTrap,
    Step2_Undefined //28
};

enum Velocity_Profile
{
    Velocity_Profile_PosTrap = 0,
    Velocity_Profile_NegTrap = 1,
    Velocity_Profile_PosTri = 2,
    Velocity_Profile_NegTri = 3,
    Velocity_Profile_PosLinHldNegLin = 4,
    Velocity_Profile_NegLinHldPosLin = 5,
    Velocity_Profile_NegLinHldNegLin = 6,
    Velocity_Profile_PosLinHldPosLin = 7,
    Velocity_Undefined,
};
struct MotionState
{
    //common para
    double CurrentPosition;
    double TargetPosition;
    double CurrentVelocity;
    double TargetVelocity;
    double CurrentAcceleration;
    double MaxVelocity;
    double OldPosition;
    double OldAcceleration;
    double MaxAcceleration;
    double MaxJerk;
    double TotalTime;  //variantB+step1 (but changed by step2)
    int segmentCount; //variantB+step1 or variantB+step2    
    double segmentTime[SEG_NUM];
    double segmentAcceleration[SEG_NUM];
    double segmentVelocity[SEG_NUM];
    double segmentPosition[SEG_NUM];
    double ReservedValue[SEG_NUM];
    int IntermediateInversion;
    double SynchronizationTime;
    double ControlCycle;
    Step1_Profile AppliedProfile;
    Step2_Profile AppliedProfile2;
    Velocity_Profile VelocityProfile;
	
    //variant-B para for step2
    int segmentBCount;
    int InvB;
    double BP;
    double BV;
    double BA;
    double BPt;
    double BVt;
    double TotalTimeB;

    //step2 segment-para
    int step2SegmentParaCalc;

    //step1 para
    bool Step1Finished;
    bool FirstIeration;
    bool reduced;
    short reseved;
	
    int currSampleIdx;
    int currSecNum;
};

class JointTrajectoryInput
{
public:
    JointTrajectoryInput(int numDOF, double cc)
    {
        numberOfDOFs = numDOF;
        controlCycle = cc;
        currentPosition.resize(numDOF);
        targetPosition.resize(numDOF);
        currentVelocity.resize(numDOF);
        targetVelocity.resize(numDOF);
        currentAcceleration.resize(numDOF);
        maxVelocity.resize(numDOF);
        maxAcceleration.resize(numDOF);
        maxJerk.resize(numDOF);

    };
    ~JointTrajectoryInput(){};

    std::vector<double> currentPosition;
    std::vector<double> targetPosition;
    std::vector<double> currentVelocity;
    std::vector<double> targetVelocity;
    std::vector<double> currentAcceleration;
    std::vector<double> maxVelocity;
    std::vector<double> maxAcceleration;
    std::vector<double> maxJerk;
    int numberOfDOFs;
    double controlCycle;
};

class JointTrajectoryOutput
{
public:
    JointTrajectoryOutput(int numDOF)
    {
        numberOfDOFs = numDOF;
        newPosition.resize(numDOF);
        newVelocity.resize(numDOF);
        newAcceleration.resize(numDOF);
    }
    ~JointTrajectoryOutput(){};
    std::vector<double> newPosition;
    std::vector<double> newVelocity;
    std::vector<double> newAcceleration;
    int numberOfDOFs;
    double leftTime;
};

struct MotionProperty
{
    double uFactor;
    double TargetPosition;
    double CurrentVelocity;
    double TargetVelocity;
    double CurrentAcceleration;
    double MaxVelocity;
    double MaxAcceleration;
    double MaxJerk;
};


typedef bool (*decisionFuncPtr)(MotionState &ms);
typedef double (*pErrFuncPtr)(MotionState &ms, const double &xx);
typedef bool (*SolutionFuncPtr)(MotionState &ms);
typedef bool (*VelocitySolution)(MotionState &ms);
bool FuncABK(MotionState &ms, pErrFuncPtr pf,double x1, double x2, double &IntermediateVariable);
void NegateStep(MotionState &ms);
void AToAMaxStep(MotionState &ms);
void AToZeroStep(MotionState &ms);
void VToVMin_PosLinStep(MotionState &ms);
void VToVMin_PosLinNegLinStep(MotionState &ms);
void VToVMin_PosLinHldStep(MotionState &ms);
void VToVMin_PosLinHldNegLinStep(MotionState &ms);
int getVelcPosFromAccCurve(MotionState &ms,double *pBuf,double *vBuf,double *aBuf,int num);
extern bool testlevel;
#endif // OTG_TYPE3_COM_H
