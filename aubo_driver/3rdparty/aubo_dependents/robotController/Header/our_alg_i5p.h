#ifndef OUR_ALG_H
#define OUR_ALG_H

using namespace std;
#define IKREAL_TYPE double
#define IkReal double
//#define NULL 0


#define ARM_DOF             6

struct Pos
{
    double x;
    double y;
    double z;
};

union cartesianPos_U
{
    Pos position;
    double positionVector[3];
};


struct Ori
{
    double w;
    double x;
    double y;
    double z;
};


union cartesianOri_U
{
    Ori orientation;
    double quaternionVector[4];
};

typedef struct
{
    cartesianPos_U cartPos;
    //cartesianOri_U cartOri;
    Ori orientation;
    //double eulerAngle[3];
    double jointpos[ARM_DOF];
}wayPoint_S;
#define RoadPoint wayPoint_S//waypoint.

enum aubo_robot_type
{
    aubo_i5,
    aubo_i7,
    aubo_i10_12,
    aubo_i10_14,
    aubo_i3,
    aubo_i5s
};
void getDhPara(double &A3, double &A4, double &D1, double &D2, double &D5, double &D6, double *D4=NULL);
bool setDhParaLen(aubo_robot_type robotType); //from server
void modifyDhParaLen(float da[6], float dd[6]);//diff = calibrated DhPara (from server) - getDhPara(client)

int ComputeIkLib(double jointpos[6], double R06[9], double positions[3], double jointResults[6][8]);
void ComputeFk_JYH(const double* joints, double *eetrans, double *eerot);

bool userCoordinateCalibLib(RoadPoint *waypointptr,char methods, double *eetrans, double *eerot, double *Porg, double &dotValue);
bool toolCoordinateCalibLib(double *rot, double *pos, int posWpNum, char poseCalib, double poseRot[3][9], double posePos[3][3], bool waypoint_check, double checkThr, double eetrans[3], double toolPoseRot[9]);
#endif
