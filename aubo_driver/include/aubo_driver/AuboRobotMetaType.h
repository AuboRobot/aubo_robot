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
#ifndef AUBOROBOTMETATYPE_H
#define AUBOROBOTMETATYPE_H

#include <iostream>
#include <stdint.h>
#include "robotiomatetype.h"



/**
 * General types
 */
typedef  uint8_t     boolean;
typedef  int8_t      int8;
typedef  int16_t     int16;
typedef  int32_t     int32;
typedef  uint8_t     uint8;
typedef  uint16_t    uint16;
typedef  uint32_t    uint32;
typedef  int64_t     int64;
typedef  uint64_t    uint64;
typedef  float       float32;
typedef  double      float64;

#define  PACKED  __attribute__((__packed__))


#ifdef __cplusplus
extern "C" {
#endif


/**
 * 命名空间  aubo_robot_namespace
 **/
namespace  aubo_robot_namespace
{

enum {
    ARM_DOF = 6,   //机械臂关节数
};


/**
 * 机械臂类型
 **/
typedef enum
{
    ROBOT_I5    = 0,
    ROBOT_I7    = 1,
    ROBOT_I10_12= 2,
    ROBOT_I3S   = 3,
    ROBOT_I3    = 4,
    ROBOT_I5S   = 5,
    ROBOT_I5L   = 6,
    ROBOT_I10S  = 7
}RobotType;


/**
  * DH参数
  **/
typedef struct
{
    double A3;
    double A4;
    double D1;
    double D2;
    double D5;
    double D6;
}RobotDhPara;








typedef enum
{
    RobotControllerErr_MotionCfgErr,    //only this is recoverable.
    RobotControllerErr_OverspeedProtect,
    RobotControllerErr_IkFailure,
    RobotControllerErr_OnlineTrajErr,
    RobotControllerErr_OfflineTrajErr,
    RobotControllerErr_StatusException,
}RobotControllerErrorCode;


typedef enum {
    RUN_TO_READY_POSITION,
    RUN_PROJECT,
    PAUSE_PROJECT,
    CONTINUE_PROJECT,
    SLOWLY_STOP_PROJECT,
    LOAD_PROJECT,
    ENTER_SAFEGUARD_MODE_BY_DI_EXTERNAL_SAFEGUARD_STOP,
    RELEASE_SAFEGUARD_MODE_IN_AUTOMATIC_MODE,
    RELEASE_SAFEGUARD_MODE_IN_MANUAL_MODE,
    MANUALLY_RELEASE_SAFEGUARD_MODE_PROMPT,
    ENTER_SAFEGUARD_MODE_BY_TRI_STATE_SWITCH,
    RELEASE_SAFEGUARD_MODE_BY_TRI_STATE_SWITCH,
    ENTER_REDUCE_MODE,
    RELEASE_REDUCE_MODE,
    REMOTE_CLEAR_ALARM_SIGNAL,
    PROJECT_STARTUP_IS_SAFETY
}InterfaceBoardSafeIoEventCode;





/**
 * 机械臂诊断信息
 **/
typedef struct PACKED
{
    uint8   armCanbusStatus;            // CAN通信状态:0x01~0x80：关节CAN通信错误（每个关节占用1bit） 0x00：无错误
    float   armPowerCurrent;            // 机械臂48V电源当前电流
    float   armPowerVoltage;            // 机械臂48V电源当前电压
    bool    armPowerStatus;             // 机械臂48V电源状态（开、关）
    char    contorllerTemp;             // 控制箱温度
    uint8   contorllerHumidity;         // 控制箱湿度
    bool    remoteHalt;                 // 远程关机信号
    bool    softEmergency;              // 机械臂软急停
    bool    remoteEmergency;            // 远程急停信号
    bool    robotCollision;             // 碰撞检测位
    bool    forceControlMode;           // 机械臂进入力控模式标志位
    bool    brakeStuats;                // 刹车状态
    float   robotEndSpeed;              // 末端速度
    int     robotMaxAcc;                // 最大加速度
    bool    orpeStatus;                 // 上位机软件状态位
    bool    enableReadPose;             // 位姿读取使能位
    bool    robotMountingPoseChanged;   // 安装位置状态
    bool    encoderErrorStatus;         // 磁编码器错误状态
    bool    staticCollisionDetect;      // 静止碰撞检测开关
    uint8   jointCollisionDetect;       // 关节碰撞检测 每个关节占用1bit 0-无碰撞 1-存在碰撞
    bool    encoderLinesError;          // 光电编码器不一致错误 0-无错误 1-有错误
    bool    jointErrorStatus;           // joint error status
    bool    singularityOverSpeedAlarm;  // 机械臂奇异点过速警告
    bool    robotCurrentAlarm;          // 机械臂电流错误警告
    uint8   toolIoError;                // tool error
    bool    robotMountingPoseWarning;   // 机械臂安装位置错位（只在力控模式下起作用）
    uint16  macTargetPosBufferSize;     // mac缓冲器长度
    uint16  macTargetPosDataSize;       // mac缓冲器有效数据长度
    uint8   macDataInterruptWarning;    // mac数据中断

}RobotDiagnosis;


typedef struct PACKED
{
    uint8 orpePause;                    // 上位机暂停状态
    uint8 orpeStop;                     // 上位机停止状态
    uint8 orpeError[16];                // 上位机错误
    uint8 systemEmergencyStop;          // 解除系统紧急停止输出信号
    uint8 reducedModeError;             // 解除缩减错误
    uint8 safetyguardResetSucc;         // 防护重置成功
}OrpeSafetyStatus;


typedef struct PACKED
{
    uint16  robotReducedConfigJointSpeed[6];    //缩减配置 关节速度限制
    uint32  robotReducedConfigTcpSpeed;         //缩减配置 TCP速度限制
    uint32  robotReducedConfigTcpForce;         //缩减配置 TCP力（暂定为碰撞等级）
    uint32  robotReducedConfigMomentum;         //缩减配置 动量
    uint32  robotReducedConfigPower;            //缩减配置 功率
    uint8   robotSafeguradResetConfig;          //防护重 置设置
    uint8   robotOperationalModeConfig;         //操作模式设置
}RobotSafetyConfig;



/**
 * @brief IO的类型枚举
 *
 **/
typedef enum
{
    RobotBoardControllerDI,    // 接口板控制器DI(数字量输入)　　　只读(一般系统内部使用)
    RobotBoardControllerDO,    // 接口板控制器DO(数字量输出)     只读(一般系统内部使用)
    RobotBoardControllerAI,    // 接口板控制器AI(模拟量输入)　   只读(一般系统内部使用)
    RobotBoardControllerAO,    // 接口板控制器AO(模拟量输出)　　　只读(一般系统内部使用)

    RobotBoardUserDI,          // 接口板用户DI(数字量输入)　　   可读可写
    RobotBoardUserDO,          // 接口板用户DO(数字量输出)      可读可写
    RobotBoardUserAI,          // 接口板用户AI(模拟量输入)      可读可写
    RobotBoardUserAO,          // 接口板用户AO(模拟量输出)      可读可写

    RobotToolDI,               // 工具端DI
    RobotToolDO,               // 工具端DO
    RobotToolAI,               // 工具端AI
    RobotToolAO,               // 工具端AO

}RobotIoType;



/**
 * IO类型
 **/
typedef enum
{
    IO_IN = 0,        //输入
    IO_OUT            //输出
}ToolIOType;


/**
 * 工具的电源类型
 **/
typedef enum
{
    OUT_0V  = 0,
    OUT_12V = 1,
    OUT_24V = 2
}ToolPowerType;


typedef enum
{
    RobotToolIoTypeDI=RobotToolDI,      //工具端DI
    RobotToolIoTypeDO=RobotToolDO       //工具端DO
}RobotToolIoType;


typedef  enum              //ＩＯ状态
{
    IO_STATUS_INVALID = 0, //有效
    IO_STATUS_VALID        //无效
}IO_STATUS;

typedef enum
{
    TOOL_DIGITAL_IO_0 = 0,
    TOOL_DIGITAL_IO_1 = 1,
    TOOL_DIGITAL_IO_2 = 2,
    TOOL_DIGITAL_IO_3 = 3

}ToolDigitalIOAddr;



/**
  * 综合描述一个IO
  **/
typedef struct PACKED
{
    char        ioId[32];      //IO-ID 目前未使用
    RobotIoType ioType;        //IO类型
    char        ioName[32];    //IO名称
    int         ioAddr;        //IO地址
    double      ioValue;       //IO状态
}RobotIoDesc;


//接口板数字量数据
typedef struct
{
    uint8 addr ;
    uint8 value;
    uint8 type;
}RobotDiagnosisIODesc;

//接口板模拟量数据
typedef struct
{
    uint8  addr ;
    float  value;
    uint8 type;
}RobotAnalogIODesc;


typedef struct PACKED
{
    ToolIOType ioType;
    uint8      ioData;
}ToolDigitalStatus;


typedef enum {
    RobotToolNoError      = 0, //无错误
    RobotToolOverVoltage  = 1, //过压
    RobotToolUnderVoltage = 2, //欠压
    RobotToolOVerTemp     = 3, //过温
    RobotToolCanBusError  = 4  //CAN总线错误
}RobotToolErrorCode;






/**
 * @brief 机械臂启动完成状态
 */
enum ROBOT_SERVICE_STATE
{
    ROBOT_SERVICE_READY = 0,
    ROBOT_SERVICE_STARTING,
    ROBOT_SERVICE_WORKING,
    ROBOT_SERVICE_CLOSING,
    ROBOT_SERVICE_CLOSED,
    ROBOT_SETVICE_FAULT_POWER,
    ROBOT_SETVICE_FAULT_BRAKE,
    ROBOT_SETVICE_FAULT_NO_ROBOT
};


enum ROBOT_INIT_PHASE{
    ROBOT_INIT_PHASE_READY=0,
    ROBOT_INIT_PHASE_HANDSHAKE,
    ROBOT_INIT_PHASE_SET_POWER,
    ROBOT_INIT_PHASE_SET_BRAKE,
    ROBOT_INIT_PHASE_SET_COLLSION_CLASS,
    ROBOT_INIT_PHASE_SET_OTHER_CMD,
    ROBOT_INIT_PHASE_WORKING
};


typedef enum
{
    RobotModeSimulator,         // 机械臂仿真模式
    RobotModeReal               // 机械臂真实模式
}RobotWorkMode;



/**
 * @brief 机械臂运动控制命令
 */
typedef enum
{
    RobotMoveStop     = 0,       // 停止
    RobotMovePause    = 1,       // 暂停
    RobotMoveContinue = 2,       // 继续
}RobotMoveControlCommand;



/**
 * @brief 机械臂控制命令
 */
enum RobotControlCommand
{
    RobotRelease           = 0,    // 释放刹车
    RobotBrake             = 1,    // 刹车
    OverspeedWarning       = 2,    // 拖动示教速度过快报警
    OverspeedRecover       = 3,    // 解除拖动过速报警
    DisableForceControl    = 4,    // 失能力控
    EnableForceControl     = 5,    // 使能力控
    OrpeOpen               = 6,    // 打开上位机软件
    OrpeClose              = 7,    // 关闭上位机软件
    EnableReadPose         = 8,    // 打开读取位姿
    DisableReadPose        = 9,    // 关闭读取位姿
    MountingPoseChanged    = 10,   // 安装位置已改变
    MountingPoseUnChanged  = 11,   // 安装位置未改变
    EnableStaticCollisionDetect    = 12, // 打开静止碰撞检测
    DisableStaticCollisionDetect   = 13, // 关闭静止碰撞检测
    ClearSingularityOverSpeedAlarm = 14, // 解除机械臂奇异点过速警告
    ClearRobotCurrentAlarm         = 15  // 解除机械臂电流错误警告
};





/**
 * @brief 位置信息
 **/
struct Pos
{
    double x;
    double y;
    double z;
};



/**
 * @brief 位置信息的共用体描述
 **/
union cartesianPos_U
{
    Pos position;
    double positionVector[3];
};



/**
 * @brief 姿态的四元素表示方法
 **/
struct Ori
{
    double w;
    double x;
    double y;
    double z;
};

/**
 * @brief 姿态的欧拉角表示方法
 **/
struct Rpy
{
    double rx;
    double ry;
    double rz;
};

typedef struct
{
    double jointPos[ARM_DOF];
}JointParam;


/**
  * @brief 描述关节的速度和加速度
  */
typedef struct
{
    double jointPara[ARM_DOF];
}JointVelcAccParam;


/**
 * 关节碰撞补偿（范围0.00~0.51度）
 **/
typedef struct
{
    double jointOffset[ARM_DOF];
}RobotJointOffset;


/**
 * @brief 机械臂的路点信息
 **/
typedef struct
{
    cartesianPos_U cartPos;            // 机械臂的位置信息(x,y,z)

    Ori            orientation;        // 机械臂姿态信息,四元素(w,x,y,z)

    double         jointpos[ARM_DOF];  // 机械臂关节角信息

}wayPoint_S;


/**
 *  @brief 描述运动属性中的偏移属性
 */
typedef struct
{
    bool  ena;                       // 是否使能偏移

    float relativePosition[3];       // 偏移量 x,y,z

    Ori   relativeOri;               // 姿态偏移

}MoveRelative;


typedef struct
{
    double minValue;
    double maxValue;

}RangeOfMotion;

/**
 * 关节运动范围
 */

typedef struct
{
    bool   enable;                        // 是否使能偏移

    RangeOfMotion rangeValues[ARM_DOF];    //运动范围

}JointRangeOfMotion;




/**
 * @brief 示教模式枚举
 **/
enum teach_mode
{
    NO_TEACH = 0,
    JOINT1,
    JOINT2,
    JOINT3,
    JOINT4,
    JOINT5,
    JOINT6,
    MOV_X,
    MOV_Y,
    MOV_Z,
    ROT_X,
    ROT_Y,
    ROT_Z
};


/**
 * @brief 运动轨迹枚举
 **/
enum move_track
{
    NO_TRACK = 0,

    //for moveJ and moveL
    TRACKING,

    //cartesian motion for moveP
    ARC_CIR,
    CARTESIAN_MOVEP,
    CARTESIAN_CUBICSPLINE,
    CARTESIAN_UBSPLINEINTP,
    CARTESIAN_GNUBSPLINEINTP,
    CARTESIAN_LOOKAHEAD,

    //joint motion for moveP
    JIONT_CUBICSPLINE,
    JOINT_UBSPLINEINTP,
    JOINT_GNUBSPLINEINTP,

    ARC,
    CIRCLE,
    ARC_ORI_ROTATED,
    CIRCLE_ORI_ROTATED,

    ORI_POSITION_ROTATE_CIRCUMFERENCE=101,
};













/**
 * @brief 工具姿态标定的方法枚举
 *
 */

enum ToolKinematicsOriCalibrateMathod
{
    ToolKinematicsOriCalibrateMathod_Invalid = -1,

    ToolKinematicsOriCalibrateMathod_xOxy,                   // 原点、x轴正半轴、x、y轴平面的第一象限上任意一点
    ToolKinematicsOriCalibrateMathod_yOyz,                   // 原点、y轴正半轴、y、z轴平面的第一象限上任意一点
    ToolKinematicsOriCalibrateMathod_zOzx,                   // 原点、z轴正半轴、z、x轴平面的第一象限上任意一点
    ToolKinematicsOriCalibrateMathod_TxRBz_TxyPBzAndTyABnz,  // 工具x轴平行反向于基坐标系z轴; 工具xOy平面平行于基坐标系z轴、工具y轴与基坐标系负z轴夹角为锐角
    ToolKinematicsOriCalibrateMathod_TyRBz_TyzPBzAndTzABnz,  // 工具y轴平行反向于基坐标系z轴; 工具yOz平面平行于基坐标系z轴、工具z轴与基坐标系负z轴夹角为锐角
    ToolKinematicsOriCalibrateMathod_TzRBz_TzxPBzAndTxABnz,  // 工具z轴平行反向于基坐标系z轴; 工具zOx平面平行于基坐标系z轴、工具x轴与基坐标系负z轴夹角为锐角

    ToolKinematicsOriCalibrateMathodCount
};



/**
 * 该结构体描述工具惯量
 *
 * 注：该结构体属于冗余数据类型，使用时把所有参数都设置为{0,0,0,0,0,0}．
 **/
typedef struct
{
    double xx;
    double xy;
    double xz;
    double yy;
    double yz;
    double zz;
}ToolInertia;


/**
 * 工具动力学参数描述
 *
 * 注意：
 * 　　　机械臂上电之前，安装在机器人末端的工具发生改变时都需要重新设置工具的动力学参数．
 * 　　　一般情况下，工具的动力学参数和运动学参数是需要一起设置的；
 * 切记：
 * 　　　该参数如果不能正确设置会影响机械臂的安全等级和运动轨迹．
 **/
typedef struct
{
    double      positionX;    // 工具重心的X坐标

    double      positionY;    // 工具重心的Y坐标

    double      positionZ;    // 工具重心的Z坐标

    double      payload;      // 工具重量

    ToolInertia toolInertia;  // 工具惯量　预留　使用是全部设置为0

}ToolDynamicsParam;


/** 工具描述　工具的运动学参数
 *
 *  该工具用于描述一个工具或者工具的运动学参数．
 */
typedef struct
{
    Pos        toolInEndPosition;     // 工具相对法兰盘的位置

    Ori        toolInEndOrientation;  // 工具相对法兰盘的姿态

}ToolInEndDesc;

typedef ToolInEndDesc ToolKinematicsParam;
typedef ToolInEndDesc RobotCameraCalib;


/**
  * 焊接摇摆结构体
  */
typedef struct
{
    bool   weaveEnable;
    int    weaveType;
    double weaveStep;
    double weaveAmplitude;
    double weaveHoldDistance;
    double weaveAngle;
}WeaveMove;


typedef struct PACKED{
    uint8 type;   //机械臂辨识参数类型
    uint8 length; //参数数据长度
    uint8 data[256];//参数实际数据
}RobotRecongnitionParam;


/**
 * 坐标系类型枚举
 **/
enum coordinate_refer
{
    BaseCoordinate = 0,  // 基座坐标系
    EndCoordinate,       // 用户坐标系
    WorldCoordinate      // 末端坐标系或工具坐标系
};


/**
 * 用户坐标系标定方法枚举
 *
 * 描述:3点标定坐标系　３个示教点的含义．
 **/
enum CoordCalibrateMathod
{
    Origin_AnyPointOnPositiveXAxis_AnyPointOnPositiveYAxis,            // 原点、x轴正半轴、y轴正半轴
    Origin_AnyPointOnPositiveYAxis_AnyPointOnPositiveZAxis,            // 原点、y轴正半轴、z轴正半轴
    Origin_AnyPointOnPositiveZAxis_AnyPointOnPositiveXAxis,            // 原点、z轴正半轴、x轴正半轴
    Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane,  // 原点、x轴正半轴、x、y轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOZPlane,  // 原点、x轴正半轴、x、z轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOZPlane,  // 原点、y轴正半轴、y、z轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOXPlane,  // 原点、y轴正半轴、y、x轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOXPlane,  // 原点、z轴正半轴、z、x轴平面的第一象限上任意一点
    Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOYPlane,  // 原点、z轴正半轴、z、y轴平面的第一象限上任意一点

    CoordTypeCount,
};

/**
 * 坐标系描述
 *
 * 该结构体描述一个坐标系。系统通过该结构体描述一个坐标系(基座坐标系, 用户坐标系, 末端坐标系或工具坐标系)。
 *
 * 坐标系分３种类型: 基座坐标系(BaseCoordinate);
 *                用户坐标系(WorldCoordinate);
 *                末端坐标系或工具坐标系(EndCoordinate);
 *
 * 定义:
 *      基座坐标系 是　根据机械臂基座建立的坐标系;
 *      用户坐标系 是  用户坐标系定义在工件上，在机器人动作允许范围内的任意位置，设定任意角度的X、Y、Z轴，原点位于机器人抓取的工件上，坐标系的方向根据客户需要任意定义。
 *      末端坐标系 是　安装在机器人末端的工具坐标系，原点及方向都是随着末端位置与角度不断变化的，该座标系实际是将基础座标系通过旋转及位移变化而来的；法兰盘是一个特殊的末端坐标系.
 *
 * 结构体参数描述：
 *      coordType    　　　　坐标系类型,描述坐标系属于那种类型
 * 　　　methods　　　 　　　　用户坐标系的标定方法　　　　　仅在coordType为用户坐标系(WorldCoordinate)时有效；
 * 　　　wayPointArray[3]　　标定用户坐标系的３个路点信息　仅在coordType为用户坐标系(WorldCoordinate)时有效；
 * 　　　toolDesc　　　　　　　末端工具描述　　　当coordType＝WorldCoordinate　表示标定用户坐标系时，安装在机器人末端的工具;
 *                                         当coordType＝EndCoordinate　 描述是哪个工具的坐标系
 *
 * 使用说明:
 *      基座坐标系
 *              coordType=BaseCoordinate
 * 　　　　　　　　其他参数默认
 *      用户坐标系
 *              coordType=WorldCoordinate
 * 　　　　　　　　methods　　　　　　　　为标定方法
 * 　　　　　　　　wayPointArray[3]　　　标定坐标系的３个路点
 * 　　　　　　　　toolDesc　　　　　　　　标定用户坐标系时，安装在机器人末端的工具
 * 　　　末端坐标系或工具坐标系
 *              coordType=EndCoordinate
 * 　　　　　　　　methods　　　　　　　　缺省，不需要设置
 * 　　　　　　　　wayPointArray[3]　　　缺省，不需要设置
 * 　　　　　　　　toolDesc　　　　　　　　机器人末端的工具
 *
 *　备注：
 * 　   法兰盘为特殊的工具,工具描述中的位置设置为(0,0,0)，姿态信息设置为(1,0,0,0)
 * 　　　其结构体定义为
 *      ｛
 *          pos{0,0,0},
 *          Ori{1,0,0,0}
 *      ｝  //伪代码
 *
 *      该结构同时用于用户坐标系的标定．一般通过示教3个示教点实现，第一个示教点是用户坐标系的原点；第二个和第三个示教点的选择根据标定方法来确定,遵循右手手法．
 */
typedef struct
{
    coordinate_refer      coordType;          // 坐标系类型

    CoordCalibrateMathod  methods;            // 用户坐标系的标定方法

    JointParam            wayPointArray[3];   //　用于标定用户坐标系的３个点(关节角)

    ToolInEndDesc         toolDesc;           //　工具描述

}CoordCalibrateByJointAngleAndTool;






/**
 * @brief 机械臂状态枚举
 **/
enum RobotState
{
    RobotStopped = 0,    //停止
    RobotRunning,        //运行
    RobotPaused,         //暂停
    RobotResumed         //恢复
};



/**
 * 机械臂重力分量x y z
 **/
typedef struct PACKED
{
    float x;
    float y;
    float z;
}RobotGravityComponent;



/**
 * 机械臂关节版本信息
 **/
typedef struct PACKED
{
    char hw_version[8];  //硬件版本信息

    char sw_version[16]; //固件版本信息
}JointVersion;


/**
 *  关节ID信息
 **/
typedef struct PACKED
{
    char productID[16];

}JointProductID;


/**
 *　该结构体描述设备信息
 **/
typedef struct PACKED
{
    uint8 type;                       // 设备型号、芯片型号：上位机主站：0x01  接口板0x02
    char revision[16];                // 设备版本号，eg:V1.0
    char manu_id[16];                 // 厂家ID，"OUR "的ASCII码0x4F 55 52 00
    char joint_type[16];              // 机械臂类型
    JointVersion joint_ver[8];        // 机械臂关节及工具端信息
    char desc[64];                    // 设备描述字符串以0x00结束
    JointProductID jointProductID[8]; // 关节ID信息
    char slave_version[16];           // 从设备版本号 - 字符串表示，如“V1.0.0
    char extio_version[16];           // IO扩展板版本号 -字符串标志，如“V1.0.0

}RobotDevInfo;


/**
 * 描述机械臂的关节状态
 */
typedef struct PACKED
{
    int    jointCurrentI;       // 关节电流    Current of driver
    int    jointSpeedMoto;      // 关节速度    Speed of driver
    float  jointPosJ;           // 关节角      Current position in radian
    float  jointCurVol;         // 关节电压    Rated voltage of motor. Unit: mV
    float  jointCurTemp;        // 当前温度    Current temprature of joint
    int    jointTagCurrentI;    // 电机目标电流 Target current of motor
    float  jointTagSpeedMoto;   // 电机目标速度 Target speed of motor
    float  jointTagPosJ;        // 目标关节角　 Target position of joint in radian
    uint16 jointErrorNum;       // 关节错误码   Joint error of joint num

}JointStatus;






/**
 * @brief 离线轨迹相关枚举
 */
enum Robot_Dyn_identify_traj
{
    Dyn_identify_traj_none = 0,
    Dyn_identify_traj_robot,      //submode: 0/1 <-> internal/hybrid
    Dyn_identify_traj_tool,       //submode: 0/1 <-> tool only/tool+friction
    Dyn_identify_traj_tool_abort
};



/**
 *  @brief 接口板固件升级枚举
 */
typedef enum
{
    update_master_board_firmware_trans_start = 1,
    update_master_board_firmware_trans_data  = 2,
    update_master_board_firmware_trans_end   = 3,
    update_slave_board_firmware_trans_start  = 4,
    update_slave_board_firmware_trans_data   = 5,
    update_slave_board_firmware_trans_end    = 6
}update_board_firmware_cmd;




/**
 * 描述机械臂事件类型　　　event define
 *
 * 机械臂的很多信息（故障，通知）是通过事件通知到客户的，所有在使用SDK时，
 * 务必注册接收事件的回调函数。
 */
typedef enum{
    RobotEvent_armCanbusError,           //机械臂CAN总线错误
    RobotEvent_remoteHalt,               //远程关机　　　　TODO
    RobotEvent_remoteEmergencyStop,      //机械臂远程急停
    RobotEvent_jointError,               //关节错误

    RobotEvent_forceControl,             //力控制
    RobotEvent_exitForceControl,         //退出力控制

    RobotEvent_softEmergency,            //软急停
    RobotEvent_exitSoftEmergency,        //退出软急停

    RobotEvent_collision,                //碰撞
    RobotEvent_collisionStatusChanged,   //碰撞状态改变
    RobotEvent_tcpParametersSucc,        //工具动力学参数设置成功
    RobotEvent_powerChanged,             //机械臂电源开关状态改变
    RobotEvent_ArmPowerOff,              //机械臂电源关闭
    RobotEvent_mountingPoseChanged,      //安装位置发生改变
    RobotEvent_encoderError,             //编码器错误

    RobotEvent_encoderLinesError,        //编码器线数不一致
    RobotEvent_singularityOverspeed,     //奇异点超速
    RobotEvent_currentAlarm,             //机械臂电流异常
    RobotEvent_toolioError,             //机械臂工具端错误
    RobotEvent_robotStartupPhase,       //机械臂启动阶段
    RobotEvent_robotStartupDoneResult,  //机械臂启动完成结果
    RobotEvent_robotShutdownDone,       //机械臂关机结果
    RobotEvent_atTrackTargetPos,        //机械臂轨迹运动到位信号通知

    RobotSetPowerOnDone,                //设置电源状态完成
    RobotReleaseBrakeDone,              //机械臂刹车释放完成
    RobotEvent_robotControllerStateChaned,  //机械臂控制状态改变
    RobotEvent_robotControllerError,        //机械臂控制错误----一般是算法规划出现问题时返回
    RobotEvent_socketDisconnected,          //socket断开连接

    RobotEvent_robotControlException,
    RobotEvent_trackPlayInterrupte,

    RobotEvent_staticCollisionStatusChanged,
    RobotEvent_MountingPoseWarning,

    RobotEvent_MacDataInterruptWarning,
    RobotEvent_ToolIoError,
    RobotEvent_InterfacBoardSafeIoEvent,

    RobotEvent_RobotHandShakeSucc,
    RobotEvent_RobotHandShakeFailed,

    RobotEvent_RobotErrorInfoNotify,

    RobotEvent_InterfacBoardDIChanged,
    RobotEvent_InterfacBoardDOChanged,
    RobotEvent_InterfacBoardAIChanged,
    RobotEvent_InterfacBoardAOChanged,

    RobotEvent_UpdateJoint6Rot360Flag,

    RobotEvent_RobotMoveControlDone,
    RobotEvent_RobotMoveControlStopDone,
    RobotEvent_RobotMoveControlPauseDone,
    RobotEvent_RobotMoveControlContinueDone,

    //主从模式切换
    RobotEvent_RobotSwitchToOnlineMaster,
    RobotEvent_RobotSwitchToOnlineSlave,

    RobotEvent_ConveyorTrackRobotStartup,
    RobotEvent_ConveyorTrackRobotCatchup,

    RobotEvent_exceptEvent = 100,


    RobotEventInvalid                         =1000,   // 无效的事件

    /**
     * RobotControllerErrorEvevt  控制器异常事件 1001~1499
     *
     * PS: 这些事件会引起机械臂运动的错误返回
     *     使用是尽量用枚举变量　枚举变量值只是为了查看日志方便
     **/
    RobotEventMoveJConfigError                 = 1001,  // moveJ configuration error 关节运动属性配置错误
    RobotEventMoveLConfigError                 = 1002,  // moveL configuration error 直线运动属性配置错误
    RobotEventMovePConfigError                 = 1003,  // moveP configuration error 轨迹运动属性配置错误
    RobotEventInvailConfigError                = 1004,  // invail configuration      无效的运动属性配置
    RobotEventWaitRobotStopped                 = 1005,  // please wait robot stopped 等待机器人停止
    RobotEventJointOutRange                    = 1006,  // joint out of range        超出关节运动范围
    RobotEventFirstWaypointSetError            = 1007,  // please set first waypoint correctly in modep    请正确设置MODEP第一个路点
    RobotEventConveyorTrackConfigError         = 1008,  // configuration error for conveyor tracking       传送带跟踪配置错误
    RobotEventConveyorTrackTrajectoryTypeError = 1009,  // unsupported conveyor tracking trajectory type   传送带轨迹类型错误
    RobotEventRelativeTransformIKFailed        = 1010,  // inverse kinematics failure due to invalid relative transform  相对坐标变换逆解失败
    RobotEventTeachModeCollision               = 1011,  // collision in teach-mode  示教模式发生碰撞
    RobotEventextErnalToolConfigError          = 1012,  // configuration error for external tool and hand workobject     运动属性配置错误,外部工具或手持工件配置错误

    RobotEventTrajectoryAbnormal               = 1101,  // Trajectory is abnormal 轨迹异常
    RobotEventOnlineTrajectoryPlanError        = 1102,  // Trajectory is abnormal,online planning failed  轨迹规划错误
    RobotEventOnlineTrajectoryTypeIIError      = 1103,  // Trajectory is abnormal,type II online planning failed 二型在线轨迹规划失败
    RobotEventIKFailed                         = 1104,  // Trajectory is abnormal,inverse kinematics failed 逆解失败
    RobotEventAbnormalLimitProtect             = 1105,  // Trajectory is abnormal,abnormal limit protection 动力学限制保护
    RobotEventConveyorTrackingFailed           = 1106,  // Trajectory is abnormal,conveyor tracking failed  传送带跟踪失败
    RobotEventConveyorOutWorkingRange          = 1107,  // Trajectory is abnormal,exceeding the conveyor working range 超出传送带工作范围
    RobotEventTrajectoryJointOutOfRange        = 1108,  // Trajectory is abnormal,joint out of range 关节超出范围
    RobotEventTrajectoryJointOverspeed         = 1109,  // Trajectory is abnormal,joint overspeed 关节超速
    RobotEventOfflineTrajectoryPlanFailed      = 1110,  // Trajectory is abnormal,Offline track planning failed 离线轨迹规划失败

    RobotEventControllerIKFailed               = 1200,  // The controller has an exception and the inverse kinematics failed 控制器异常，逆解失败
    RobotEventControllerStatusException        = 1201,  // The controller has an exception and the status is abnormal 控制器异常，状态异常

    RobotEventMoveEnterStopState               = 1300,  // Movement enters the stop state 运动进入到stop阶段



    /**
     * 通知性事件
     */


    //unknown event
    robot_event_unknown,

    //user event
    RobotEvent_User    = 9000,                            // first user event id
    RobotEvent_MaxUser = 9999                             // last user event id

}RobotEventType;


/** 事件类型 **/
typedef struct{
    RobotEventType  eventType;       //事件类型号
    int             eventCode;       //
    std::string     eventContent;    //事件内容
}RobotEventInfo;




/**
 *  接口函数 返回值信息  成功返回InterfaceCallSuccCode(0);失败返回对应的错误号
 *
 *　下面是错误代码列表
 *       大于20000的错误码　　表示错误发生是由于控制器抛出的事件导致的
 */
enum
{
    InterfaceCallSuccCode = 0,          //接口调用成功的返回值
};

typedef enum
{
    ErrnoSucc = aubo_robot_namespace::InterfaceCallSuccCode,       // 成功

    ErrCode_Base                           = 10000,
    ErrCode_Failed                         = 10001, // 通用失败 failed
    ErrCode_ParamError                     = 10002, // 参数错误 parameters error
    ErrCode_ConnectSocketFailed            = 10003, // 连接失败 socket connect failed Socket
    ErrCode_SocketDisconnect               = 10004, // Socket断开连接　socket disconnected Socket
    ErrCode_CreateRequestFailed            = 10005, // 创建请求失败 create request failed
    ErrCode_RequestRelatedVariableError    = 10006, // 请求相关的内部变量出错 internal error
    ErrCode_RequestTimeout                 = 10007, // 请求超时 timout
    ErrCode_SendRequestFailed              = 10008, // 发送请求信息失败 send request failed
    ErrCode_ResponseInfoIsNULL             = 10009, // 响应信息为空 response is null
    ErrCode_ResolveResponseFailed          = 10010, // 解析响应失败 parse response failed
    ErrCode_FkFailed                       = 10011, // 正解出错 fk failed
    ErrCode_IkFailed                       = 10012, // 逆解出错 ik failed
    ErrCode_ToolCalibrateError             = 10013, // 工具标定参数有错 tool coordinate paramter error
    ErrCode_ToolCalibrateParamError        = 10014, // 工具标定参数有错 tool coordinate paramter error
    ErrCode_CoordinateSystemCalibrateError = 10015, // 坐标系标定失败 user coordinate calibrate failed
    ErrCode_BaseToUserConvertFailed        = 10016, // 基坐标系转用户座标失败 base coordinate convert to user coordinate fialed
    ErrCode_UserToBaseConvertFailed        = 10017, // 用户坐标系转基座标失败 user coordinate convert to base coordinate fialed


    ErrCode_MotionRelatedVariableError     = 10018, // 运动相关的内部变量出错 move funcation paramters error
    ErrCode_MotionRequestFailed            = 10019, // 运动请求失败 call move funcation failed
    ErrCode_CreateMotionRequestFailed      = 10020, // 生成运动请求失败 create request failed
    ErrCode_MotionInterruptedByEvent       = 10021, // 运动被事件中断 move funcation interrupt
    ErrCode_MotionWaypointVetorSizeError   = 10022, // 运动相关的路点容器的长度不符合规定 parameter error
    ErrCode_ResponseReturnError            = 10023, // 服务器响应返回错误 server reponse error
    ErrCode_RealRobotNoExist               = 10024, // 真实机械臂不存在，因为有些接口只有在真是机械臂存在的情况下才可以被调用 real robot no exist

    ErrCode_moveControlSlowStopFailed      = 11025, // 调用缓停接口失败 call function failed, server side error
    ErrCode_moveControlFastStopFailed      = 11026, // 调用急停接口失败 call function failed, server side error
    ErrCode_moveControlPauseFailed         = 11027, // 调用暂停接口失败 call function failed, server side error
    ErrCode_moveControlContinueFailed      = 11028, // 调用继续接口失败 call function failed, server side error


    /**
     * 控制器异常
     * PS: 这些事件会引起机械臂运动的错误返回
     **/
    ErrCodeMoveJConfigError                 = 21001,  // moveJ configuration error 关节运动属性配置错误
    ErrCodeMoveLConfigError                 = 21002,  // moveL configuration error 直线运动属性配置错误
    ErrCodeMovePConfigError                 = 21003,  // moveP configuration error 轨迹运动属性配置错误
    ErrCodeInvailConfigError                = 21004,  // invail configuration      无效的运动属性配置
    ErrCodeWaitRobotStopped                 = 21005,  // please wait robot stopped 等待机器人停止
    ErrCodeJointOutRange                    = 21006,  // joint out of range        超出关节运动范围
    ErrCodeFirstWaypointSetError            = 21007,  // please set first waypoint correctly in modep    请正确设置MODEP第一个路点
    ErrCodeConveyorTrackConfigError         = 21008,  // configuration error for conveyor tracking       传送带跟踪配置错误
    ErrCodeConveyorTrackTrajectoryTypeError = 21009,  // unsupported conveyor tracking trajectory type   传送带轨迹类型错误
    ErrCodeRelativeTransformIKFailed        = 21010,  // inverse kinematics failure due to invalid relative transform  相对坐标变换逆解失败
    ErrCodeTeachModeCollision               = 21011,  // collision in teach-mode  示教模式发生碰撞
    ErrCodeextErnalToolConfigError          = 21012,  // configuration error for external tool and hand workobject     运动属性配置错误,外部工具或手持工件配置错误

    ErrCodeTrajectoryAbnormal               = 21101,  // Trajectory is abnormal 轨迹异常
    ErrCodeOnlineTrajectoryPlanError        = 21102,  // Trajectory is abnormal,online planning failed  轨迹规划错误
    ErrCodeOnlineTrajectoryTypeIIError      = 21103,  // Trajectory is abnormal,type II online planning failed 二型在线轨迹规划失败
    ErrCodeIKFailed                         = 21104,  // Trajectory is abnormal,inverse kinematics failed 逆解失败
    ErrCodeAbnormalLimitProtect             = 21105,  // Trajectory is abnormal,abnormal limit protection 动力学限制保护
    ErrCodeConveyorTrackingFailed           = 21106,  // Trajectory is abnormal,conveyor tracking failed  传送带跟踪失败
    ErrCodeConveyorOutWorkingRange          = 21107,  // Trajectory is abnormal,exceeding the conveyor working range 超出传送带工作范围
    ErrCodeTrajectoryJointOutOfRange        = 21108,  // Trajectory is abnormal,joint out of range 关节超出范围
    ErrCodeTrajectoryJointOverspeed         = 21109,  // Trajectory is abnormal,joint overspeed 关节超速
    ErrCodeOfflineTrajectoryPlanFailed      = 21110,  // Trajectory is abnormal,Offline track planning failed 离线轨迹规划失败

    ErrCodeControllerIKFailed               = 21200,  // The controller has an exception and the inverse kinematics failed 控制器异常，逆解失败
    ErrCodeControllerStatusException        = 21201,  // The controller has an exception and the status is abnormal 控制器异常，状态异常

    ErrCodeMoveEnterStopState               = 21300,  // Movement enters the stop state 运动进入到stop阶段
    ErrCodeMoveInterruptedByEvent           = 21301,  // Movement interrupted by the event 运动被事件中断


}RobotErrorCode;


}
#ifdef __cplusplus
}
#endif








/**
 * @brief             获取实时关节状态回调函数类型.
 * @param jointStatus　当前的关节状态;
 * @param size　　　　　上一个参数（jointStatus）的长度;
 * @param arg　　　　　　使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RealTimeJointStatusCallback)(const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg);


/**
 * @brief              获取实时路点信息的回调函数类型.
 * @param wayPoint　   当前的路点信息;
 * @param arg　　　　　　使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RealTimeRoadPointCallback)  (const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg);


/**
 *@brief         获取实时末端速度的回调函数类型
 *@param speed   当前的末端速度;
 *@param arg　　　使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RealTimeEndSpeedCallback)  (double speed, void *arg);


/**
 * @brief      获取机械臂事件信息的回调函数类型
 * @param arg  使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RobotEventCallback)         (const aubo_robot_namespace::RobotEventInfo *eventInfo, void *arg);




#endif // AUBOROBOTMETATYPE_H







//#define ErrnoSucc                               0                   // 成功
//#define ErrCode_Base                            10000
//#define ErrCode_Failed                          ErrCode_Base + 1    // 通用失败 failed
//#define ErrCode_ParamError                      ErrCode_Base + 2    // 参数错误 parameters error
//#define ErrCode_ConnectSocketFailed             ErrCode_Base + 3    // 连接失败 socket connect failed Socket
//#define ErrCode_SocketDisconnect                ErrCode_Base + 4    // 断开连接 socket disconnected Socket
//#define ErrCode_CreateRequestFailed             ErrCode_Base + 5    // 创建请求失败 create request failed
//#define ErrCode_RequestRelatedVariableError     ErrCode_Base + 6    // 请求相关的内部变量出错 internal error
//#define ErrCode_RequestTimeout                  ErrCode_Base + 7    // 请求超时 timout
//#define ErrCode_SendRequestFailed               ErrCode_Base + 8    // 发送请求信息失败 send request failed
//#define ErrCode_ResponseInfoIsNULL              ErrCode_Base + 9    // 响应信息为空 response is null
//#define ErrCode_ResolveResponseFailed           ErrCode_Base + 10   // 解析响应失败 parse response failed
//#define ErrCode_FkFailed                        ErrCode_Base + 11   // 正解出错 fk failed
//#define ErrCode_IkFailed                        ErrCode_Base + 12   // 逆解出错 ik failed
//#define ErrCode_ToolCalibrateError              ErrCode_Base + 13   // 工具标定参数有错 tool coordinate paramter error
//#define ErrCode_ToolCalibrateParamError         ErrCode_Base + 14   // 工具标定参数有错 tool coordinate paramter error
//#define ErrCode_CoordinateSystemCalibrateError  ErrCode_Base + 15   // 坐标系标定失败 user coordinate calibrate failed
//#define ErrCode_BaseToUserConvertFailed         ErrCode_Base + 16   // 基坐标系转用户座标失败 base coordinate convert to user coordinate fialed
//#define ErrCode_UserToBaseConvertFailed         ErrCode_Base + 17   // 用户坐标系转基座标失败 user coordinate convert to base coordinate fialed
//#define ErrCode_MotionRelatedVariableError      ErrCode_Base + 18   // 运动相关的内部变量出错 move funcation paramters error
//#define ErrCode_MotionRequestFailed             ErrCode_Base + 19   // 运动请求失败 call move funcation failed
//#define ErrCode_CreateMotionRequestFailed       ErrCode_Base + 20   // 生成运动请求失败 create request failed
//#define ErrCode_MotionInterruptedByEvent        ErrCode_Base + 21   // 运动被事件中断 move funcation interrupt
//#define ErrCode_MotionWaypointVetorSizeError    ErrCode_Base + 22   // 运动相关的路点容器的长度不符合规定 parameter error
//#define ErrCode_ResponseReturnError             ErrCode_Base + 23   // 服务器响应返回错误 server reponse error
//#define ErrCode_RealRobotNoExist                ErrCode_Base + 24   // 真实机械臂不存在，因为有些接口只有在真是机械臂存在的情况下才可以被调用 real robot no exist
//#define ErrCode_moveControlSlowStopFailed       ErrCode_Base + 25   // 调用缓停接口失败 call function failed, server side error
//#define ErrCode_moveControlFastStopFailed       ErrCode_Base + 26   // 调用急停接口失败 call function failed, server side error
//#define ErrCode_moveControlPauseFailed          ErrCode_Base + 27   // 调用暂停接口失败 call function failed, server side error
//#define ErrCode_moveControlContinueFailed       ErrCode_Base + 28   // 调用继续接口失败 call function failed, server side error
