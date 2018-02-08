#ifndef AUBOROBOTMETATYPE_H
#define AUBOROBOTMETATYPE_H

#include <iostream>
#include <stdint.h>
#include "robotiomatetype.h"


/* General types */
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


/** 命名空间 **/
namespace  aubo_robot_namespace
{

enum {ARM_DOF = 6};          /** 机械臂关节数 **/



/** 机械臂关节版本信息　**/
typedef struct PACKED
{
    //硬件版本信息
    char hw_version[8];
    //固件版本信息
    char sw_version[16];
}JointVersion;



typedef struct PACKED
{
    char productID[16];
}JointProductID;


/**
 *　该结构体描述设备信息
 **/
typedef struct PACKED
{
    //设备型号、芯片型号：上位机主站：0x01  接口板0x02
    uint8 type;
    //设备版本号，V1.0
    char revision[16];
    //厂家ID，"OUR "的ASCII码0x4F 55 52 00
    char manu_id[16];
    //机械臂类型
    char joint_type[16];
    //机械臂关节及工具端信息
    JointVersion joint_ver[8];
    //设备描述字符串以0x00结束
    char desc[64];
    //关节ID信息
    JointProductID jointProductID[8];

    //从设备版本号 - 字符串表示，如“V1.0.0
    char slave_version[16];
    //IO扩展板版本号 -字符串标志，如“V1.0.0
    char extio_version[16];

}RobotDevInfo;


/**
  * 该结构体描述机械臂的关节状态
  *
  */
typedef struct PACKED
{
    int    jointCurrentI;       /**< Current of driver   关节电流*/
    int    jointSpeedMoto;      /**< Speed of driver 　　关节速度*/
    float  jointPosJ;           /**< Current position in radian 　关节角*/
    float  jointCurVol;         /**< Rated voltage of motor. Unit: mV 　关节电压*/
    float  jointCurTemp;        /**< Current temprature of joint 　　　　当前温度*/
    int    jointTagCurrentI;    /**< Target current of motor 　　　　　 　电机目标电流*/
    float  jointTagSpeedMoto;   /**< Target speed of motor 　　　　　　　 电机目标速度*/
    float  jointTagPosJ;        /**< Target position of joint in radian 目标关节角　*/
    uint16 jointErrorNum;       /**< Joint error of joint num　　　　　　关节错误码 */
}JointStatus;


/**
 * event define   描述机械臂事件类型
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


    RobotEvent_exceptEvent = 100,

    //unknown event
    robot_event_unknown,

    //user event
    RobotEvent_User = 1000,                            // first user event id
    RobotEvent_MaxUser = 65535                         // last user event id

}RobotEventType;


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
    REMOTE_CLEAR_ALARM_SIGNAL
}InterfaceBoardSafeIoEventCode;


typedef enum {
    RobotToolNoError      = 0, //无错误
    RobotToolOverVoltage  = 1, //过压
    RobotToolUnderVoltage = 2, //欠压
    RobotToolOVerTemp     = 3, //过温
    RobotToolCanBusError  = 4  //CAN总线错误
}RobotToolErrorCode;


/** 事件类型 **/
typedef struct{
    RobotEventType  eventType;       //事件类型号
    int             eventCode;       //
    std::string     eventContent;    //事件内容
}RobotEventInfo;




/****机械臂重力分量x y z *****/
typedef struct PACKED
{
    float x;
    float y;
    float z;
}RobotGravityComponent;


/****机械臂诊断信息****/
typedef struct PACKED
{
    //CAN通信状态:0x01~0x80：关节CAN通信错误（每个关节占用1bit）
    //0x00：无错误 0xff：CAN总线存在错误
    uint8 armCanbusStatus;
    //机械臂48V电源当前电流
    float armPowerCurrent;
    //机械臂48V电源当前电压
    float armPowerVoltage;
    //机械臂48V电源状态（开、关）
    bool  armPowerStatus;
    //控制箱温度
    char  contorllerTemp;
    //控制箱湿度
    uint8 contorllerHumidity;
    //远程关机信号
    bool  remoteHalt;
    //机械臂软急停
    bool  softEmergency;
    //远程急停信号
    bool  remoteEmergency;
    //碰撞检测位
    bool  robotCollision;
    //机械臂进入力控模式标志位
    bool  forceControlMode;
    //刹车状态
    bool brakeStuats;
    //末端速度
    float robotEndSpeed;
    //最大加速度
    int robotMaxAcc;
    //上位机软件状态位
    bool orpeStatus;
    //位姿读取使能位
    bool enableReadPose;
    //安装位置状态
    bool robotMountingPoseChanged;
    //磁编码器错误状态
    bool encoderErrorStatus;
    //静止碰撞检测开关
    bool staticCollisionDetect;
    //关节碰撞检测 每个关节占用1bit 0-无碰撞 1-存在碰撞
    uint8 jointCollisionDetect;
    //光电编码器不一致错误 0-无错误 1-有错误
    bool encoderLinesError;
    //joint error status
    bool jointErrorStatus;
    //机械臂奇异点过速警告
    bool singularityOverSpeedAlarm;
    //机械臂电流错误警告
    bool robotCurrentAlarm;
    //tool error
    uint8 toolIoError;
    //机械臂安装位置错位（只在力控模式下起作用）
    bool robotMountingPoseWarning;
    //mac缓冲器长度
    uint16 macTargetPosBufferSize;
    //mac缓冲器有效数据长度
    uint16 macTargetPosDataSize;
    //mac数据中断
    uint8  macDataInterruptWarning;

}RobotDiagnosis;



//扩展设备诊断信息
typedef struct PACKED
{
    //机械臂状态
    //1- 接口板上电
    //2- 自检完毕
    //3- MAC连接
    //4- 等待机械臂上电
    //5- 初始化机械臂
    //6- 机械臂正常运行
    //7- 关机
    uint8_t robot_state;
    //机械臂当前运行状态
    //1- 开环
    //2- 电流环
    //3- 速度环
    //4- 位置环
    uint8_t robot_status;
    //目标位置
    float target_pos[6];
    //理论速度rad/s
    float theoretical_speed[6];
    //理论加速度rad/ss
    float theoretical_acc[6];
    //理论电流 /10 A
    int32_t theoretical_current[6];
}RobotExtDiagnosis;

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


typedef struct PACKED
{
    uint8 orpePause;               //上位机暂停状态
    uint8 orpeStop;                //上位机停止状态
    uint8 orpeError[16];           //上位机错误
    uint8 systemEmergencyStop;     //解除系统紧急停止输出信号
    uint8 reducedModeError;        //解除缩减错误
    uint8 safetyguardResetSucc;    //防护重置成功
}OrpeSafetyStatus;


typedef struct PACKED
{
    uint8  OriginPoseState;
    float   OriginPose[6];
}OriginPose;





/***************************关于IO的数据类型***************************************************/

/** 描述IO的类型**/
typedef enum
{
    RobotBoardControllerDI,    //接口板控制器DI(数字量输入)　　　只读(一般系统内部使用)
    RobotBoardControllerDO,    //接口板控制器DO(数字量输出)     只读(一般系统内部使用)
    RobotBoardControllerAI,    //接口板控制器AI(模拟量输入)　   只读(一般系统内部使用)
    RobotBoardControllerAO,    //接口板控制器AO(模拟量输出)　　　只读(一般系统内部使用)

    RobotBoardUserDI,          //接口板用户DI(数字量输入)　　可读可写
    RobotBoardUserDO,          //接口板用户DO(数字量输出)   可读可写
    RobotBoardUserAI,          //接口板用户AI(模拟量输入)   可读可写
    RobotBoardUserAO,          //接口板用户AO(模拟量输出)   可读可写

    RobotToolDI,               //工具端DI
    RobotToolDO,               //工具端DO
    RobotToolAI,               //工具端AI
    RobotToolAO,               //工具端AO

}RobotIoType;


typedef enum
{
    RobotToolIoTypeDI=RobotToolDI,      //工具端DI
    RobotToolIoTypeDO=RobotToolDO       //工具端DO
}RobotToolIoType;



typedef enum          //ＩＯ类型
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
    //数字IO数据
    uint8 ioData;
    //模拟IO数据
    float aiData[2];
    //系统电压
    float systemVoltage;
    //系统温度
    float systemTemperature;
    //错误状态
    uint8 errorStatus;
}RobotToolStatus;


typedef struct PACKED
{
    ToolIOType ioType;
    uint8 ioData;
}ToolDigitalStatus;

typedef struct PACKED
{
    //电源类型
    ToolPowerType  powerType;
    //系统电压
    float systemVoltage;
    //数字IO数据
    ToolDigitalStatus digitalIoStatus[4];
    //模拟IO数据
    float aiData[2];
    //系统温度
    float systemTemperature;
    //错误状态
    uint8 errorStatus;
}RobotToolAllIOStatus;

typedef struct
{
    //工具端配置
    //[0]:工具端电压类型
    //[1]:ＩＯ配置
    uint8 config[4];
}RobotToolConfig;

/*****************************************************************************************************/









typedef enum{
    RobotModeSimulator, //机械臂仿真模式
    RobotModeReal       //机械臂真实模式
}RobotWorkMode;














typedef enum {
    RobotMoveStop     = 0,
    RobotMovePause    = 1,
    RobotMoveContinue = 2,
}RobotMoveControlCommand;


enum RobotControlCommand{
    RobotRelease        = 0,    //释放刹车
    RobotBrake          = 1,    //刹车
    OverspeedWarning    = 2,    //拖动示教速度过快报警
    OverspeedRecover    = 3,    //解除拖动过速报警
    DisableForceControl = 4,    //失能力控
    EnableForceControl  = 5,    //使能力控
    OrpeOpen            = 6,    //打开上位机软件
    OrpeClose           = 7,    //关闭上位机软件
    EnableReadPose      = 8,    //打开读取位姿
    DisableReadPose     = 9,    //关闭读取位姿
    MountingPoseChanged    = 10,//安装位置已改变
    MountingPoseUnChanged  = 11,//安装位置未改变
    EnableStaticCollisionDetect  = 12,   //打开静止碰撞检测
    DisableStaticCollisionDetect = 13,   //关闭静止碰撞检测
    ClearSingularityOverSpeedAlarm = 14, //解除机械臂奇异点过速警告
    ClearRobotCurrentAlarm = 15          //解除机械臂电流错误警告
};


enum Robot_Dyn_identify_traj
{
    Dyn_identify_traj_none = 0,
    Dyn_identify_traj_robot, //submode: 0/1 <-> internal/hybrid
    Dyn_identify_traj_tool,  //submode: 0/1 <-> tool only/tool+friction
    Dyn_identify_traj_tool_abort
};



/**
 * @brief 机械臂初始化阶段
 */
enum ROBOT_INIT_PHASE{
    ROBOT_INIT_PHASE_READY=0,
    ROBOT_INIT_PHASE_HANDSHAKE,
    ROBOT_INIT_PHASE_SET_POWER,
    ROBOT_INIT_PHASE_SET_BRAKE,
    ROBOT_INIT_PHASE_SET_COLLSION_CLASS,
    ROBOT_INIT_PHASE_SET_OTHER_CMD,
    ROBOT_INIT_PHASE_WORKING
};

/**
 * @brief 机械臂启动完成状态
 */
enum ROBOT_SERVICE_STATE{
    ROBOT_SERVICE_READY=0,
    ROBOT_SERVICE_STARTING,
    ROBOT_SERVICE_WORKING,
    ROBOT_SERVICE_CLOSING,
    ROBOT_SERVICE_CLOSED,
    ROBOT_SETVICE_FAULT_POWER,
    ROBOT_SETVICE_FAULT_BRAKE,
    ROBOT_SETVICE_FAULT_NO_ROBOT
};


/** 机械臂状态枚举　**/
enum RobotState
{
    RobotStopped = 0,
    RobotRunning,
    RobotPaused,
    RobotResumed
};

/** 运动模式枚举　**/
enum move_mode
{
    NO_MOVEMODE = 0,
    MODEJ,
    MODEL,
    MODEP
};

/** 运动轨迹枚举　**/
enum move_track
{
    NO_TRACK = 0,

    //for moveJ and moveL
    TRACKING,

    //cartesian motion for movep
    ARC_CIR,
    CARTESIAN_MOVEP,
    CARTESIAN_CUBICSPLINE,
    CARTESIAN_UBSPLINEINTP,

    //joint motion  for movep
    JIONT_CUBICSPLINE,
    JOINT_UBSPLINEINTP,
};


/** 坐标系枚举　**/
enum coordinate_refer
{
    BaseCoordinate = 0,
    EndCoordinate,
    WorldCoordinate
};


/** 示教模式枚举　**/
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


/** 路点位置信息的表示方法　**/
struct Pos
{
    double x;
    double y;
    double z;
};

/** 路点位置信息的表示方法　**/
union cartesianPos_U
{
    Pos position;
    double positionVector[3];
};


/** 姿态的四元素表示方法　**/
struct Ori
{
    double w;
    double x;
    double y;
    double z;
};

/** 姿态的欧拉角表示方法　**/
struct Rpy
{
    double rx;
    double ry;
    double rz;
};


/** 描述机械臂的路点信息　**/
typedef struct
{
    cartesianPos_U cartPos;     //机械臂的位置信息　X,Y,Z

    Ori orientation;            //机械臂姿态信息　　通过四元素表示,可以通过工具函数实现与欧拉角互转

    double jointpos[ARM_DOF];   //机械臂关节角信息
}wayPoint_S;


/**
  * 描述关节的速度和加速度
  */
typedef struct
{
    double jointPara[ARM_DOF];
}JointVelcAccParam;


typedef struct
{
    double jointPos[ARM_DOF];
}JointParam;


/**
 *  描述运动属性中的偏移属性
 */
typedef struct
{
    bool  ena;                       //是否使能偏移

    float relativePosition[3];       //偏移量 x,y,z

}MoveRelative;




/** 该结构体描述工具的参数
  * 工具标定后既有相对末端坐标系的位置也有姿态。
  */
typedef struct
{
    Pos        toolInEndPosition;     //工具相对末端坐标系的位置

    Ori        toolInEndOrientation;  //工具相对末端坐标系的姿态
}ToolInEndDesc;


typedef ToolInEndDesc ToolKinematicsParam;



enum ToolKinematicsOriCalibrateMathod{
    ToolKinematicsOriCalibrateMathod_Invalid = -1,

    ToolKinematicsOriCalibrateMathod_xOxy,       // 原点、x轴正半轴、x、y轴平面的第一象限上任意一点
    ToolKinematicsOriCalibrateMathod_yOyz,       // 原点、y轴正半轴、y、z轴平面的第一象限上任意一点
    ToolKinematicsOriCalibrateMathod_zOzx,       // 原点、z轴正半轴、z、x轴平面的第一象限上任意一点

    ToolKinematicsOriCalibrateMathod_TxRBz_TxyPBzAndTyABnz,  //工具x轴平行反向于基坐标系z轴; 工具xOy平面平行于基坐标系z轴、工具y轴与基坐标系负z轴夹角为锐角
    ToolKinematicsOriCalibrateMathod_TyRBz_TyzPBzAndTzABnz,  //工具y轴平行反向于基坐标系z轴; 工具yOz平面平行于基坐标系z轴、工具z轴与基坐标系负z轴夹角为锐角
    ToolKinematicsOriCalibrateMathod_TzRBz_TzxPBzAndTxABnz,  //工具z轴平行反向于基坐标系z轴; 工具zOx平面平行于基坐标系z轴、工具x轴与基坐标系负z轴夹角为锐角

    ToolKinematicsOriCalibrateMathodCount
};


/** 该结构体描述工具惯量　**/
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
 * 该结构体描述工具的　动力学参数
 *
 * 注意：在跟换机械臂的工具时，工具的动力学参数和运动学参数是需要一起设置的。
 **/
typedef struct
{
    double positionX;    //工具重心的X坐标

    double positionY;    //工具重心的Y坐标

    double positionZ;    //工具重心的Z坐标

    double payload;      //工具重量

    ToolInertia toolInertia;  //工具惯量

}ToolDynamicsParam;


//关节碰撞补偿（范围0.00~0.51度）
typedef struct
{
    double jointOffset[ARM_DOF];
}RobotJointOffset;


/** 坐标系标定方法枚举 **/
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

    CoordTypeCount
};


/**
 * 该结构体描述一个坐标系。系统根据该结构体能够唯一个确定一个坐标系。
 *
 * 坐标系分３种类型：基座标系(BaseCoordinate)，末端坐标系(EndCoordinate)，用户坐标系(WorldCoordinate);
 *
 *　基座坐标系是　根据机械臂底座建立的坐标系
 *　末端坐标系是　根据法兰盘中心建立的坐标系
 *　用户坐标系是　用户根据自己的需求建立的实际需要用到的坐标系,系统根据用户提供的３个点和标定方法确定用户坐标系的X轴,Y轴,Z轴。
 * 　　　　　　　 在实际应用中标定坐标系时会用到工具，系统为了准确的得到坐标系标定的３个点，所以用户需要提供工具信息，
 * 　　　　　　　 如果没有使用工具可以将工具描述(toolDesc)设置为０．
 *
 * 使用说明：
 * 　　1:当coordType==BaseCoordinate或者coordType==EndCoordinate时，下面3个参数(methods,wayPointArray,toolDesc)系统不做处理，
 * 　　　因为系统根据这个参数coordType已经可以确定该坐标系了。
 *
 * 　　2:如果坐标系标定的时候没有使用工具，工具描述中的位置和姿态信息应设置为０。
 */
typedef struct
{
    coordinate_refer    coordType;       //坐标系类型：当coordType==BaseCoordinate或者coordType==EndCoordinate是，下面3个参数不做处理

    CoordCalibrateMathod methods;        // 坐标系标定方法

    JointParam       wayPointArray[3];   //用于标定坐标系的３个点（关节角），对应于机械臂法兰盘中心点基于基座标系

    ToolInEndDesc    toolDesc;           //标定的时候使用的工具描述

}CoordCalibrateByJointAngleAndTool;



/**
 *  接口调用成功的返回值
 *  接口返回值多数是整型值,成功返回InterfaceCallSuccCode，失败返回错误号，
 *
 * 　注：错误号也是正整数。
 */
enum
{
    InterfaceCallSuccCode = 0,          //接口调用成功的返回值
};

typedef enum
{
    ErrnoSucc = aubo_robot_namespace::InterfaceCallSuccCode,  /** 成功　**/

    ErrCode_Base = 10000,
    ErrCode_Failed,       /** 通用失败　**/
    ErrCode_ParamError,   /** 参数错误　**/
    ErrCode_ConnectSocketFailed,        /** Socket连接失败　**/
    ErrCode_SocketDisconnect,           /** Socket断开连接　**/
    ErrCode_CreateRequestFailed,        /** 创建请求失败　**/
    ErrCode_RequestRelatedVariableError,/** 请求相关的内部变量出错　**/
    ErrCode_RequestTimeout,             /** 请求超时　**/
    ErrCode_SendRequestFailed,          /** 发送请求信息失败　**/
    ErrCode_ResponseInfoIsNULL ,        /** 响应信息为空　**/
    ErrCode_ResolveResponseFailed ,     /** 解析响应失败　**/
    ErrCode_FkFailed,                   /** 正解出错　**/
    ErrCode_IkFailed,                   /** 逆解出错　**/
    ErrCode_ToolCalibrateError,              /** 工具标定参数有错**/
    ErrCode_ToolCalibrateParamError,         /** 工具标定参数有错**/
    ErrCode_CoordinateSystemCalibrateError,  /** 坐标系标定失败　**/
    ErrCode_BaseToUserConvertFailed,         /** 基坐标系转用户座标失败　**/
    ErrCode_UserToBaseConvertFailed,         /** 用户坐标系转基座标失败　**/

    //move
    ErrCode_MotionRelatedVariableError,      /** 运动相关的内部变量出错　**/
    ErrCode_MotionRequestFailed,             /** 运动请求失败**/
    ErrCode_CreateMotionRequestFailed,       /** 生成运动请求失败**/
    ErrCode_MotionInterruptedByEvent,        /** 运动被事件中断　**/
    ErrCode_MotionWaypointVetorSizeError,    /** 运动相关的路点容器的长度不符合规定　**/
    ErrCode_ResponseReturnError,             /** 服务器响应返回错误　**/
    ErrCode_RealRobotNoExist,                /** 真实机械臂不存在，因为有些接口只有在真是机械臂存在的情况下才可以被调用　**/


    ErrCode_Count = ErrCode_RealRobotNoExist-ErrCode_Base+2,

}RobotErrorCode;



typedef enum
{
    update_master_board_firmware_trans_start = 1,
    update_master_board_firmware_trans_data = 2,
    update_master_board_firmware_trans_end = 3,
    update_slave_board_firmware_trans_start = 4,
    update_slave_board_firmware_trans_data = 5,
    update_slave_board_firmware_trans_end = 6
}update_board_firmware_cmd;



}
#ifdef __cplusplus
}
#endif



/**
 * @brief 获取实时关节状态的回调函数类型.
 * @param jointStatus　当前的关节状态;
 * @param size　　　　　上一个参数（jointStatus）的长度;
 * @param arg　　　　　　这个参数是使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RealTimeJointStatusCallback)(const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg);


/**
 * @brief 获取实时路点信息的回调函数类型.
 * @param wayPoint　   当前的路点信息;
 * @param arg　　　　　　这个参数是使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RealTimeRoadPointCallback)  (const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg);


/**
 *@brief 获取实时末端速度的回调函数类型
 *@param speed   当前的末端速度;
 *@param arg　　　这个参数是使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RealTimeEndSpeedCallback)  (double speed, void *arg);


/**
 * @brief  获取机械臂事件信息的回调函数类型
 * @param arg　这个参数是使用者在注册回调函数中传递的第二个参数;
 */
typedef void (*RobotEventCallback)         (const aubo_robot_namespace::RobotEventInfo *eventInfo, void *arg);




#endif // AUBOROBOTMETATYPE_H
