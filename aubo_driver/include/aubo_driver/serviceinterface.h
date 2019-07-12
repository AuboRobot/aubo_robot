/** ************************************************************************************************************************************************************
  *Copyright(C), 2016-2017,OUR(Beijing) Robotics Technolohy Co., Ltd
  *
  *FileName:     serviceinterface.h
  *
  *Author:
  *
  *Version:      V1.0.0
  *
  *Date:         2017-06-01
  *
  *Description:  本文件使用机械臂的接口类. 是基于C++开发的,其中类中的方法是操作机械臂的接口.
  *
  *Others:   　　　接口中长度单位都为米(m)   角度单位（弧度）
  *
  *Function List:
  *
  * 1. 回调函数的类型:
        1.1  typedef void (*RealTimeJointStatusCallback)(aubo_robot_namespace::JointStatus *jointStatus, int size);   获取关节状态的回调函数类型
        1.2  typedef void (*RealTimeRoadPointCallback)  (const aubo_robot_namespace::wayPoint_S  *wayPoint);          获取路点信息的回调函数类型
        1.3  typedef void (*RealTimeEndSpeedCallback)   (double speed);                                               获取末端速度的回调函数类型
        1.4  typedef void (*RobotEventCallback)         (const aubo_robot_namespace::RobotEventInfo *eventInfo);      获取机械臂事件信息的回调函数类型

  * 2. 构造函数
        2.1  ServiceInterface()

  * 3. 系统接口
        3.1  robotServiceLogin()     登录接口: 与机械臂服务器建立连接.   这个接口是使用其他所有接口的前提,只有在该接口正确返回的情况下,才能使用其他接口.
        3.2  robotServiceLogout()    退出登录,断开与机械臂服务器的连接

  * 4. 机械臂运动接口
        4.1  robotServiceSetGlobalMoveMaxAcc()     设置全局的最大运动加速度
        4.2  robotServiceSetGlobalMoveMaxVelc()    设置全局的最大运动速度
        4.3  robotServiceJointMoveOnBaseCoord()    基于基座标系的关节运动   该函数进行了重载
        4.4  robotServiceTeachJointMove()          示教关节运动
        4.5  robotServiceTeachPosition()           示教位置运动

  * 5. 状态推送接口
  *
  *
  *
  *注：
  * 　　接口中关于长度的单位都为米　　　关于角度的单位都为弧度
  *
  *
  *History:
     1.Date:           V1.0.0
       Author:
       Modification:   create
 ************************************************************************************************************************************************************/


/**
CopyRight © AUBO Robotics Technology Co.Ltd. All Rights Reserved

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of mosquitto nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


This product includes software developed by the OpenSSL Project for use in the
OpenSSL Toolkit. (http://www.openssl.org/)
This product includes cryptographic software written by Eric Young
(eay@cryptsoft.com)
This product includes software written by Tim Hudson (tjh@cryptsoft.com)
*/



#ifndef SERVICEINTERFACE_H
#define SERVICEINTERFACE_H

#include <vector>
#include "AuboRobotMetaType.h"        //接口需要用到的数据类型


class RobotControlServices;
class RobotMoveService;
class RobotIoService;
class RobotConveyorTrack;
class robotOtherService;
class ServiceInterface    /** 对外接口类 : 为用户提供开发接口 **/
{
public:

    /**
     * @brief 构造函数
     */
    ServiceInterface();
    
    /**
     * @brief 析构函数
     */
    ~ServiceInterface();

public: /** 数据类型初始化*/
    static void initPosDataType(aubo_robot_namespace::Pos &postion);

    static void initOriDataType(aubo_robot_namespace::Ori &ori);

    static void initMoveRelativeDataType(aubo_robot_namespace::MoveRelative &moveRelative);

    static void initWayPointDataType(aubo_robot_namespace::wayPoint_S &wayPoint);

    static void initToolInEndDescDataType(aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);

    static void initCoordCalibrateByJointAngleAndToolDataType(aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coord);

    static void initToolInertiaDataType(aubo_robot_namespace::ToolInertia &toolInertia);

    static void initToolDynamicsParamDataType(aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);


    /**********************************************************************************************************************************************
     ***********　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　机械臂系统接口　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　**********
     **********************************************************************************************************************************************/

public:

    /**
     * @brief 登录　　与机械臂服务器建立网络连接；
     *        该接口的成功是调用其他接口的前提。
     *
     * @param host     机械臂服务器的IP地址
     * @param port     机械臂服务器的端口号
     * @param userName 用户名  默认为Aubo
     * @param possword 密码 　 默认为123456
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceLogin(const char* host, int port, const char *userName, const char* possword);

    int  robotServiceLogin(const char* host, int port, const char *userName, const char* possword, aubo_robot_namespace::RobotType &robotType, aubo_robot_namespace::RobotDhPara &robotDhPara);

    /**
     * @brief robotServiceGetConnectStatus   获取当前的连接状态
     * @param connectStatus　　　　　　　　　　　输出参数
     */
    void robotServiceGetConnectStatus(bool &connectStatus);


    /**
     * @brief 退出登录
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceLogout();

    int  robotServiceRobotHandShake(bool isBlock);


    /**********************************************************************************************************************************************
     *****　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　状态推送　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　*******
     **********************************************************************************************************************************************/
public:

    /**
     * @brief robotServiceSetRealTimeJointStatusPush   设置是否允许实时关节状态推送
     * @param enable   true表示允许　　　false表示不允许
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetRealTimeJointStatusPush(bool enable);


    /**
     * @brief robotServiceRegisterRealTimeJointStatusCallback   注册用于获取关节状态的回调函数
     *        注册回调函数后,服务器实时推送当前的关节状态信息.
     * @param ptr   获取实时关节状态信息的函数指针，当ptr==NULL时，相当于取消回调函数的注册,取消该推送信息也可以通过该接口robotServiceSetRealTimeJointStatusPush进行。
     * @param arg   这个参数系统不做任何处理，只是进行了缓存，当回调函数触发时该参数会通过回调函数的参数传回。
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceRegisterRealTimeJointStatusCallback(RealTimeJointStatusCallback ptr, void  *arg);



    /**
     * @brief robotServiceSetRealTimeRoadPointPush   设置是否允许实时路点信息推送
     * @param enable　　true表示允许　　　false表示不允许
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetRealTimeRoadPointPush(bool enable);

    /**
     * @brief robotServiceRegisterRealTimeRoadPointCallback   注册用于获取实时路点的回调函数
     *        注册回调函数后,服务器会推送当前的路点信息　当ptr==NULL时，相等于取消回调函数的注册。
     * @param ptr  获取实时路点信息的函数指针，当ptr==NULL时，相当于取消回调函数的注册,取消该推送信息也可以通过该接口robotServiceSetRealTimeRoadPointPush进行。
     * @param arg　这个参数系统不做任何处理，只是进行了缓存，当回调函数触发时该参数会通过回调函数的参数传回。
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceRegisterRealTimeRoadPointCallback(const RealTimeRoadPointCallback ptr, void  *arg);


    /**
     * @brief robotServiceSetRealTimeEndSpeedPush   设置是否允许实时末端速度推送
     * @param enable   true表示允许　　　false表示不允许
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetRealTimeEndSpeedPush(bool enable);

    /**
     * @brief robotServiceRegisterRealTimeEndSpeedCallback   注册用于获取实时末端速度的回调函数
     * @param ptr  获取实时末端速度的函数指针　当ptr==NULL时，相等于取消回调函数的注册，取消该推送信息也可以通过该接口robotServiceSetRealTimeEndSpeedPush进行。
     * @param arg　这个参数系统不做任何处理，只是进行了缓存，当回调函数触发时该参数会通过回调函数的参数传回。
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceRegisterRealTimeEndSpeedCallback(const RealTimeEndSpeedCallback ptr, void  *arg);


    /**
     * @brief robotServiceRegisterRobotEventInfoCallback  注册用于获取机械臂事件信息的回调函数
     *              注:关于事件信息信息的推送没有提供是否允许推送的接口，因为机械臂的很多重要通知都是通过事件推送实现的,所以事件信息是系统默认推送的,不允许取消的。
     * @param ptr   获取机械臂事件信息的函数指针　当ptr==NULL时，相等于取消回调函数的注册。
     * @param arg　　这个参数系统不做任何处理，只是进行了缓存，当回调函数触发时该参数会通过回调函数的参数传回。
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceRegisterRobotEventInfoCallback(RobotEventCallback ptr, void  *arg);



private:
    /**
     * @brief 接受服务器推送的实时关节状态 并进行处理  (SDK内部使用,开发者不需要关心)
     * @param jointStatus   机械臂关节状态,输入参数
     * @param size          jointStatus数组的长度   长度为6,因为有6个关节
     */
    static void  recvRealTimeJointStatusPushCallback(const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg);

    static void  recvRealTimeWaypointPushCallback(const aubo_robot_namespace::wayPoint_S *waypoint,void *arg);

    /**
     * @brief 接受服务器推送的实时末端速度 并进行处理  (SDK内部使用,开发者不需要关心)
     * @param speed
     */
    static void  recvRealTimeEndSpeedPushCallback(double speed, void *arg);


    /**
     * @brief 接收服务器推送的事件并进行处理  (SDK内部使用,开发者不需要关心)
     * @param eventInfo  事件信息  输入参数
     */
    static void  recvRobotEventPushCallback(const aubo_robot_namespace::RobotEventInfo *info, void *arg);


    /*******************************************************************机械臂运动接口相关的接口*************************************************************
     * 运动属性包含:
     *          0:关节型运动的最大速度和加速度;  (当运动类型为关节型运动时有效)
     *          1:末端型运动的最大速度和加速度;  (当运动类型为末端型运动时有效)
     *          2:路点信息缓存(轨迹运动使用);
     *          3:交融半径(轨迹运动子类型MOVEP使用);
     *          4:圆的圈数(当轨迹类型为ARC_CIR时有效，当圆的圈数属性（CircularLoopTimes）为０时，表示圆弧轨迹;当圆的圈数属性（CircularLoopTimes）大于０时，表示圆轨迹。)
     *          5:偏移量属性　　(除示教运动外的所有运动有效)
     *          6:工具参数属性
     *          7:设置示教坐标系(仅适用于示教运动)
     *
     *
     * 本机械臂接口支持下面几种运动，3大类:
     *          1:关节运动    对应接口函数为 robotServiceJointMove();
     *          2:直线运动    对应接口函数为 robotServiceLineMove();
     *          3:轨迹运动    其中轨迹运动又可以分为 ARC_CIR, CARTESIAN_MOVEP, CARTESIAN_CUBICSPLINE,CARTESIAN_UBSPLINEINTP,JIONT_CUBICSPLINE,JOINT_UBSPLINEINTP;
     *                       对应接口函数为 robotServiceTrackMove();
     *根据上面的运动扩展接口:
     *          1:
     *
     *
     **************************************************************************************************************************************************/

public:

    /**
     * @brief robotServiceInitGlobalMoveProfile   初始化全局的运动属性
     *        调用此函数时机械臂不运动，该函数初始化各个运动属性为默认值。
     * 　　　　初始化后各属性的默认值为:
     *              0:关节型运动的最大速度和加速度属性:　关节最大速度默认为25度米每秒；关节最大加速度默认为25度米每秒方
     *              1:末端型运动的最大速度和加速度属性:  末端最大速度默认为３米每秒;末端最大加速度３米每秒方
     *              2:路点信息缓存属性:　　默认路点缓存为空
     *              3:交融半径属性:       默认为0.02米
     *              4:圆的圈数属性:       默认为0
     *              5:偏移量属性属性:     默认没有偏移
     *              6:工具参数属性属性:   无工具即工具的位置为０００;
     *              7:示教坐标系属性:　   示教坐标系为基座标系
     *
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceInitGlobalMoveProfile();


    /**
     * 设置和获取　关节型　运动的最大速度和加速度
     * 关节型运动包含：
     * 　　　　关节运动；
     * 　　　　示教运动中的关节示教（JOINT1，JOINT2，JOINT3，JOINT4，JOINT5，JOINT6）
     * 　　　　轨迹运动下的（JIONT_CUBICSPLINE，JOINT_UBSPLINEINTP）
     *
     * 注意：用户在设置速度和加速度时，需要根据运动的类型设置，
     * 　　　关节型运动设置关节型运动的最大速度和加速度,关节型运动的最大速度是180度每秒，最大加速度为180度每秒方;
     * 　　　末端型运动会设置末端型运动的最大速度和加速度，末端型运动的最大速度为2米每秒，最大加速度为2米每秒方;
     *
     **/
    int   robotServiceSetGlobalMoveJointMaxAcc (const aubo_robot_namespace::JointVelcAccParam  &moveMaxAcc);

    int   robotServiceSetGlobalMoveJointMaxVelc(const aubo_robot_namespace::JointVelcAccParam  &moveMaxVelc);

    void  robotServiceGetGlobalMoveJointMaxAcc (aubo_robot_namespace::JointVelcAccParam        &moveMaxAcc);

    void  robotServiceGetGlobalMoveJointMaxVelc(aubo_robot_namespace::JointVelcAccParam        &moveMaxVelc);


    /**
     * 设置和获取　末端型　运动的最大速度和加速度
     * 末端型包含：　直线运动（MODEL）；
     * 　　　　示教运动中的位置示教和姿态示教（MOV_X，MOV_Y，MOV_Z，ROT_X，ROT_Y，ROT_Z）；
     * 　　　　轨迹运动下的（ARC_CIR,　CARTESIAN_MOVEP,　CARTESIAN_CUBICSPLINE,　CARTESIAN_UBSPLINEINTP）
     *
     * 注意：用户在设置速度和加速度时，需要根据运动的类型设置，
     *      关节型运动设置关节型运动的最大速度和加速度,关节型运动的最大速度是180度每秒，最大加速度为180度每秒方;
     * 　　　末端型运动会设置末端型运动的最大速度和加速度，末端型运动的最大速度为2米每秒，最大加速度为2米每秒方;
     *
     **/
    int   robotServiceSetGlobalMoveEndMaxLineAcc (double  moveMaxAcc);

    int   robotServiceSetGlobalMoveEndMaxLineVelc(double  moveMaxVelc);

    void  robotServiceGetGlobalMoveEndMaxLineAcc (double  &moveMaxAcc);

    void  robotServiceGetGlobalMoveEndMaxLineVelc(double  &moveMaxVelc);

    int   robotServiceSetGlobalMoveEndMaxAngleAcc (double  moveMaxAcc);

    int   robotServiceSetGlobalMoveEndMaxAngleVelc(double  moveMaxVelc);

    void  robotServiceGetGlobalMoveEndMaxAngleAcc (double  &moveMaxAcc);

    void  robotServiceGetGlobalMoveEndMaxAngleVelc(double  &moveMaxVelc);

    /**
      *　设置加加速度
      */
    int   robotServiceSetJerkAccRatio(double acc);

    void   robotServiceGetJerkAccRatio(double  &acc);

    /** 运动属性中的路点设置与获取　多用于轨迹运动**/
    void  robotServiceClearGlobalWayPointVector();

    /**
     * @brief robotServiceAddGlobalWayPoint  添加路点，一般用于robotServiceTrackMove中
     * @param wayPoint　　法兰盘中心基于基座标系的路点信息
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int   robotServiceAddGlobalWayPoint(const aubo_robot_namespace::wayPoint_S &wayPoint);

    /**
     * @brief robotServiceAddGlobalWayPoint  添加路点，一般用于robotServiceTrackMove中
     * @param jointAngle　　　  关节信息
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     **/

    int   robotServiceAddGlobalWayPoint(const double jointAngle[aubo_robot_namespace::ARM_DOF]);

    void  robotServiceGetGlobalWayPointVector(std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);




    /** 运动属性之交融半径的设置与获取　交融半径的方位：0.0m~0.05m  注意：交融半径必须大于0.0**/
    float robotServiceGetGlobalBlendRadius();

    int   robotServiceSetGlobalBlendRadius(float value);


    double  robotServiceGetTrackPlaybackCycle();

    int robotServiceSetTrackPlaybackCycle(double second);


    /** 运动属性之圆轨迹时圆的圈数
     * 当轨迹类型为ARC_CIR时有效，当圆的圈数属性（CircularLoopTimes）为０时，表示圆弧轨迹;
     *                        当圆的圈数属性（CircularLoopTimes）大于０时，表示圆轨迹。
     **/
    int   robotServiceGetGlobalCircularLoopTimes();

    void  robotServiceSetGlobalCircularLoopTimes(int times);


    /**
     * @brief robotServiceSetMoveRelativeParam   设置运动属性中的偏移属性
     * @param relativeMove      基于基座标系的偏移
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceSetMoveRelativeParam(const aubo_robot_namespace::MoveRelative &relativeMoveOnBase);              //基于基座标系

    /**
     * @brief robotServiceSetMoveRelativeParam   设置运动属性中的偏移属性
     * @param relativeMoveOnUserCoord      基于用户坐标系(下面参数userCoord)下的偏移
     * @param userCoord        用户坐标系
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceSetMoveRelativeParam(const aubo_robot_namespace::MoveRelative   &relativeMoveOnUserCoord,
                                          const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord);  //基于自定义坐标系


    /** 跟随模式之提前到位　当前仅适用于关节运动**/
    int  robotServiceSetNoArrivalAhead();

    int  robotServiceSetArrivalAheadDistanceMode(double distance /*米*/);

    int  robotServiceSetArrivalAheadTimeMode(double second /*秒*/);

    int  robotServiceSetArrivalAheadBlendDistanceMode(double distance /*米*/);


    /**
     * @brief robotServiceSetTeachCoordinateSystem   设置示教运动的坐标系
     * @param coordSystem　　　用户坐标系    通过该参数确定一个坐标系,具体使用参考类型定义处的使用说明
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetTeachCoordinateSystem(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordSystem);


    /**
     * @brief robotServiceJointMove   运动接口之关节运动
     *        属于关节型运动,调用该函数机械臂开始运动
     * @param wayPoint　　目标路点信息
     * @param IsBolck    IsBolck==true  代表阻塞，机械臂运动直到到达目标位置或者出现故障后返回。
     *                   IsBolck==false 代表非阻塞，立即返回，运动指令发送成功就返回，函数返回后机械臂开始运动。
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceJointMove(aubo_robot_namespace::wayPoint_S   &wayPoint,     bool IsBolck);

    int robotServiceJointMove(double jointAngle[aubo_robot_namespace::ARM_DOF], bool IsBolck);

    //基于跟随模式的轴动接口
    int robotServiceFollowModeJointMove(double jointAngle[aubo_robot_namespace::ARM_DOF]);

    /**
     * @brief robotServiceLineMove    运动接口之直线运动
     *        属于末端型运动,调用该函数机械臂开始运动
     * @param wayPoint　　　目标路点信息
     * @param IsBolck      参考robotServiceJointMove函数的解释
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceLineMove(aubo_robot_namespace::wayPoint_S   &wayPoint,      bool IsBolck);

    int robotServiceLineMove(double jointAngle[aubo_robot_namespace::ARM_DOF],  bool IsBolck);


    /**
     * @brief robotServiceRotateMove   运动接口之保持当前位置变换姿态做旋转运动
     * @param rotateAxis    转轴[x,y,z]    当绕Ｘ转，[１，０，０]
     * @param rotateAngle　 转角
     * @param IsBolck      是否阻塞
     * @return
     */
    int robotServiceRotateMove(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const double rotateAxisOnUserCoord[3], double rotateAngle,  bool IsBolck);

    /**
     * @brief robotServiceTrackMove   运动接口之轨迹运动
     *
     * @param subMoveMode   当subMoveMode==ARC_CIR,　CARTESIAN_MOVEP,　CARTESIAN_CUBICSPLINE,　CARTESIAN_UBSPLINEINTP时，该运动属于末端型运动；
     *                      当subMoveMode==JIONT_CUBICSPLINE，JOINT_UBSPLINEINTP时，该运动属于关节型运动；
     *
     *                      当subMoveMode==ARC_CIR　表示圆或者圆弧　　　　当圆的圈数属性（CircularLoopTimes）为０时，表示圆弧轨迹，
     * 　　　　　　　　　　　　　　　　　　　　　　　　　　　　                当圆的圈数属性（CircularLoopTimes）大于０时，表示圆轨迹。
     *
     *                      当subMoveMode==CARTESIAN_MOVEP 　　　　　表示MOVEP轨迹，需要用户这只交融半径的属性。
     *
     *                      当subMoveMode==JOINT_UBSPLINEINTP　　　 轨迹复现接口
     *
     * @param IsBolck　　　　参考robotServiceJointMove函数的解释
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceTrackMove(aubo_robot_namespace::move_track subMoveMode,     bool IsBolck);



    /**
     * @brief robotMoveLineToTargetPosition     保持当前位姿通过直线运动的方式运动到目标位置,其中目标位置是通过相对当前位置的偏移给出
     * @param userCoord                         用户坐标系　该坐标系参数（userCoord）,表示下面的偏移量参数(relativeMoveOnUserCoord)是基于该平面的。
     * @param toolInEndDesc                     工具参数　　　当没有使用工具时，将此参数设置为０；
     * @param relativeMoveOnUserCoord           基于用户坐标系下的偏移
     * @param IsBolck　　　　　　　　　　　　　　　　是否阻塞　　　参考robotServiceJointMove函数的解释
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotMoveLineToTargetPositionByRelative(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord,
                                      const aubo_robot_namespace::MoveRelative    &relativeMoveOnUserCoord,      //目标位置相对当前位置的偏移
                                      bool IsBolck);                                                  //是否阻塞


    /** 保持当前位姿通过关节运动的方式运动到目标位置   参数描述参考robotMoveLineToTargetPosition **/
    int robotMoveJointToTargetPositionByRelative(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord,
                                       const aubo_robot_namespace::MoveRelative    &relativeMoveOnUserCoord,      //目标位置相对当前位置的偏移
                                       bool IsBolck = false);                                          //是否阻塞


    /**
     * @brief robotMoveLineToTargetPosition   保持当前位姿通过直线运动的方式运动到目标位置。
     * @param userCoord                       用户坐标系　该坐标系参数（userCoord）,表示下面的目标位置(positionOnUserCoord)是基于该平面的。
     * @param positionOnUserCoord             基于用户平面表示的目标位置
     * @param toolInEndDesc                   工具参数　　　当没有使用工具时，将此参数设置为０;
     * @param IsBolck　　　　　　　　　　　　　　　是否阻塞　　　参考robotServiceJointMove函数的解释
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    //不进行运动
    int getJointAngleByTargetPositionKeepCurrentOri(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordSystem,
                                                    const aubo_robot_namespace::Pos &toolEndPositionOnUserCoord,
                                                    const aubo_robot_namespace::ToolInEndDesc   &toolInEndDesc,     //相对于用户坐标系的目标位置
                                                    double jointAngle[aubo_robot_namespace::ARM_DOF]);


    int robotMoveLineToTargetPosition(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool  &userCoord,
                                      const aubo_robot_namespace::Pos             &toolEndPositionOnUserCoord,
                                      const aubo_robot_namespace::ToolInEndDesc   &toolInEndDesc,     //相对于用户坐标系的目标位置
                                      bool IsBolck = false);                                          //是否阻塞


    /** 保持当前位姿通过关节运动的方式运动到目标位置   参数描述参考robotMoveLineToTargetPosition **/
    int robotMoveJointToTargetPosition(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool     &userCoord,
                                       const aubo_robot_namespace::Pos             &toolEndPositionOnUserCoord,
                                       const aubo_robot_namespace::ToolInEndDesc   &toolInEndDesc,    //相对于用户坐标系的目标位置
                                       bool  IsBolck = false);                                        //是否阻塞


    /**
     * @brief 开始示教
     * @param mode        示教关节:JOINT1,JOINT2,JOINT3, JOINT4,JOINT5,JOINT6,   位置示教:MOV_X,MOV_Y,MOV_Z   姿态示教:ROT_X,ROT_Y,ROT_Z
     * @param direction   运动方向   正方向true  反方向false
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceTeachStart(aubo_robot_namespace::teach_mode mode, bool direction);


    /** @brief  结束示教 */
    int robotServiceTeachStop();


    /**
     * @brief 机械臂运动控制   停止,暂停,继续
     * @param cmd    控制命令
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand cmd);

    int  robotMoveFastStop();



    /** 离线轨迹 **/
    int robotServiceOfflineTrackWaypointAppend(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);

    int robotServiceOfflineTrackWaypointAppend(const char *fileName);

    int robotServiceOfflineTrackWaypointClear ();

    int robotServiceOfflineTrackMoveStartup   (bool  IsBolck);

    int robotServiceOfflineTrackMoveStop      ();


    /** tcp转can透传　**/
    int  robotServiceEnterTcp2CanbusMode();

    int  robotServiceLeaveTcp2CanbusMode();

    int  robotServiceSetRobotPosData2Canbus(double jointAngle[aubo_robot_namespace::ARM_DOF]);

    int  robotServiceSetRobotPosData2Canbus(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);


    int  startupOfflineExcitTrajService(const char *trackFile, aubo_robot_namespace::Robot_Dyn_identify_traj type, int subtype, bool isBolck);

    int  getDynIdentifyResultsService(std::vector<int> &paramVector);


    /*******************************************************************工具接口*************************************************************
     * 工具接口：正解接口,
     *         逆解接口,
     *         工具标定接口
     *         坐标系标定接口
     * 　　　　　基座标系转用户坐标系接口
     * 　　　　　用户坐标系转基坐标系接口
     *
     *
     *
     **************************************************************************************************************************************************/

public:
    /**
     * @brief 正解　　　　　此函数为正解函数，已知关节角求对应位置的位置和姿态。
     * @param jointAngle  六个关节的关节角  　　　输入参数   单位:弧度(rad)
     * @param size        关节角数组长度   规定为6个
     * @param wayPoint    正解结果    　　　输出参数
     * 　　　结果示例: 六个关节角 {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
     *              位置 'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
     *              姿态 'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
     *
     * @return　调用成功返回ErrnoSucc;错误返回错误号  调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceRobotFk(const double *jointAngle, int size, aubo_robot_namespace::wayPoint_S &wayPoint);


    /**
     * @brief 逆解　　　　　此函数为机械臂逆解函数，根据位置信息(x,y,z)和对应位置的参考姿态(w,x,y,z)得到对应位置的关节角信息。
     *      机器人运动学方程的逆解，也称机器人的逆运动学问题。
     *      逆运动学问题：对某个机器人，当给出机器人手部（法兰盘中心）在基座标系中所处的位置和姿态时，求出其对应的关节角信息。
     * @param position   目标路点的位置   　　　单位:米   输入参数
     * @param ori        目标路点的参考姿态            　输入参数　　　　例如：可以获取当前位置位姿作为此参数，这样相当于保持当前姿态
     * @param wayPoint   计算结果----目标路点信息
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceRobotIk(const double *startPointJointAngle,const aubo_robot_namespace::Pos &position, const aubo_robot_namespace::Ori &ori, aubo_robot_namespace::wayPoint_S &wayPoint);

    int robotServiceRobotIk(const aubo_robot_namespace::Pos &position, const aubo_robot_namespace::Ori &ori, std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);


    //工具标定
    int robotServiceToolCalibration(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointPosVector,  char poseCalibMethod,
                                    aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);

    //工具标定
    int robotServiceToolCalibration(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointPosCalibVector,
                                    const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointOriCalibVector,
                                    aubo_robot_namespace::ToolKinematicsOriCalibrateMathod poseCalibMethod,
                                    aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);






    /**
     * @brief robotServiceCheckUserCoordinate    检查提供的参数是否标定出一个坐标系
     * @param coordSystem　　　　坐标系提供的参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceCheckUserCoordinate(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool  &coordSystem);

    int robotServiceUserCoordinateCalibration(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool  &coordSystem, double bInWPos[3], double bInWOri[9], double wInBPos[3]);

    int robotServiceOriMatrixToQuaternion(double eerot[], aubo_robot_namespace::Ori &result);


    /**
     * @brief baseToUserCoordinate                基座标系转用户坐标系
     *
     *        概述:  将法兰盘中心基于基座标系下的位置和姿态　转成　工具末端基于用户座标系下的位置和姿态。
     *
     *      　扩展1:  法兰盘中心可以看成是一个特殊的工具，即工具的位置为(0,0,0)姿态为（1,0,0,0）
     * 　　　　　　　  因此当工具为(0,0,0)时，相当于将法兰盘中心基于基座标系下的位置和姿态　转成　法兰盘中心基于用户座标系下的位置和姿态。
     *
     * 　　　　扩展2:  用户坐标系也可以选择成基座标系，　　即：userCoord.coordType = BaseCoordinate
     *               因此当用户平面为基座标系时，相当于将法兰盘中心基于基座标系下的位置和姿态　转成　工具末端基于基座标系下的位置和姿态，
     *               即在基座标系加工具。
     *
     * @param flangeCenterPositionOnBase          基于基座标系的法兰盘中心位置信息（x,y,z）  单位(m)
     * @param flangeCenterOrientationOnBase       基于基座标系的姿态信息(w, x, y, z)
     * @param userCoord                           用户坐标系    通过该参数确定一个坐标系
     * @param toolInEndDesc                       工具信息
     * @param toolEndPositionOnUserCoord          基于用户座标系的工具末端位置信息    输出参数
     * @param toolEndOrientationOnUserCoord       基于用户座标系的工具末端姿态信息    输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    static int baseToUserCoordinate( const aubo_robot_namespace::Pos            &flangeCenterPositionOnBase,    //基于基座标系的法兰盘中心位置信息
                                     const aubo_robot_namespace::Ori            &flangeCenterOrientationOnBase, //基于基座标系的姿态信息
                                     const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord,  //用户坐标系
                                     const aubo_robot_namespace::ToolInEndDesc  &toolInEndDesc,                 //工具参数
                                     aubo_robot_namespace::Pos                  &toolEndPositionOnUserCoord,    //基于用户座标系的工具末端位置信息
                                     aubo_robot_namespace::Ori                  &toolEndOrientationOnUserCoord  //基于用户座标系的工具末端姿态信息
                                     );


    /**
     * @brief baseToBaseAdditionalTool        　基坐标系转基座标得到工具末端点的位置和姿态
     *
     * @param flangeCenterPositionOnBase       基于基座标系的法兰盘中心位置信息
     * @param flangeCenterOrientationOnBase    基于基座标系的姿态信息
     * @param toolInEndDesc                    工具信息
     * @param toolEndPositionOnUserCoord       基于基座标系的工具末端位置信息    输出参数
     * @param toolEndOrientationOnUserCoord    基于基座标系的工具末端姿态信息    输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    static int baseToBaseAdditionalTool( const aubo_robot_namespace::Pos            &flangeCenterPositionOnBase,   //基于基座标系的法兰盘中心位置信息
                                         const aubo_robot_namespace::Ori            &flangeCenterOrientationOnBase, //基于基座标系的法兰盘姿态信息
                                         const aubo_robot_namespace::ToolInEndDesc  &toolInEndDesc,                 //工具信息
                                         aubo_robot_namespace::Pos                  &toolEndPositionOnBase,    //基于用户座标系的工具末端位置信息
                                         aubo_robot_namespace::Ori                  &toolEndOrientationOnBase  //基于用户座标系的工具末端姿态信息);
                                         );


    /**
     * @brief userToBaseCoordinate                用户坐标系位置和姿态信息转基座标系下位置和姿态信息
     *
     *        概述:  将工具末端基于用户座标系下的位置和姿态　转成　法兰盘中心基于基座标系下的位置和姿态。
     *
     *      　扩展1:  法兰盘中心可以看成是一个特殊的工具，即工具的位置为(0,0,0)姿态为（1,0,0,0）
     * 　　　　　　　  因此当工具工具的位置为(0,0,0)姿态为（1,0,0,0）时，表示toolEndPositionOnUserCoord和toolEndOrientationOnUserCoord是无工具的。
     *
     * 　　　　扩展2:  用户坐标系也可以选择成基座标系，　　即：userCoord.coordType = BaseCoordinate
     *               因此当用户平面为基座标系时，相当于 将工具末端基于基座标系下的位置和姿态　转成　法兰盘中心基于基座标系下的位置和姿态，
     *               即在基座标系去工具.
     *
     *       扩展３:  利用该函数和逆解组合实现　　　　　当用户提供自定义坐标系（特殊为基座标系）下工具末端的位置和姿态　得到　基座标系下法兰盘中心的位置和姿态
     * 　　　　　　　　然后在逆解，得到目标路点。
     *
     * @param toolEndPositionOnUserCoord          基于用户座标系的工具末端位置信息
     * @param toolEndOrientationOnUserCoord       基于用户座标系的工具末端姿态信息
     * @param userCoord                           用户坐标系    通过该参数确定一个坐标系
     * @param toolInEndDesc                       工具信息
     * @param flangeCenterPositionOnBase          基于基座标系的法兰盘中心位置信息
     * @param flangeCenterOrientationOnBase       基于基座标系的姿态信息
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     *
     * 注：这个的坐标系转换不支持末端系　　即不支持userCoord==EndCoordinate,如果userCoord==EndCoordinate会报参数错误(ErrCode_ParamError)
     */
    static int userToBaseCoordinate( const aubo_robot_namespace::Pos            &toolEndPositionOnUserCoord,    //基于用户座标系的工具末端位置信息
                                     const aubo_robot_namespace::Ori            &toolEndOrientationOnUserCoord, //基于用户座标系的工具末端姿态信息
                                     const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord,  //用户坐标系
                                     const aubo_robot_namespace::ToolInEndDesc  &toolInEndDesc,                 //工具信息
                                     aubo_robot_namespace::Pos                  &flangeCenterPositionOnBase,    //基于基座标系的法兰盘中心位置信息
                                     aubo_robot_namespace::Ori                  &flangeCenterOrientationOnBase  //基于基座标系的法兰盘中心姿态信息
                                     );


    /**
     * @brief userCoordPointToBasePoint    将空间内一个基于用户坐标系的位置信息(x,y,z)　转换成基于　基座标下的位置信息(x,y,z)
     *
     * @param userCoordPoint        用户坐标系下的位置信息 x,y,z（必须的基于下面参数提供的坐标系下的）
     * @param userCoordSystem　　　　用户坐标系描述    通过该参数确定一个坐标系　不能为末端坐标系　如果userCoord==EndCoordinate会报参数错误(ErrCode_ParamError)
     * @param basePoint　　　　　　　　基于基坐标系下的位置信息x,y,z
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     *
     * 注：这个的坐标系转换不支持末端系　　即不支持userCoord==EndCoordinate,如果userCoord==EndCoordinate会报参数错误(ErrCode_ParamError)
     */
    static int userCoordPointToBasePoint(const aubo_robot_namespace::Pos &userCoordPoint,
                                         const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem,
                                         aubo_robot_namespace::Pos &basePoint);


    //法兰盘姿态转成工具姿态
    static int endOrientation2ToolOrientation(aubo_robot_namespace::Ori &tcpOriInEnd, const aubo_robot_namespace::Ori &endOri,  aubo_robot_namespace::Ori &toolOri);

    //工具姿态转成法兰盘姿态
    static int toolOrientation2EndOrientation(aubo_robot_namespace::Ori &tcpOriInEnd, const aubo_robot_namespace::Ori &toolOri, aubo_robot_namespace::Ori &endOri);


    static int getTargetWaypointByPosition(const aubo_robot_namespace::wayPoint_S       &sourceWayPointOnBaseCoord,
                                           const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem,
                                           const aubo_robot_namespace::Pos              &toolEndPosition,
                                           const aubo_robot_namespace::ToolInEndDesc    &toolInEndDesc,
                                           aubo_robot_namespace::wayPoint_S             &targetWayPointOnBaseCoord);



    /**
     * @brief quaternionToRPY     四元素转欧拉角
     * @param ori　　　　　姿态的四元素表示方法
     * @param rpy　　　　　姿态的欧拉角表示方法
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int quaternionToRPY(const aubo_robot_namespace::Ori &ori, aubo_robot_namespace::Rpy &rpy);

    /**
     * @brief RPYToQuaternion    欧拉角转四元素
     * @param rpy       姿态的欧拉角表示方法
     * @param ori       姿态的四元素表示方法
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int RPYToQuaternion(const aubo_robot_namespace::Rpy &rpy, aubo_robot_namespace::Ori &ori);


    /**
     * @brief getErrDescByCode  根据错误号获取错误信息
     * @param code　　错误号
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    std::string getErrDescByCode(aubo_robot_namespace::RobotErrorCode code);



    /**********************************************************************************************************************************************
     ************************************************************机械臂控制接口**********************************************************************
     **********************************************************************************************************************************************/
public:

    /**
     * @brief 机械臂控制
     * @param cmd   控制命令
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  rootServiceRobotControl(const aubo_robot_namespace::RobotControlCommand cmd);


    /**
     * @brief 设置机械臂的电源状态
     * @param value
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServicePowerControl(bool value);

    int  robotServiceReleaseBrake();



    /**
     * @brief rootServiceRobotStartup    启动机械臂-----该操作会完成机械臂的上电，松刹车，设置碰撞等级，设置动力学参数等功能。
     * @param toolDynamicsParam　　　　　　动力学参数
     * @param collisionClass　　　　　　　　碰撞等级
     * @param readPose                   是否允许读取位置　　　默认是true
     * @param staticCollisionDetect      
     * @param boardBaxAcc
     * @param result
     * @param IsBolck
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int rootServiceRobotStartup(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam, uint8 collisionClass, bool readPose,
                                bool staticCollisionDetect, int boardBaxAcc, aubo_robot_namespace::ROBOT_SERVICE_STATE &result, bool IsBolck = true);


    /**
     * @brief    关机
     * @param IsBolck        是否阻塞
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int  robotServiceRobotShutdown(bool IsBolck = true);


    /**********************************************************************************************************************************************
     **********************************************************************************************************************************************
     ************************************************************关于末端工具的接口*******************************************************************
     **********************************************************************************************************************************************
     **********************************************************************************************************************************************/
public:
    /** 设置无工具的动力学参数　**/
    int  robotServiceSetNoneToolDynamicsParam();

    /** 设置工具的动力学参数　 **/
    int  robotServiceSetToolDynamicsParam(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    /** 获取工具的动力学参数　 **/
    int  robotServiceGetToolDynamicsParam(aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    /** 设置无工具运动学参数　 **/
    int  robotServiceSetNoneToolKinematicsParam();

    /** 设置工具的运动学参数　 **/
    int  robotServiceSetToolKinematicsParam(const aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);

    /** 获取工具的运动学参数　 **/
    int  robotServiceGetToolKinematicsParam(aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);



    /**********************************************************************************************************************************************
     ************************************************************机械臂相关参数设置与获取的接口*********************************************************
     **********************************************************************************************************************************************/
public:

    /**
     * @brief robotServiceGetRobotWorkMode　　　获取当前机械臂模式
     * @param mode   输出参数　　仿真或真实的枚举类型
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetRobotWorkMode(aubo_robot_namespace::RobotWorkMode &mode);

    /**
     * @brief robotServiceSetRobotWorkMode  设置当前机械臂模式   仿真或真实
     * @param mode　　　仿真或真实的枚举类型
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetRobotWorkMode(aubo_robot_namespace::RobotWorkMode mode);

    /**
     * @brief robotServiceSetRobotWorkMode  获取重力分量
     * @param mode　　　输出参数　　仿真或真实的枚举类型
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetRobotGravityComponent(aubo_robot_namespace::RobotGravityComponent  &gravityComponent);

    //业务接口: 获取当前碰撞等级
    int robotServiceGetRobotCollisionCurrentService(int &collisionGrade);

    //业务接口: 设置碰撞等级
    int robotServiceSetRobotCollisionClass(int grade);

    //业务接口:获取设备信息
    int robotServiceGetRobotDevInfoService(aubo_robot_namespace::RobotDevInfo &devInfo);

    //设置最大加速度
    int robotServiceSetRobotMaxACC(int maxAcc);

    //碰撞恢复
    int robotServiceCollisionRecover();

    int robotServiceGetRobotCurrentState(aubo_robot_namespace::RobotState &state);

    int robotServiceGetMacCommunicationStatus(bool  &value);


    /**
     * @brief robotServiceGetIsRealRobotExist  获取是否存在真实机械臂
     * @param value   输出参数　　存在为true,不存在为false
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetIsRealRobotExist(bool &value);

    /**
     * @brief robotServiceGetJoint6Rotate360EnableFlag   获取６关节旋转360使能标志
     * @param value
     * @return
     */
    int robotServiceGetJoint6Rotate360EnableFlag(bool &value);

    /**
     * @brief robotServiceGetRobotJointStatus   获取机械臂关节状态
     * @param jointStatus    关节角缓冲区　　输出参数
     * @param size           关节角缓冲区长度
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetRobotJointStatus(aubo_robot_namespace::JointStatus *jointStatus, int size);


    /**
     * @brief robotServiceGetRobotDiagnosisInfo   获取机械臂诊断信息
     * @param robotDiagnosisInfo   输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetRobotDiagnosisInfo(aubo_robot_namespace::RobotDiagnosis &robotDiagnosisInfo);


    /**
     * @brief robotServiceGetJointAngleInfo   获取机械臂当前关节角信息
     * @param jointParam    输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetJointAngleInfo(aubo_robot_namespace::JointParam &jointParam);


    /**
     * @brief robotServiceGetCurrentWaypointInfo   获取机械臂当前路点信息
     * @param wayPoint   输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetCurrentWaypointInfo(aubo_robot_namespace::wayPoint_S &wayPoint);

    /*****************************************************************************************************************************************************/
    /*      安全ＩＯ相关                                                                                                                      */
    /*                                                                                                                                      */
    /****************************************************************************************************************************************************/

    int robotServiceSetRobotAtOriginPose();

    /**
     * @brief 通知接口板上位机暂停状态
     * @param data
     * @return
     */
    int robotServiceSetRobotOrpePause(uint8 data);

    /**
     * @brief 通知接口板上位机停止状态
     * @param data
     * @return
     */
    int robotServiceSetRobotOrpeStop(uint8 data);

    /**
     * @brief 通知接口板上位机错误
     * @param 16个字节的错误数据，每个错误占一个bit
     * @return
     */
    int robotServiceSetRobotOrpeError(uint8 data[], int len);

    /**
     * @brief 解除系统紧急停止输出信号 0-无动作 1-解除
     * @param data
     * @return
     */
    int robotServiceClearSystemEmergencyStop(uint8 data);

    /**
     * @brief 解除缩减模式错误 0-无效 1-解除
     * @param data
     * @return
     */
    int robotServiceClearReducedModeError(uint8 data);

    /**
     * @brief 防护重置成功 0-无动作 1-解除
     * @param data
     * @return
     */
    int robotServiceRobotSafetyguardResetSucc(uint8 data);




    /**********************************************************************************************************************************************
     ************************************************************关于接口板IO的接口**********************************************************************
     **********************************************************************************************************************************************/
public:
    /**
     * @brief 获取接口板指定IO集合的配置信息
     * @param ioType         IO类型的集合    输入参数
     * @param configVector   IO配置信息的集合 输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetBoardIOConfig(const std::vector<aubo_robot_namespace::RobotIoType> &ioType, std::vector<aubo_robot_namespace::RobotIoDesc> &configVector);


    /**
     * @brief 获取接口板指定IO集合的状态信息
     * @param ioType
     * @param statusVector
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetBoardIOStatus(const std::vector<aubo_robot_namespace::RobotIoType>  ioType, std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);

    /**
     * @brief 根据接口板IO类型和名称设置IO状态
     * @param type     IO类型
     * @param name     IO名称
     * @param value    IO状态
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double value);

    /**
     * @brief 根据接口板IO类型和地址设置IO状态
     * @param type    IO类型
     * @param addr    IO地址
     * @param value   IO状态
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotIoType type, int    addr,      double value);


    /**
     * @brief 根据接口板IO类型和名称获取IO状态
     * @param type     IO类型
     * @param name     IO名称
     * @param value    IO状态
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double &value);

    /**
     * @brief 根据接口板IO类型和地址获取IO状态
     * @param type    IO类型
     * @param addr    IO地址
     * @param value   IO状态
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotIoType type, int    addr,      double &value);



    /**
     * @brief 返回当前机械臂是否运行在联机模式
     * @param isOnlineMode  true：联机 false：非联机
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceIsOnlineMode(bool &isOnlineMode);

    /**
     * @brief 返回当前机械臂是否运行在联机主模式
     * @param isOnlineMode  true：联机主模式 false：联机从模式
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceIsOnlineMasterMode(bool &isOnlineMasterMode);



public:
    //业务接口: 获取机械臂安全配置
    int  robotServiceGetRobotSafetyConfig(aubo_robot_namespace::RobotSafetyConfig &safetyConfig);

    //业务接口: 获取机械臂安全配置
    int  robotServiceSetRobotSafetyConfig(const aubo_robot_namespace::RobotSafetyConfig &safetyConfig);

    //业务接口: 获取机械臂安全状态
    int  robotServiceGetOrpeSafetyStatus(aubo_robot_namespace::OrpeSafetyStatus &safetyStatus);






    /**********************************************************************************************************************************************
     ************************************************************关于tool IO的接口**********************************************************************
     **********************************************************************************************************************************************/
public:

    /**
     * @brief 设置工具端电源电压类型
     * @param type   电源电压类型
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetToolPowerVoltageType  (aubo_robot_namespace::ToolPowerType type);


    /**
     * @brief  获取工具端电源电压类型
     * @param  输出参数   接口调用成功后返回的电源电压类型
     * @return
     */
    int robotServiceGetToolPowerVoltageType  (aubo_robot_namespace::ToolPowerType &type);


    /**
      * @brief 获取工具端的电源电压
      * @param value   输出参数
      * @return
      */
    int robotServiceGetToolPowerVoltageStatus(double &value);

    /**
     * @brief robotServiceSetToolPowerTypeAndDigitalIOType  设置工具端电源电压类型and所有数字量IO的类型
     * @param type
     * @param io0
     * @param io1
     * @param io2
     * @param io3
     * @return
     */
    int robotServiceSetToolPowerTypeAndDigitalIOType(aubo_robot_namespace::ToolPowerType type,
                                                     aubo_robot_namespace::ToolIOType io0,
                                                     aubo_robot_namespace::ToolIOType io1,
                                                     aubo_robot_namespace::ToolIOType io2,
                                                     aubo_robot_namespace::ToolIOType io3);


    /**
     * @brief 设置工具端数字量IO的类型    输入或者输出
     * @param 地址
     * @param 类型  输入或者输出
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetToolDigitalIOType(aubo_robot_namespace::ToolDigitalIOAddr addr,  aubo_robot_namespace::ToolIOType type);


    /**
     * @brief 获取工具端所有数字量IO的状态
     * @param statusVector    输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetAllToolDigitalIOStatus(std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);

    /**
     * @brief 根据地址设置工具端数字量IO的状态
     * @param addr    IO地址
     * @param value   IO状态
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetToolDOStatus          (aubo_robot_namespace::ToolDigitalIOAddr addr, aubo_robot_namespace::IO_STATUS value);

    /**
     * @brief 根据名称设置工具端数字量IO的状态
     * @param addr    IO地址
     * @param value   IO状态
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceSetToolDOStatus(std::string name,        aubo_robot_namespace::IO_STATUS value);

    /**
     * @brief 根据名称获取工具端IO的状态
     * @param name     IO名称
     * @param value    IO状态  输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetToolIoStatus(std::string name, double &value);

    /**
     * @brief 获取工具端所有AI的状态
     * @param 输出参数
     * @return　调用成功返回ErrnoSucc;错误返回错误号
     */
    int robotServiceGetAllToolAIStatus       (std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);



public:  //固件升级
    int  robotServiceUpdateRobotBoardFirmware(aubo_robot_namespace::update_board_firmware_cmd cmd, const void *data, uint16 length);

    int  robotServiceGetBoardFirmwareUpdateResultService(bool &value);

    int  robotServiceGetRobotEthernetDeviceName(std::string &ethernetDeviceName);

public:
    /**
     * @brief 设置关节碰撞补偿（范围0.00~0.51度）
     * @param jointOffset
     * @return
     */
    int robotServiceSetRobotJointOffset(aubo_robot_namespace::RobotJointOffset &jointOffset);


public:  //传送带跟踪

    int robotServiceSetConveyorEncoderReset(void);

    /**
     * @brief 启动传送带
     * @return
     */
    int robotServiceSetConveyorStartup(void);

    /**
     * @brief 停止传送带
     * @return
     */
    int robotServiceSetConveyorStop(void);

    /**
     * @brief 设置传送带方向
     * @param dir
     * @return
     */
    int robotServiceSetConveyorDir(int dir);


    /**
     * @brief 设置手眼标定结果关系
     * @param robotCameraCalib
     */
    int robotServiceSetRobotCameraCalib(const aubo_robot_namespace::RobotCameraCalib &robotCameraCalib);


    /**
     * @brief 设置传送带线速度
     * @param conveyorVelc (米/秒）
     */
    int robotServiceSetConveyorVelc(const double conveyorVelc);


    /**
     * @brief 设置编码器距离关系
     * @param encoderValPerMeter 编码器距离关系（编码器脉冲个数/米)
     */
    int robotServiceSetEncoderValPerMeter(const uint32_t &encoderValPerMeter);


    /**
     * @brief 设置传送带起始窗口上限
     * @param startWindowUpstream　单位：米
     */
    int robotServiceSetStartWindowUpstream(double startWindowUpstream);


    /**
     * @brief 设置传送带起始窗口下限
     * @param startWindowDownstream 单位：米
     */
    int robotServiceSetStartWindowDownstream(double startWindowDownstream);

    /**
     * @brief 设置传送带跟踪轨迹下限 单位：米
     * @param trackDownstream
     */
    int robotServiceSetConveyorTrackDownstream(double trackDownstream);


    int robotServiceAppendObject2ConveyorTrackQueue(const aubo_robot_namespace::Pos &objectPos, const aubo_robot_namespace::Ori &objectOri, uint32_t timestamp);


    int robotServiceEnableConveyorTrack();


    int robotServiceGetConveyorEncoderVal(int &value);

    //设置传送带跟踪的最大速度
    int robotServiceSetRobotConveyorTrackMaxVelc(double robotConveyorTrackMaxVelc);

    //设置传送带跟踪的最大加速度
    int robotServiceSetRobotConveyorTrackMaxAcc(double robotConveyorTrackMaxAcc);

    //设置传送带跟踪的系统延时时间
    int robotServiceSetRobotConveyorSystemDelay(double robotConveyorSystemDelay);

    //设置机械臂工具
    int robotServiceSetRobotTool(const aubo_robot_namespace::ToolInEndDesc &robotTool);

public:
    int robotServiceSetWeaveMoveParameters(const aubo_robot_namespace::WeaveMove &weaveMove);


public:
    int robotServiceSetRobotRecognitionParam(const aubo_robot_namespace::RobotRecongnitionParam &param);

    int robotServiceGetRobotRecognitionParam(int type, aubo_robot_namespace::RobotRecongnitionParam &param);


private:   /**SDK内部使用 开发者不需要关心**/

    RobotControlServices             *m_robotBaseService;

    RobotIoService                   *m_robotIoService;

    RobotMoveService                 *m_robotMoveService;

    RobotConveyorTrack               *m_robotConveyorTrack;

    robotOtherService                *m_robotOtherService;

private:

    void                              *m_realTimeJointStatusCallbackArg;
    RealTimeJointStatusCallback        m_realTimeJointStatusCallback;
    pthread_mutex_t                    m_realTimeJointStatusCallbackChangeMutex;

    void                              *m_realTimeRoadPointCallbackArg;
    RealTimeRoadPointCallback          m_realTimeRoadPointCallback;
    pthread_mutex_t                    m_realTimeRoadPointCallbackChangeMutex;

    void                              *m_robotEventCallbackArg;
    RobotEventCallback                 m_robotEventCallback;
    pthread_mutex_t                    m_robotEventCallbackChangeMutex;

    void                              *m_robotEndSpeedCallbackArg;
    RealTimeEndSpeedCallback           m_robotEndSpeedCallback;
    pthread_mutex_t                    m_robotEndSpeedCallbackChangeMutex;
};



#endif // SERVICEINTERFACE_H
