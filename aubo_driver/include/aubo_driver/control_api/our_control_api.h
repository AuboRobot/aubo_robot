#ifndef OUR_CONTROL_API_V1_H
#define OUR_CONTROL_API_V1_H

#if defined(__WIN32__) || defined (WIN32)
#define AUBOAPI_DLLSHARED_EXPORT __declspec(dllexport)
#else
#define AUBOAPI_DLLSHARED_EXPORT
#endif


#include "metaType.h"


extern "C"
{

/***connect to server***/
AUBOAPI_DLLSHARED_EXPORT int  login (const char *ip, int port, const char *userName, const char *password);

/***disconnect ***/
AUBOAPI_DLLSHARED_EXPORT int  logout_system();

/***register for event callback function***/
int  register_event_push_callback     (EventPushCallback ptr);

/***register for realtime position callback function***/
int  register_roadPoint_event_callback(RealTimeRoadPointEventCallback ptr);

/***enable realtime position callback event***/
int  enable_realtime_roadpoint_event();

/***disable realtime position callback event***/
int  disenable_realtime_roadpoint_event();


/***joint move in joint space***/
AUBOAPI_DLLSHARED_EXPORT int  robot_moveJ(robot_move_profile *move_profile, const our_robot_road_point *roadPoint);

/***line move in tool pose space***/
AUBOAPI_DLLSHARED_EXPORT int  robot_moveL(robot_move_profile *move_profile, const our_robot_road_point *roadPoint);

/***track move in tool pose space***/
AUBOAPI_DLLSHARED_EXPORT int  robot_moveP(robot_move_profile *move_profile, const our_robot_road_point *point_array,int len);

/***line move to position（x,y,z）***/
AUBOAPI_DLLSHARED_EXPORT int  robot_moveLTo(robot_move_profile *moveProfile, double targetX, double targetY, double targetZ);

/***control the servo move directly,with out plan***/
AUBOAPI_DLLSHARED_EXPORT int  robot_servoj(double *jointAngle, int len);

/***joint move in teach mode***/
AUBOAPI_DLLSHARED_EXPORT int  robot_joint_move_teach(bool dir, int jointId, int speed);

/***joint step move in teach mode***/
AUBOAPI_DLLSHARED_EXPORT int  robot_joint_step_move_teach(bool dir, int jointId, int speed, double jointStep);

/***point move in teach mode***/
AUBOAPI_DLLSHARED_EXPORT int  robot_point_move_teach(robot_coord_type coordinage, robot_moveway move_code, int speed);

/***point step move in teach mode***/
AUBOAPI_DLLSHARED_EXPORT int  robot_point_step_move_teach(robot_coord_type coordinage, robot_moveway move_code, int speed, double step);

/***inverse kinematics***/
AUBOAPI_DLLSHARED_EXPORT int  robot_ik(double targetX,  double targetY, double targetZ, double *currentJointPos, our_robot_road_point *roadPoint);

/***forward kinematics***/
AUBOAPI_DLLSHARED_EXPORT int  robot_fk(double *jointPos, our_robot_road_point *roadPoint);

/***stop move***/
AUBOAPI_DLLSHARED_EXPORT int  robot_move_stop();





/***get project number***/
AUBOAPI_DLLSHARED_EXPORT int get_project_count(int *count);

/***get project list***/
AUBOAPI_DLLSHARED_EXPORT int get_project_list(our_project_info *projects, int *len);

/***load a project***/
AUBOAPI_DLLSHARED_EXPORT int project_load(const our_control_project_load *projectInfo);

/***control project(run,stop,pause)****/
AUBOAPI_DLLSHARED_EXPORT int project_control(our_project_control controlCommand);




/***get system state***/
AUBOAPI_DLLSHARED_EXPORT int  get_robot_system_status(our_control_robot_system_status *robotSystemStatus);

/***get joint state**/
AUBOAPI_DLLSHARED_EXPORT int  get_robot_joint_status (our_control_joint_status *status, int len);

/***get device info***/
AUBOAPI_DLLSHARED_EXPORT int  get_robot_device_info  (RobotDeviceInfo *deviceInfo);

/***get robot move state***/
AUBOAPI_DLLSHARED_EXPORT int  get_robot_run_status   (our_robot_status *status);

/***get robot current position***/
AUBOAPI_DLLSHARED_EXPORT int  get_robot_position     (our_robot_road_point *pos);

/***get robot end speed***/
AUBOAPI_DLLSHARED_EXPORT int  get_robot_end_speed    (double *speed);



/***set robot power***/
AUBOAPI_DLLSHARED_EXPORT int set_robot_power(int value);

/***set collision level***/
AUBOAPI_DLLSHARED_EXPORT int set_collision_grade(int value);

/***set robot mode***/
AUBOAPI_DLLSHARED_EXPORT int set_robot_mode(our_control_robot_mode mode);

/***get rbot mode***/
AUBOAPI_DLLSHARED_EXPORT int get_robot_mode(our_control_robot_mode *mode);

/***set tcp center***/
AUBOAPI_DLLSHARED_EXPORT int set_robot_tcp_param (double x, double y, double z, double payload );

/***get tcp center***/
AUBOAPI_DLLSHARED_EXPORT int get_robot_tcp_param (double *x,double *y,double *z,double *payload);

/***set brake***/
AUBOAPI_DLLSHARED_EXPORT int set_robot_brake();

/***release brake***/
AUBOAPI_DLLSHARED_EXPORT int set_robot_brake_release();

/***warning for over speed***/
AUBOAPI_DLLSHARED_EXPORT int set_rverspeed_warning();

/***recover from over speed waring***/
AUBOAPI_DLLSHARED_EXPORT int set_overspeed_warning_recover();

/***enable force control***/
AUBOAPI_DLLSHARED_EXPORT int set_enable_force_control();

/***disable force control***/
AUBOAPI_DLLSHARED_EXPORT int set_disable_force_control();

/***enable read pose function***/
AUBOAPI_DLLSHARED_EXPORT int set_enable_read_pose();

/***disable read pose function***/
AUBOAPI_DLLSHARED_EXPORT int set_disable_read_pose();

/***set fixed pose changed***/
AUBOAPI_DLLSHARED_EXPORT int set_mounting_pose_changed();

/***set fixed pose unchanged***/
AUBOAPI_DLLSHARED_EXPORT int set_mounting_pose_unchanged();

/***enable static collision detect function***/
AUBOAPI_DLLSHARED_EXPORT int set_enable_static_collision_detect();

/***disable static collision detect function***/
AUBOAPI_DLLSHARED_EXPORT int set_disable_static_collision_detect();




/***set single io status***/
AUBOAPI_DLLSHARED_EXPORT int set_single_io_status( our_contorl_io_type  ioType, our_contorl_io_mode ioMode, int ioIndex, double ioValue);

/***get single io status***/
AUBOAPI_DLLSHARED_EXPORT int get_single_io_status( our_contorl_io_type  ioType, our_contorl_io_mode ioMode, int ioIndex, double *ioValue);

/***set one or more io status***/
AUBOAPI_DLLSHARED_EXPORT int set_io_status( const our_control_io_status *ioStatusArray, int len);

/***get one or more io status***/
AUBOAPI_DLLSHARED_EXPORT int get_io_status( our_control_io_status *ioStatusArray, int len);



/***init move profile***/
AUBOAPI_DLLSHARED_EXPORT void init_move_profile_for_script();

/***enable scurve***/
AUBOAPI_DLLSHARED_EXPORT void set_scurve(int value);

/***set move relative offset***/
AUBOAPI_DLLSHARED_EXPORT void set_relative_offset(double x,double y,double z);

/***disable move relative offset***/
AUBOAPI_DLLSHARED_EXPORT void disable_relative_offset();

/***set feature(base,end)***/
AUBOAPI_DLLSHARED_EXPORT void set_feature(int type);

/***set user feature***/
AUBOAPI_DLLSHARED_EXPORT void set_user_feature(double waypoint1[],double waypoint2[],double waypoint3[],int type);

/***add waypoint for movep***/
AUBOAPI_DLLSHARED_EXPORT void add_waypoint_for_moveP(const double pos[], int count);

/***enable or disable block function,which mean whether wait for the move finish signal.***/
AUBOAPI_DLLSHARED_EXPORT void set_enable_block(int value);

/***movej for script***/
AUBOAPI_DLLSHARED_EXPORT int robot_moveJ_for_script(double pos[], int count, double acc, double velc);

/***movel for script***/
AUBOAPI_DLLSHARED_EXPORT int robot_moveL_for_script(double pos[], int count, double acc, double velc);

/***movep for script***/
AUBOAPI_DLLSHARED_EXPORT int robot_moveP_for_script(double acc, double velc, double blendRadius, int trackMode);

/***moveto for script***/
AUBOAPI_DLLSHARED_EXPORT int robot_movelTo_for_script(double targetX, double targetY, double targetZ, double acc, double velc);

}

#endif // OUR_CONTROL_API_V1_H
