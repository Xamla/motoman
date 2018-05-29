cmake_minimum_required(VERSION 2.8.3)
project(motoman_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs trajectory_msgs actionlib_msgs industrial_msgs genmsg message_generation)

add_message_files(
  FILES
  DynamicJointsGroup.msg
  DynamicJointPoint.msg
  DynamicJointTrajectory.msg
  DynamicJointState.msg
  DynamicJointTrajectoryFeedback.msg
  JobPosData.msg
  MovCtrlData.msg
  UserVarPrimitive.msg
  )

add_service_files(
  FILES
  CmdJointTrajectoryEx.srv
  WriteIO.srv
  ReadIO.srv
  ListJobs.srv
  StartJob.srv
  WaitForJobEnd.srv
  SetMasterJob.srv
  SetCurJob.srv
  GetCurJob.srv
  ResetAlarm.srv
  Hold.srv
  GetMasterJob.srv
  DeleteJob.srv
  CancelError.srv
  PutUserVars.srv
  GetUserVars.srv
  SkillEnd.srv
  SkillRead.srv
  )

add_action_files(
  DIRECTORY action
  FILES StartJob.action
)

generate_messages(
  DEPENDENCIES trajectory_msgs std_msgs actionlib_msgs industrial_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs trajectory_msgs actionlib_msgs industrial_msgs genmsg
)

