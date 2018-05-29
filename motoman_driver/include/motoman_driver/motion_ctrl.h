/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifndef MOTOMAN_DRIVER_MOTION_CTRL_H
#define MOTOMAN_DRIVER_MOTION_CTRL_H

#include <boost/thread/thread.hpp>
#include "simple_message/smpl_msg_connection.h"
#include "motoman_driver/simple_message/motoman_motion_ctrl.h"
#include "motoman_driver/simple_message/motoman_motion_reply.h"
#include "motoman_driver/simple_message/motoman_read_single_io_reply.h"
#include "motoman_driver/simple_message/motoman_write_single_io_reply.h"
#include "motoman_driver/simple_message/motoman_simple_rpc_reply.h"
#include "motoman_driver/simple_message/motoman_simple_rpc.h"
#include "motoman_msgs/PutUserVars.h"
#include "motoman_msgs/GetUserVars.h"
#include "motoman_msgs/UserVarPrimitive.h"
#include "motoman_msgs/SkillEnd.h"
#include "motoman_msgs/SkillRead.h"

namespace motoman
{
namespace motion_ctrl
{
using industrial::smpl_msg_connection::SmplMsgConnection;
using motoman::simple_message::motion_reply::MotionReply;
typedef motoman::simple_message::motion_ctrl::MotionControlCmd MotionControlCmd;
using motoman::simple_message::io_ctrl_reply::ReadSingleIOReply;
using motoman::simple_message::io_ctrl_reply::WriteSingleIOReply;
using motoman::simple_message::rpc_ctrl::SimpleRpc;
using motoman::simple_message::rpc_ctrl_reply::SimpleRpcReply;

/**
 * \brief Wrapper class around Motoman-specific motion control commands
 */

enum FILEOPERATION_ERROR_CODE
{
  SUCCESS = 0,                       //Normal end
  NO_SPECIFIED_FILE = -1,            // Error (No specified file or descriptor)
  UNABLE_TO_USE_FC_API = -2,         // Unable to use File Control API and Existing File Access API
  SPECIFIED_DRIVER_NAME_INVALID = -3 //Specified drive name is invalid.
};

struct _MP_TASK_SEND_DATA
{
  short sTaskNo;
  char reserved[2];
} __attribute__((__packed__));
typedef struct _MP_TASK_SEND_DATA MP_TASK_SEND_DATA;

struct _MP_JOB_NAME_RSP_DATA
{
  char cJobName[33];
  char reserved[3];
} __attribute__((__packed__));
typedef struct _MP_JOB_NAME_RSP_DATA MP_JOB_NAME_RSP_DATA;

struct _MP_CUR_JOB_RSP_DATA
{
  short usJobLine;
  short usStep;
  char cJobName[33];
  char reserved[3];
} __attribute__((__packed__));
typedef struct _MP_CUR_JOB_RSP_DATA MP_CUR_JOB_RSP_DATA;

struct _MP_START_JOB_SEND_DATA
{
  short sTaskNo;
  char cJobName[33];
  char reserved[5];
} __attribute__((__packed__));
typedef struct _MP_START_JOB_SEND_DATA MP_START_JOB_SEND_DATA;

struct _MP_HOLD_SEND_DATA
{
  short sHold;
  char reserved[2];
} __attribute__((__packed__));
typedef struct _MP_HOLD_SEND_DATA MP_HOLD_SEND_DATA;
struct _MP_STD_RSP_DATA
{
  short err_no;
  char reserved[2];
} __attribute__((__packed__));
typedef struct _MP_STD_RSP_DATA MP_STD_RSP_DATA;

struct _MP_WAIT_JOB_SEND_DATA
{
  short sTaskNo;
  short sTime;
} __attribute__((__packed__));
typedef struct _MP_WAIT_JOB_SEND_DATA MP_WAIT_JOB_SEND_DATA;

struct _MP_CUR_JOB_SEND_DATA
{
  short usJobLine;
  char cJobName[33];
  char reserved[5];
} __attribute__((__packed__));
typedef struct _MP_CUR_JOB_SEND_DATA MP_CUR_JOB_SEND_DATA;

struct _MP_DELETE_JOB_SEND_DATA
{
  char cJobName[33];
  char reserved[7];
} __attribute__((__packed__));
typedef struct _MP_DELETE_JOB_SEND_DATA MP_DELETE_JOB_SEND_DATA;

struct _MP_JOB_POS_DATA
{
  long ctrl_grp; /* control group CTRLG_T*/
  long posType;  /* position data type */
  long varIndex; /* position variable number CTRLG_T*/
  long attr;     /* data attribute */
  long attrExt;  /* data attribute(expansion) */
  long pos[8];   /*position data */
} __attribute__((__packed__));
typedef struct _MP_JOB_POS_DATA MP_JOB_POS_DATA;

struct _MP_MOV_CTRL_DATA
{
  char intpType;  /* interpolation type */
  char intpKind;  /* move instruction type*/
  char speedType; /* speed type */
  char reserved0; /* speed setting value (VJ=,V=,VR=,VE=,VS=) */
  long speedValue;
  long posNum;                /* position data number in move instruction */
  MP_JOB_POS_DATA posData[3]; /* position data structure */
  char reserved1[32];
} __attribute__((__packed__));
typedef struct _MP_MOV_CTRL_DATA MP_MOV_CTRL_DATA;

struct _MP_JOB_STEP_RSP_DATA
{
  short err_no; /* error number */
  char reserved0[2];
  long attr;        /* step attribute (future function) */
  char comment[33]; /* comment */
  char reserved1[3];
  long movCtrlDataNum;             /* move instruction number in step */
  MP_MOV_CTRL_DATA movCtrlData[4]; /* data structure for every move instruction*/
  long posLevel;                   /* setting value of positioning level (PL=) */
  long cornerRadius;               /* setting value of conner radius (CR=) */
  long accValue;                   /* specified value of acceleration (ACC=) */
  long decValue;                   /* specified value of deceleration (DEC=) */
  char reserved2[32];
} __attribute__((__packed__));
typedef struct _MP_JOB_STEP_RSP_DATA MP_JOB_STEP_RSP_DATA;

struct _MP_MASTER_JOB_SEND_DATA
{
  short sTaskNo;
  char cJobName[33];
  char reserved[5];
} __attribute__((__packed__));
typedef struct _MP_MASTER_JOB_SEND_DATA MP_MASTER_JOB_SEND_DATA;

struct _MP_JOB_STEP_NO_SEND_DATA
{
  short usStep;
  char reserved0[2];
  char cJobName[33];
  char reserved1[3];
} __attribute__((__packed__));
typedef struct _MP_JOB_STEP_NO_SEND_DATA MP_JOB_STEP_NO_SEND_DATA;

typedef unsigned char MP_B_VAR_BUFF;
typedef short MP_I_VAR_BUFF;
typedef long MP_D_VAR_BUFF;
typedef float MP_R_VAR_BUFF;
typedef char MP_S_VAR_BUFF[16];
struct _MP_USR_VAR_INFO
{
  int var_type;
  int var_no;
  union {
    MP_B_VAR_BUFF b;
    MP_I_VAR_BUFF i;
    MP_D_VAR_BUFF d;
    MP_R_VAR_BUFF r;
    MP_S_VAR_BUFF s;
  } val;
} __attribute__((__packed__));
typedef struct _MP_USR_VAR_INFO MP_USR_VAR_INFO;

struct _LIST_JOBS_RSP_DATA
{
  uint16_t err_no;
  uint16_t jobCount;
  int32_t fileSize;
  char fileName[128];
} __attribute__((__packed__));
typedef struct _LIST_JOBS_RSP_DATA LIST_JOBS_RSP_DATA;

struct _READ_FILE_CHUNK_SEND_DATA
{
  int32_t offset;
  uint32_t length;
  char fileName[128];
} __attribute__((__packed__));
typedef struct _READ_FILE_CHUNK_SEND_DATA READ_FILE_CHUNK_SEND_DATA;

struct _READ_FILE_CHUNK_RSP_DATA
{
  uint16_t err_no;
  char reserved[2];
  int32_t bytesRead;
  uint8_t buffer[900];
} __attribute__((__packed__));

typedef struct _READ_FILE_CHUNK_RSP_DATA READ_FILE_CHUNK_RSP_DATA;

struct _READ_SKILL_RSP_DATA
{
  uint32_t skillPending[2]; // boolean value for each robotNo (1 = TRUE, 0 = FALSE)
  char cmd[2][256];   // skill command text
} __attribute__((__packed__));
typedef struct _READ_SKILL_RSP_DATA READ_SKILL_RSP_DATA;

struct _END_SKILL_SEND_DATA
{
    uint16_t robotNo;
    char reserved[2];
} __attribute__((__packed__));
typedef struct _END_SKILL_SEND_DATA END_SKILL_SEND_DATA;

class MotomanMotionCtrl
{
  static boost::mutex mutex_;
  static boost::mutex skill_que_mutex_;

public:
  /**
   * \brief Default constructor
   */
  MotomanMotionCtrl() {}

  bool init(SmplMsgConnection *connection, int robot_id);

public:
  bool controllerReady();
  bool setTrajMode(bool enable);
  bool setStreamMode(bool enable);
  bool MotomanMotionCtrlsetMaxAcc(int groupNo, float *max_acc);
  bool getMaxAcc(int groupNo, float *max_acc);
  bool setMaxAcc(int groupNo, float *max_acc);
  bool stopTrajectory();

  static std::string getErrorString(const MotionReply &reply);
  static std::string getErrorString(const ReadSingleIOReply &reply);
  static std::string getErrorString(const WriteSingleIOReply &reply);

  bool updateSkillQue();

  bool readFromIO(int address, int *value);
  bool writeToIO(int address, int value);

  bool listJobs(std::vector<std::string> &result);

  bool deleteJob(const std::string jobName, int &errorNumber);
  bool startJob(int taskNumber, std::string jobName, int &errorNumber);
  bool setHold(int hold, int &errorNumber);
  bool waitForJobEnd(int taskNumber, int time, int &errorNumber);
  bool getMasterJob(int taskNumber, std::string &jobName);
  bool setMasterJob(int taskNumber, const std::string &jobName, int &errorNumber);

  bool getCurJob(int taskNumber, int &jobLine, int &step, std::string &jobName);
  bool setCurJob(int jobLine, const std::string &jobName, int &errorNumber);

  bool putUserVars(const motoman_msgs::PutUserVars::Request &req, motoman_msgs::PutUserVars::Response &res);
  bool getUserVars(const motoman_msgs::GetUserVars::Request &req, motoman_msgs::GetUserVars::Response &res);

  bool resetAlarm(int &errorNumber);
  bool cancelError(int &errorNumber);

  bool endSkill(int robotNo);
  bool readSkill(std::vector<int> &skillPending, std::vector<std::string> &cmds);

protected:
  SmplMsgConnection *connection_;
  int robot_id_;
  int call_id_;

  bool sendAndReceive(MotionControlCmd command, MotionReply &reply);
  bool sendAndReceiveRpc(SimpleRpc *data, SimpleRpcReply *reply);
  bool readFileChunk(int offset, int length, const std::string &fileName, char *resultBuffer, int &errorNumber);
  bool requestListJobs(int &jobCount, int &fileSize, std::string &fileName, int &errorNumber);
  bool removeFile(const std::string fileName, int &errorNumber);
};

} // namespace motion_ctrl
} // namespace motoman

#endif // MOTOMAN_DRIVER_MOTION_CTRL_H
