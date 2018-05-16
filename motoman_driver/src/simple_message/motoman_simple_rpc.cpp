/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Delft Robotics Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Delft Robotics Institute, nor the names
 *    of its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 *
 * \author G.A. vd. Hoorn (TU Delft Robotics Institute)
 */

#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include "motoman_driver/simple_message/motoman_simple_rpc.h"
#include "motoman_driver/simple_message/buffer_utils.h"

//--------------
// RPC Section
//--------------
/*
struct _SmBodyMotoRpc
{
    INT32 callId;
    char functionName[32];
    INT32 argumentsSize;    // in bytes
    unsigned char argumentsData[960];
} __attribute__((__packed__));
typedef struct _SmBodySimpleRpc SmBodyMotoRpc;

struct _SmBodyMotoRpcReply
{
    INT32 callId;
    INT32 status;
    INT32 resultSize;       // in bytes
    unsigned char resultData[960];
} __attribute__((__packed__));
typedef struct _SmBodyMotoRpcReply SmBodyMotoRpcReply;
*/
using industrial::shared_types::shared_int;

namespace motoman
{
namespace simple_message
{
namespace rpc_ctrl
{

SimpleRpc::SimpleRpc(void)
{
  this->init();
}
SimpleRpc::~SimpleRpc(void)
{
}

void SimpleRpc::init()
{
  // TODO: is '0' a good initial value?
  this->init(0, 0);
}

void SimpleRpc::init(shared_int id, shared_int arg)
{
  this->setCallId(id);
  this->setArgumentsSize(arg);
}

void SimpleRpc::copyFrom(SimpleRpc &src)
{
  this->setCallId(src.getCallId());
  this->setArgumentsSize(src.getArgumentsSize());
  this->setArgumentsData(src.getArgumentsData());
  this->setFunctionName(src.getFunctionName());
}

bool SimpleRpc::operator==(SimpleRpc &rhs)
{
  return this->argumentsSize_ == rhs.argumentsSize_ && this->callId_ == rhs.callId_ && this->getArgumentsData() == rhs.getArgumentsData() && this->getFunctionName() == rhs.getFunctionName();
}

bool SimpleRpc::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SimpleRpc command load");

  if (!buffer->load(this->callId_))
  {
    LOG_ERROR("Failed to load SimpleRpc callId");
    return false;
  }

  if (!loadString(*buffer, this->getFunctionName(), MAX_FN_CNT))
  {
    LOG_ERROR("Failed to load SimpleRpc functionName");
    return false;
  }

  if (!buffer->load(this->argumentsSize_))
  {
    LOG_ERROR("Failed to load SimpleRpc value");
    return false;
  }

  if (!loadByteArray(*buffer, this->getArgumentsData(), MAX_DATA_CNT))
  {
    LOG_ERROR("Failed to load ArgumentsData");
    return false;
  }

  LOG_COMM("SimpleRpc data successfully loaded");
  return true;
}

bool SimpleRpc::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SimpleRpc command unload");

  if (!unloadByteArray(*buffer, this->argumentsData_, MAX_DATA_CNT))
  {
    LOG_ERROR("Failed to unload SimpleRpc argumentsData_");
    return false;
  }

  if (!buffer->unload(this->argumentsSize_) || this->argumentsSize_ > MAX_DATA_CNT)
  {
    LOG_ERROR("Failed to unload SimpleRpc argumentsSize_");
    return false;
  }
  this->argumentsData_.resize(this->argumentsSize_);

  if (!unloadString(*buffer, this->functionName_, this->MAX_FN_CNT))
  {
    LOG_ERROR("Failed to unload SimpleRpc functionName_");
    return false;
  }

  if (!buffer->unload(this->callId_))
  {
    LOG_ERROR("Failed to unload SimpleRpc callId_");
    return false;
  }

  LOG_COMM("SimpleRpc data successfully unloaded");
  return true;
}

} // namespace rpc_ctrl
} // namespace simple_message
} // namespace motoman
