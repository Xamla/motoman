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

#include <string>
#include "motoman_driver/simple_message/motoman_simple_rpc.h"
#include "motoman_driver/simple_message/motoman_simple_rpc_reply.h"
#include "motoman_driver/simple_message/buffer_utils.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

using industrial::shared_types::shared_int;

namespace motoman
{
namespace simple_message
{
namespace rpc_ctrl_reply
{

SimpleRpcReply::SimpleRpcReply(void)
{
  this->init();
}
SimpleRpcReply::~SimpleRpcReply(void)
{
}

void SimpleRpcReply::init()
{
}

void SimpleRpcReply::copyFrom(const SimpleRpcReply &src)
{
  this->callId_ = src.callId_;
  this->status_ = src.status_;
  this->resultSize_ = src.resultSize_;
  this->setResultData(src.getResultData());
}

bool SimpleRpcReply::operator==(const SimpleRpcReply &rhs)
{
  return this->callId_ == rhs.callId_ && this->status_ == rhs.status_ && this->resultData_ == rhs.resultData_;
}

bool SimpleRpcReply::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SimpleRpcReply load");

  if (!buffer->load(this->callId_))
  {
    LOG_ERROR("Failed to load SimpleRpcReply callId_");
    return false;
  }

  if (!buffer->load(this->status_))
  {
    LOG_ERROR("Failed to load SimpleRpcReply status_");
    return false;
  }

  if (!buffer->load(this->resultSize_))
  {
    LOG_ERROR("Failed to load SimpleRpcReply resultSize_");
    return false;
  }

  if (!loadByteArray(*buffer, this->resultData_, MAX_DATA_CNT))
  {
    LOG_ERROR("Failed to load SimpleRpcReply resultData_");
    return false;
  }

  LOG_COMM("SimpleRpcReply data successfully loaded");
  return true;
}

bool SimpleRpcReply::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SimpleRpcReply unload");

  if (!unloadByteArray(*buffer, this->resultData_, MAX_DATA_CNT))
  {
    LOG_ERROR("Failed to unload SimpleRpcReply resultData_");
    return false;
  }

  if (!buffer->unload(this->resultSize_) || this->resultSize_ > MAX_DATA_CNT)
  {
    LOG_ERROR("Failed to unload SimpleRpcReply resultSize_");
    return false;
  }

  if (!buffer->unload(this->status_))
  {
    LOG_ERROR("Failed to unload SimpleRpcReply status_");
    return false;
  }

  if (!buffer->unload(this->callId_))
  {
    LOG_ERROR("Failed to unload SimpleRpcReply callId_");
    return false;
  }

  this->resultData_.resize(this->resultSize_);
  LOG_COMM("SimpleRpcReply data successfully unloaded");
  return true;
}

} // namespace rpc_ctrl_reply
} // namespace simple_message
} // namespace motoman
