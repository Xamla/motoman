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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SIMPLE_RPC_REPLY_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SIMPLE_RPC_REPLY_H

#include <string>
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "buffer_utils.h"
#include "simple_message/log_wrapper.h"
namespace motoman
{
namespace simple_message
{
namespace rpc_ctrl_reply
{


class SimpleRpcReply : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  SimpleRpcReply(void);
  /**
   * \brief Destructor
   *
   */
  ~SimpleRpcReply(void);

  /**
   * \brief Initializes a empty motion control reply
   *
   */
  void init();

  /**
   * \brief Sets the result code
   *
   * \param result code
   */
  void setCallId(industrial::shared_types::shared_int callId)
  {
    this->callId_ = callId;
  }

  /**
   * \brief Returns the result code
   *
   * \return callId number
   */
  industrial::shared_types::shared_int getCallId() const
  {
    return this->callId_;
  }

  /**
   * \brief Returns the result code
   *
   * \return status
   */
  void setStatus(industrial::shared_types::shared_int val)
  {
    this->status_ = val;
  }

  /**
   * \brief Returns the result code
   *
   * \return status
   */
  industrial::shared_types::shared_int getStatus() const
  {
    return this->status_;
  }

  /**
   * \brief Sets call id
   *
   * \param address Controller address of the targeted IO element.
   */
  void setResultData(const std::vector<char>& val)
  {
    this->resultData_ = val;
  }

  const std::vector<char>& getResultData() const
  {
    return this->resultData_;
  }
  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(const SimpleRpcReply &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(const SimpleRpcReply &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 3 * sizeof(industrial::shared_types::shared_int) + MAX_DATA_CNT * sizeof(unsigned char);
  }

private:
  /**
   * \brief callId
   */
  industrial::shared_types::shared_int callId_;
  /**
   * \brief status
   */
  industrial::shared_types::shared_int status_;

  /**
   * \brief resultSize
   */
  industrial::shared_types::shared_int resultSize_;

  static const size_t MAX_DATA_CNT = 960;

   /**
   * \brief resultSize
   */
  std::vector<char> resultData_;
};
}  // namespace io_ctrl_reply
}  // namespace simple_message
}  // namespace motoman

#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_SIMPLE_RPC_REPLY_H
