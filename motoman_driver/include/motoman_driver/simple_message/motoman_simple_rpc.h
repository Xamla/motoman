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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_SIMPLE_RPC_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_SIMPLE_RPC_H

#ifdef ROS
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "simple_serialize.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

namespace motoman
{
namespace simple_message
{
namespace rpc_ctrl
{

namespace RpcCmds
{
enum RpcCmd
{
    UNDEFINED = 0,
    LIST_JOBS = 300101, // get all jobs on controller
};
} // namespace RpcCmds
typedef RpcCmds::RpcCmd RpcCmd;
/**
 * \brief Class encapsulated write single io data. Motoman specific interface
 * to write a single IO element on the controller.
 *
 * The byte representation of a write single IO command is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   address             (industrial::shared_types::shared_int)    4  bytes
 *   value               (industrial::shared_types::shared_int)    4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class SimpleRpc : public industrial::simple_serialize::SimpleSerialize
{
  public:
    /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
    SimpleRpc(void);
    /**
   * \brief Destructor
   *
   */
    ~SimpleRpc(void);

    /**
   * \brief Initializes a empty write single io command
   *
   */
    void init();

    /**
   * \brief Initializes a complete write single io command
   *
   */
    void init(industrial::shared_types::shared_int id, industrial::shared_types::shared_int arg);

    /**
   * \brief Sets call id
   *
   * \param address Controller address of the targeted IO element.
   */
    void setCallId(industrial::shared_types::shared_int id)
    {
        this->callId_ = id;
    }

    /**
     * \brief Sets call id
     *
     * \param address Controller address of the targeted IO element.
     */
    industrial::shared_types::shared_int getCallId()
    {
        return this->callId_ ;
    }

    /**
   * \brief Sets call id
   *
   * \param address Controller address of the targeted IO element.
   */
    void setFunctionName(const std::string& val)
    {
        this->functionName_ = val;
    }

        /**
     * \brief Returns function name
     *
     * \return data value
     */
    std::string getFunctionName() const
    {
        return this->functionName_;
    }

    /**
     * \brief Sets arguments size
     *
     * \param size value
     */
    void setArgumentsSize(industrial::shared_types::shared_int arg)
    {
        this->argumentsSize_ = arg;
    }

    /**
     * \brief gets argument size
     *
     */
    industrial::shared_types::shared_int getArgumentsSize()
    {
        return this->argumentsSize_ ;
    }

    /**
     * \brief Sets call id
     *
     * \param address Controller address of the targeted IO element.
     */
    void setArgumentsData(const std::vector<char>& val)
    {
        this->argumentsData_ = val;
    }

    /**
     * \brief Clears command data
     */
    void clearFunctionName()
    {
        this->functionName_ = "";
    }

    /**
     * \brief Returns command data
     *
     * \return data value
     */
    std::vector<char> getArgumentsData() const
    {
        return this->argumentsData_;
    }

    /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
    void copyFrom(SimpleRpc &src);

    /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
    bool operator==(SimpleRpc &rhs);

    // Overrides - SimpleSerialize
    bool load(industrial::byte_array::ByteArray *buffer);
    bool unload(industrial::byte_array::ByteArray *buffer);
    unsigned int byteLength()
    {
        return 2 * sizeof(industrial::shared_types::shared_int)
        + MAX_FN_CNT
        + MAX_DATA_CNT;
    }

  private:
    /**
   * \brief Address of RPC element.
   */
    industrial::shared_types::shared_int callId_;

    /**
   * \brief Maximum length (# of char elements) of data buffer
   */
    static const size_t MAX_FN_CNT = 32;
    /**
   * \brief Value of RPC element.
   */
    std::string functionName_; //char

    /**
   * \brief Value of RPC element.
   */
    industrial::shared_types::shared_int argumentsSize_;

    /**
   * \brief Maximum length (# of char elements) of data buffer
   */
    static const size_t MAX_DATA_CNT = 960;
    /**
   * \brief Value of RPC element.
   */
    std::vector<char> argumentsData_;//unsinged char

};
} // namespace rpc_ctrl
} // namespace simple_message
} // namespace motoman

#endif /* MOTOMAN_DRIVER_SIMPLE_MESSAGE_SIMPLE_RPC_H */
