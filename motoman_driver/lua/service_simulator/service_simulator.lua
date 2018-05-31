#!/usr/bin/env th
local ros = require 'ros'


local nh
local services = {}


local function printXamlaBanner()
  print([[
    _  __                __
   | |/ /___ _____ ___  / /___ _
   |   / __ `/ __ `__ \/ / __ `/
  /   / /_/ / / / / / / / /_/ /
 /_/|_\__,_/_/ /_/ /_/_/\__,_/]])
end


local function simpleHandler(request, response, header)
    response.success = true
    return true
end


--- std_srvs/Trigger
local function handleRobotEnable(request, response, header)
    return true
end


--- std_srvs/Trigger
local function handleRobotDisable(request, response, header)
    return true
end


--- industrial_msgs/StopMotion
--[[
    ---
    industrial_msgs/ServiceReturnCode code
]]
--- ServiceReturnCode
--[[
    # Service return codes for simple requests.  All ROS-Industrial service
    # replies are required to have a return code indicating success or failure
    # Specific return codes for different failure should be negative.
    int8 val

    int8 SUCCESS = 1
    int8 FAILURE = -1
]]
local function handleStopMotion(request, response, header)
    response.code.val = 1
    return true
end


--- motoman_msgs/ListJobs
--[[
    ---
    bool success
    string message # informational, e.g. for error messages
    string[] available_jobs
]]
local function handleListJobs(request, response, header)
    response.success = true
    response.message = 'ok'
    response.available_jobs = { 'dummy_job_1', 'dummy_job_2', 'rosvita_rocks' }
    return true
end


--- motoman_msgs/GetMasterJob
--[[
    int16  task_no	# Task number 0: Master task, 1-15: Subtask 1 - 15
    ---
    bool success
    string message # informational, e.g. for error messages
    string job_name # Job name (up to 32 characters for a job name)
]]
local function handleGetMasterJob(request, response, header)

    response.success = true
    response.message = 'ok'
    response.job_name = string.format('dummy_master_job_%d', request.task_no)
    return true
end


local MP_VAR = {
    B=0,
    I=1,
    D=2,
    R=3,
    S=4,
    P=5,
    BP=6,
    EX=7,
}


--- motoman_msgs/GetUserVars
--[[
    UserVarPrimitive[] variables # only var_type and var_no fields are used (value is ignored)
    ---
    bool success
    string message # informational, e.g. for error messages
    int32 err_no
    UserVarPrimitive[] variables
]]
--- motoman_msgs/UserVarPrimitive
--[[
    int32 MP_VAR_B=0
    int32 MP_VAR_I=1
    int32 MP_VAR_D=2
    int32 MP_VAR_R=3
    int32 MP_VAR_S=4
    int32 MP_VAR_P=5
    int32 MP_VAR_BP=6
    int32 MP_VAR_EX=7
    int32 var_type	# variable types field
    int32 var_no	# variable number field
    int64 int_value		# byte, interger (short), double integer (long)
    float64 float_value # real
    string string_value # string
]]

local function createUserVarPrimitive(v)
    local m = ros.Message('motoman_msgs/UserVarPrimitive')
    if v ~= nil then
        m:fillFromTable(v)
    end
    return m
end


local function handleGetUserVars(request, response, header)
    local variables = response.variables
    for i,v in ipairs(request.variables) do
        local v = createUserVarPrimitive(v.values)

        if v.var_no < 0 or v.var_no > 100 then
            response.success = false
            response.message = string.format('var_no %d out of range', v.var_no)
            return true
        end

        local t = v.var_type
        if t == MP_VAR.B then  -- byte
            v.int_value = 12
        elseif t == MP_VAR.I then -- int
            v.int_value = 1234
        elseif t == MP_VAR.D then -- double int
            v.int_value = 123456
        elseif t == MP_VAR.R then -- real
            v.float_value = 123.456
        elseif t == MP_VAR.S then -- string
            v.string_value = 'test'
        else
            response.success = false
            response.message = string.format('Variable type %d not supported', t)
            return true
        end

        variables[#variables + 1] = v
    end

    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/ReadIO
--[[
    uint64 adress
    ---
    bool success
    string message # informational, e.g. for error messages
    uint64 value
]]
local function handleIORead(request, response, header)
    response.success = true
    response.message = 'ok'
    response.value = 1
    return true
end


--- motoman_msgs/WriteIO
--[[
    uint64 adress
    uint64 value
    ---
    bool success
    string message # informational, e.g. for error messages
]]
local function handleIOWrite(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/PutUserVars
--[[
    UserVarPrimitive[] variables
    ---
    bool success
    string message # informational, e.g. for error messages
    int32 err_no
]]
local function handlePutUserVars(request, response, header)
    response.success = true
    response.message = 'ok'
    for i,v in ipairs(request.variables) do

        local t = v.var_type
        if t == MP_VAR.B then  -- byte
            -- v.int_value
        elseif t == MP_VAR.I then -- int
            -- v.int_value
        elseif t == MP_VAR.D then -- double int
            -- v.int_value
        elseif t == MP_VAR.R then -- real
            -- v.float_value
        elseif t == MP_VAR.S then -- string
            v.string_value = 'test'
        else
            response.success = false
            response.err_no = -1
            response.message = string.format('Variable type %d not supported', t)
            return true
        end
    end
    return true
end


--- motoman_msgs/ResetAlarm
--[[
    ---
    bool success
    string message # informational, e.g. for error messages
    int32 err_no # error code
]]
local function handleResetAlarm(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/SetAlarm
--[[
    int16 alm_code
    uint8 sub_code
    string alm_msg
    ---
    bool success
    string status_message # informational, e.g. for error messages
    int32 err_no # error code
]]
local function handleSetAlarm(request, response, header)
    response.success = true
    response.status_message = 'ok'
    return true
end


--- motoman_msgs/SetCurJob
--[[
    int32  job_line # Job line
    string job_name	# Job name (up to 32 characters for a job name)
    ---
    bool success
    string message  # informational, e.g. for error messages
    int32 err_no    # error code
    # 0x0000 Normal end
    # 0x2060 In error/alarm status
    # 0x2010 Robot is in operation
    # 0x2110 Inaccessible data
    # 0x4040 Specified JOB not found
    # 0x5200 Over data range
]]
local function handleSetCurJob(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/SetMasterJob
--[[
    int16  task_no	# Task number 0: Master task, 1-15: Subtask 1 - 15
    string job_name	# Job name (up to 32 characters for a job name)
    ---
    bool success
    string message  # informational, e.g. for error messages
    int32 err_no    # error code
    # 0x0000 Normal end
    # 0x2060 In error/alarm status
    # 0x2010 Robot is in operation
    # 0x3400 Cannot operate MASTER JOB
    # 0x3410 The JOB name is already registered in another task.
    # 0x4040 Specified JOB not found
]]
local function handleSetMasterJob(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/SkillEnd
--[[
    uint32 robot_no
    ---
    bool success
    string status_message
]]
local function handleSkillEnd(request, response, header)
    response.success = true
    response.status_message = 'ok'
    return true
end


--- motoman_msgs/SkillRead
--[[
    ---
    bool success
    string status_message
    string[] cmd
    bool[] skill_pending
]]
local function handleSkillRead(request, response, header)
    response.success = true
    response.status_message = 'ok'
    response.cmd = { '', '' }
    response.skill_pending = { false, false }
    return true
end


--- motoman_msgs/StartJob
--[[
    int16  task_no  # Task number: Always specify the master task, 0.
    string job_name	# Job name (up to 32 characters for a job name)
    ---
    bool success
    string message  # informational, e.g. for error messages
    int32 err_no    # error code
    # 0x0000 Normal end
    # 0x2010 Robot is in operation
    # 0x2030 In HOLD status (PP)
    # 0x2040 In HOLD status (External)
    # 0x2050 In HOLD status (Command)
    # 0x2060 In error/alarm status
    # 0x2070 In SERVO OFF status
    # 0x2080 Wrong operation mode
    # 0x3040 The home position is not registered
    # 0x3050 Out of range (ABSO data)
    # 0x3400 Cannot operate MASTER JOB
    # 0x3410 The JOB name is already registered in another task.
    # 0x4040 Specified JOB not found
]]
local function handleStartJob(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/WaitForJobEnd
--[[
    int16 task_no  # Task number
    int16 time	# Time (unit: sec)
    ---
    bool success
    string message # informational, e.g. for error messages
    int32 err_no # error code
]]
local function handleWaitForJobEnd(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


local function advertiseSerices()
    services['/robot_enable']  = { type = 'std_srvs/Trigger', handler = handleRobotEnable }
    services['/robot_disable'] = { type = 'std_srvs/Trigger', handler = handleRobotDisable }
    services.stop_motion      = { type = 'industrial_msgs/StopMotion', handler = handleStopMotion }
    services.cancel_error     = { type = 'motoman_msgs/CancelError', handler = simpleHandler }
    services.delete_job       = { type = 'motoman_msgs/DeleteJob', handler = simpleHandler }
    services.get_master_job   = { type = 'motoman_msgs/GetMasterJob', handler = handleGetMasterJob }
    services.get_user_vars    = { type = 'motoman_msgs/GetUserVars', handler = handleGetUserVars }
    services.hold_job         = { type = 'motoman_msgs/Hold', handler = simpleHandler }
    services.io_read          = { type = 'motoman_msgs/ReadIO', handler = handleIORead }
    services.io_write         = { type = 'motoman_msgs/WriteIO', handler = handleIOWrite }
    services.list_jobs        = { type = 'motoman_msgs/ListJobs', handler = handleListJobs }
    services.put_user_vars    = { type = 'motoman_msgs/PutUserVars', handler = handlePutUserVars }
    services.reset_alarm      = { type = 'motoman_msgs/ResetAlarm', handler = handleResetAlarm }
    services.set_alarm        = { type = 'motoman_msgs/SetAlarm', handler = handleSetAlarm }
    services.set_cur_job      = { type = 'motoman_msgs/SetCurJob', handler = handleSetCurJob }
    services.set_master_job   = { type = 'motoman_msgs/SetMasterJob', handler = handleSetMasterJob }
    services.skill_end        = { type = 'motoman_msgs/SkillEnd', handler = handleSkillEnd }
    services.skill_read       = { type = 'motoman_msgs/SkillRead', handler = handleSkillRead }
    services.start_job        = { type = 'motoman_msgs/StartJob', handler = handleStartJob }
    services.wait_for_job_end = { type = 'motoman_msgs/WaitForJobEnd', handler = handleWaitForJobEnd }

    for name,svc in pairs(services) do
        ros.INFO('Starting simulated service "%s" (%s)', name, svc.type)
        svc.srv = ros.SrvSpec(svc.type)
        local node_handle = svc.nh or nh
        svc.svc = node_handle:advertiseService(name, svc.srv, svc.handler)
    end
end


local function shutdownServices()
    for name,svc in pairs(services) do
        ros.INFO('Shutting down %s', name)
        svc.svc:shutdown()
    end
end


local function main()
    ros.init('sda_service_simulator')
    nh = ros.NodeHandle()

    printXamlaBanner()
    print('SDA service simulator v0.1\n')

    advertiseSerices()

    local wait = ros.Rate(10)   -- spin with 10 Hz

    while ros.ok() do
        ros.spinOnce()
        wait:sleep()
    end

    shutdownServices()
    ros.shutdown()
end


main()
