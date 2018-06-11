#!/usr/bin/env th
local ros = require 'ros'


local nh
local services = {}


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


local robot_state = {
    job_list = { 'dummy_job_1', 'dummy_job_2', 'rosvita_rocks' },
    robot_enabled = false,
    io_state = {},
    user_variables = {},
    cur_job = {},
    master_job = {}
}


local io_ranges = {
    { 10, 2567, 'Universal input' },
    { 10010, 12567, 'Universal output' },
    { 20010, 22567, 'External input' },
    { 30010, 32567, 'External output' },
    { 40010, 41607, 'Specific input' },
    { 50010, 52007, 'Specific output' },
    { 60010, 60647, 'Interface panel' },
    { 70010, 79997, 'Auxiliary relay' },
    { 80010, 80647, 'Control input' },
    { 82010, 82207, 'Pseudo input' },
    { 25010, 27567, 'Network input' },
    { 35010, 37567, 'Network output' },
    { 1000000, 1000559, 'Register' },
}


local valid_write_io_ranges = {
    { 10010, 12567, 'Universal output' },
    { 60010, 60647, 'Interface panel' },
    { 25010, 27567, 'Network input' },
    { 1000000, 1000559, 'Register' }
}


local function fillIoTable(first, last)
    local x = 0
    local t = robot_state.io_state
    for i=first,last do
        t[i] = x
        x = x + 1
        if x > 255 then
            x = 0
        end
    end
end


local function initializeRobotState()
    -- fill user variables with dummy values
    local numeric_types = { MP_VAR.B, MP_VAR.I, MP_VAR.D, MP_VAR.R }
    for _,type_code in ipairs(numeric_types) do
        local t = {}
        for i=1,100 do
            t[i] = i
        end
        robot_state.user_variables[type_code] = t
    end
    local t = {}
    for i=1,100 do
        t[i] = string.format('string%03d', i)
    end
    robot_state.user_variables[MP_VAR.S] = t

    -- generate dummy io state
    for _,io_range in ipairs(io_ranges) do
        local lo,hi,name = unpack(io_range)
        fillIoTable(lo, hi)
    end

    for i=1,16 do
        robot_state.cur_job[i] = {
            job_line = 1,
             step = 2,
            job_name = string.format('dummy_job%d', i)
        }
    end

    for i=1,16 do
        robot_state.master_job[i] = string.format('master_job%d', i)
    end
end


local function printSplash()
    print([[
    _  __                __
   | |/ /___ _____ ___  / /___ _
   |   / __ `/ __ `__ \/ / __ `/
  /   / /_/ / / / / / / / /_/ /
 /_/|_\__,_/_/ /_/ /_/_/\__,_/]])
    print('SDA service simulator v0.1\n')
end


local function simpleHandler(request, response, header)
    response.success = true
    return true
end


--- std_srvs/Trigger
local function handleRobotEnable(request, response, header)
    robot_state.robot_enabled = true
    response.success = true;
    response.message = 'ok';
    return true
end


--- std_srvs/Trigger
local function handleRobotDisable(request, response, header)
    robot_state.robot_enabled = false
    response.success = true;
    response.message = 'ok';
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
    response.available_jobs = robot_state.job_list
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
    if request.task_no < 0 or request.task_no > 15 then
        response.success = false
        response.message = 'task_no out of range (0-15)'
        return true
    end

    response.job_name = robot_state.master_job[request.task_no + 1]
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/GetCurJob
--[[
    int16 task_no	# Task number 0: Master task, 1-15: Subtask 1 - 15
    ---
    bool success
    string message  # informational, e.g. for error messages
    uint16 job_line # Job line
    uint16 step     # Step
    string job_name # Job name (up to 32 characters for a job name)
]]
local function handleGetCurJob(request, response, header)
    if request.task_no < 0 or request.task_no > 15 then
        response.success = false
        response.message = 'task_no out of range (0-15)'
        return true
    end

    local job = robot_state.cur_job[request.task_no + 1]
    response.job_line = job.job_line
    response.step = job.step
    response.job_name = job.job_name

    response.success = true
    response.message = 'ok'
    return true
end


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
    local user_variables = robot_state.user_variables
    for i,v in ipairs(request.variables) do
        local v = createUserVarPrimitive(v.values)

        if v.var_no < 0 or v.var_no > 100 then
            response.success = false
            response.message = string.format('var_no %d out of range', v.var_no)
            return true
        end

        local t = v.var_type
        if t == MP_VAR.B or t == MP_VAR.I or t == MP_VAR.D then
            v.int_value = user_variables[t][v.var_no+1]
        elseif t == MP_VAR.R then -- real
            v.float_value = user_variables[t][v.var_no+1]
        elseif t == MP_VAR.S then -- string
            v.string_value = user_variables[t][v.var_no+1]
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
    uint64 address
    ---
    bool success
    string message # informational, e.g. for error messages
    uint64 value
]]
local function handleIORead(request, response, header)
    local address = request.address
    local v = robot_state.io_state[address]
    if v == nil then
        response.success = false
        response.message = string.format('Invalid address %d specified', address)
        response.value = 0
    else
        response.success = true
        response.message = 'ok'
        response.value = v
    end
    return true
end


local function isValidWriteAddress(address)
    for _,valid_range in ipairs(valid_write_io_ranges) do
        local lo, hi = unpack(valid_range)
        if lo >= address and address <= hi then
            return true
        end
    end
    return false
end


--- motoman_msgs/WriteIO
--[[
    uint64 address
    uint64 value
    ---
    bool success
    string message # informational, e.g. for error messages
]]
local function handleIOWrite(request, response, header)
    local address = request.address

    if isValidWriteAddress(address) then
        robot_state.io_state[address] = request.value
        response.success = true
        response.message = 'ok'
    else
        response.success = false
        response.message = string.format('Invalid IO address %d for writing specified', address)
    end

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
    local user_variables = robot_state.user_variables
    for i,v in ipairs(request.variables) do

        if v.var_no < 0 or v.var_no > 100 then
            response.success = false
            response.message = string.format('var_no %d out of range', v.var_no)
            return true
        end

        local t = v.var_type
        if t == MP_VAR.B or t == MP_VAR.I or t == MP_VAR.D then
            user_variables[t][v.var_no+1] = v.int_value
        elseif t == MP_VAR.R then -- real
            user_variables[t][v.var_no+1] = v.float_value
        elseif t == MP_VAR.S then -- string
            user_variables[t][v.var_no+1] = v.string_value
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
    string message # informational, e.g. for error messages
    int32 err_no # error code
]]
local function handleSetAlarm(request, response, header)
    response.success = true
    response.message = 'ok'
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

    local master_job = robot_state.cur_job[1]
    master_job.job_line = request.job_line
    master_job.job_name = request.job_name

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
    if request.task_no < 0 or request.task_no > 15 then
        response.success = false
        response.message = 'task_no out of range (0-15)'
        return true
    end
    if #request.job_name > 32 then
        response.success = false
        response.message = 'job_name too long'
        return true
    end

    robot_state.master_job[request.task_no + 1] = request.job_name
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/SkillEnd
--[[
    uint32 robot_no
    ---
    bool success
    string message
]]
local function handleSkillEnd(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/SkillRead
--[[
    ---
    bool success
    string message
    string[] cmd
    bool[] skill_pending
]]
local function handleSkillRead(request, response, header)
    response.success = true
    response.message = 'ok'
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


--- motoman_msgs/GetPlayStatus
--[[
    ---
    bool success
    string message  # informational, e.g. for error messages
    int32 err_no    # 0 Normal end, -1 Error
    int32 on_hold   # 1 Hold ON, 0 Hold OFF
    int32 on_play   # 1 Start ON, 0 Start OFF
]]
local function handleGetPlayStatus(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/GetMode
--[[
    ---
    bool success
    string message  # informational, e.g. for error messages
    int32 err_no    # 0 Normal end, -1 Error
    int32 s_mode   # 1 TEACH, 2 PLAY
    int32 s_remote # 0 Remote OFF, 1 Remote ON
]]
local function handleGetMode(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/SetServoPower
--[[
    bool power_on # Power on = true, off = false
    ---
    bool success
    string message  # informational, e.g. for error messages
    int32 err_no    # error code
    # 0x0000 Normal end
    # 0x2060 In error/alarm status
    # 0x3450 Failed (Unable to turn servo on)
]]
local function handleSetServoPower(request, response, header)
    response.success = true
    response.message = 'ok'
    return true
end


--- motoman_msgs/GetServoPower
--[[

    ---
    bool success
    string message  # informational, e.g. for error messages
    int32 err_no    # error code
    bool power_on # Power on = true, off = false
]]
local function handleGetServoPower(request, response, header)
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
    services.get_cur_job      = { type = 'motoman_msgs/GetCurJob', handler = handleGetCurJob }
    services.get_master_job   = { type = 'motoman_msgs/GetMasterJob', handler = handleGetMasterJob }
    services.get_user_vars    = { type = 'motoman_msgs/GetUserVars', handler = handleGetUserVars }
    services.hold             = { type = 'motoman_msgs/Hold', handler = simpleHandler }
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
    services.get_play_status  = { type = 'motoman_msgs/GetPlayStatus', handler = handleGetPlayStatus }
    services.get_mode         = { type = 'motoman_msgs/GetMode', handler = handleGetMode }
    services.set_servo_power  = { type = 'motoman_msgs/SetServoPower', handler = handleSetServoPower }
    services.get_servo_power  = { type = 'motoman_msgs/GetServoPower', handler = handleGetServoPower }

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
    printSplash()

    local ros_args = {}

    -- filter ROS special command line args, see http://wiki.ros.org/Remapping%20Arguments
    for k,v in pairs(arg) do
        if v:sub(1, 2) == '__' then
            table.insert(ros_args, v)
            arg[k] = nil
        end
    end

    -- ros initialization
    local ros_init_options = 0
    ros.init('sda_service_simulator', ros_init_options, ros_args)
    nh = ros.NodeHandle()

    initializeRobotState()
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
