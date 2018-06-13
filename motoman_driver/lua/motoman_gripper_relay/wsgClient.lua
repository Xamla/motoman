local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local grippers = xamlamoveit.grippers

local COMMAND_IDS = {
    HOMING = 0,
    GRASP = 1,
    RELEASE = 2,
    MOVE = 3,
    ACK = 4
}

local function queryConfig(node_handle, namespace)
    namespace = namespace or node_handle:getNamespace()
    print(namespace)
    local config = node_handle:getParamVariable(namespace)
    return config
end

local Skill = torch.class('Skill')

function Skill:__init(robot_no, cmd, pending)
    self.robot_no = robot_no
    self.cmd = cmd
    self.pending = pending
end

function Skill:__tostring()
    return string.format('robot_no %d; cmd %s', self.robot_no, self.cmd)
end

local function sf(...)
    return string.format(...)
end

local skill_read_srv_spec = ros.SrvSpec('motoman_msgs/SkillRead')
local skill_end_srv_spec = ros.SrvSpec('motoman_msgs/SkillEnd')
local set_alarm_srv_spec = ros.SrvSpec('motoman_msgs/SetAlarm')

local WsgMotomanClient = torch.class('WsgMotomanClient')

function WsgMotomanClient:__init(node_handle)
    self.node_handle = node_handle
    local config = queryConfig(node_handle)

    while config == nil and ros.ok() do
        config = queryConfig(node_handle)
        ros.Duration(1):sleep()
        ros.spinOnce()
    end
    self.skill_read_client = self.node_handle:serviceClient(sf('%s/skill_read', config.robot_ns), skill_read_srv_spec)
    self.skill_end_client = self.node_handle:serviceClient(sf('%s/skill_end', config.robot_ns), skill_end_srv_spec)
    self.skill_que = {}

    self.set_alarm_srv = self.node_handle:serviceClient(sf('%s/set_alarm', config.robot_ns), set_alarm_srv_spec)
    self.gripper_client = {}
    if config.gripper_list then
        for i, v in ipairs(config.gripper_list) do
            ros.INFO('create id %d, namspace: %s', v.id, v.ns)
            self.gripper_client[v.id] = grippers.WeissTwoFingerModel(node_handle, v.ns, v.action_ns)
            ros.INFO('connected to %s/%s', v.ns, v.action_ns)
        end
    end
end

function WsgMotomanClient:setAlarm(msg, code)
    local req = self.set_alarm_srv:createRequest()
    req.alm_code = 8055
    req.sub_code = math.max(code or 1, 1)
    req.alm_msg = msg
    local res = self.set_alarm_srv:call(req)
    if res then
        ros.INFO('Alarm could be set: %s. Message: %s', tostring(res.message), msg)
    end
end

function WsgMotomanClient:process()
    local req = self.skill_read_client:createRequest()
    local res = self.skill_read_client:call(req)

    if res and res.success then
        if res.skill_pending[1] then
            self.skill_que[#self.skill_que + 1] = Skill.new(0, res.cmd[1], res.skill_pending[1])
        end
        if res.skill_pending[2] then
            self.skill_que[#self.skill_que + 1] = Skill.new(1, res.cmd[2], res.skill_pending[2])
        end
    end

    if self.curr_skill == nil then
        if #self.skill_que == 0 then
            return
        else
            self.curr_skill = table.remove(self.skill_que, 1)
        end
    end

    local tokens = string.split(self.curr_skill.cmd, '%;')
    local id = COMMAND_IDS[tokens[1]]
    if id == nil then
        ros.ERROR('command token unknown %s', id)
        self.curr_skill = nil
        return
    end

    local gripper_id = tonumber(tokens[2])
    if self.gripper_client[gripper_id] ~= nil then
        local success, task
        if id == 0 then
            task = self.gripper_client[gripper_id]:home({timeout = 20000})
            if task:hasCompletedSuccessfully() == false then
                self:setAlarm(sf('home gripper [id: %d] failed', gripper_id), 1)
            end
        elseif id == 1 then
            task =
                self.gripper_client[gripper_id]:grasp(
                {
                    width = tonumber(tokens[3]) / 1000,
                    speed = tonumber(tokens[4]) / 1000,
                    force = tonumber(tokens[5]),
                    timeout = 20000
                }
            )
            if task:hasCompletedSuccessfully() == false then
                self:setAlarm(sf('grasp gripper [id: %d] failed', gripper_id))
            end
        elseif id == 2 then
            task =
                self.gripper_client[gripper_id]:release(
                {width = tonumber(tokens[3]) / 1000, speed = tonumber(tokens[4]) / 1000, timeout = 20000}
            )

            if task:hasCompletedSuccessfully() == false then
                self:setAlarm(sf('release gripper [id: %d] failed', gripper_id), 2)
            end
        elseif id == 3 then
            task =
                self.gripper_client[gripper_id]:move(
                {
                    width = tonumber(tokens[3]) / 1000,
                    speed = tonumber(tokens[4]) / 1000,
                    force = tonumber(tokens[5]),
                    stop_on_block = tonumber(tokens[6]) ~= 0,
                    timeout = 20000
                }
            )

            if task:hasCompletedSuccessfully() == false then
                self:setAlarm(sf('move gripper with id: %d failed', gripper_id), 3)
            end
        elseif id == 4 then
            success =
                pcall(
                function()
                    self.gripper_client[gripper_id]:acknowledgeError()
                end
            )
            if success == false then
                self:setAlarm(sf('ack_err gripper [id: %d] failed', gripper_id), 4)
            end
        end
    else
        self:setAlarm(sf('no gripper with id: %d', gripper_id), 5)
    end

    req = self.skill_end_client:createRequest()
    req.robot_no = self.curr_skill.robot_no
    res = self.skill_end_client:call(req)
    if res and res.success then
        self.curr_skill = nil
    end
end

function WsgMotomanClient:shutdown()
    if self.gripper_client ~= nil then
        for i, v in ipairs(self.gripper_client) do
            v:shutdown()
        end
    end
end

local cmd = torch.CmdLine()
local parameter = xamlamoveit.xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
ros.init(parameter['__name'] or 'WsgMotomanClient')

local nh = ros.NodeHandle('~')
local sp = ros.AsyncSpinner() -- background job
sp:start()

local client = WsgMotomanClient.new(nh)

local dt = ros.Rate(10)
local error_msg_func = function(x)
    ros.ERROR(debug.traceback())
    return x
end
while ros.ok() do
    local status, err =
        xpcall(
        function()
            client:process()
        end,
        error_msg_func
    )
    ros.spinOnce()
    dt:sleep()
end

client:shutdown()
sp:stop()
ros.shutdown()
