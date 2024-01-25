--[[
   PI Controller used for testing the controller design of the rudder
--]]





UPDATE_RATE_HZ = 1

-- select the servo powering the rudder
local servo_function = Parameter()
local servo_func = servo_function:init("SERVO1_FUNCTION")
local ground_steer = 26

-- setup a parameter block
local PARAM_TABLE_KEY = 73
local PARAM_TABLE_PREFIX = 'STRCTL_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'could not add param table')

-- gcs messages
local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_EMERGENCY = 0

-- drive modes
local rover_guided_mode_num = 15

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end
gcs:send_text(MAV_SEVERITY_INFO,"Loaded gen_control3.lua")
STRCTL_ENABLE = bind_add_param('ENABLE', 1, 1)
if STRCTL_ENABLE:get() <= 0 then
   return
end

gcs:send_text(MAV_SEVERITY_INFO,"Loaded gen_control2.lua")

-- MIN, MAX and IDLE PWM for throttle output
STRCTL_PWM_MIN = bind_add_param('PWM_MIN', 2, 1000)
STRCTL_PWM_MAX = bind_add_param('PWM_MAX', 3, 2000)
STRCTL_PWM_IDLE = bind_add_param('PWM_IDLE', 4, 1500)

-- P and I gains for controller
STRCTL_PID_P = bind_add_param('PID_P', 5, 0.1)
STRCTL_PID_I = bind_add_param('PID_I', 6, 0.1)

-- maximum I contribution
STRCTL_PID_IMAX = bind_add_param('PID_IMAX', 7, 1.0)

-- RCn_OPTION value for 3 position switch
STRCTL_RC_FUNC  = bind_add_param('RC_FUNC',  8, 300)

-- output servo channel that we will control
STRCTL_THR_CHAN = bind_add_param('THR_CHAN', 9, 1)

-- battery index to monitor, 0 is first battery
STRCTL_BAT_IDX  = bind_add_param('BAT_IDX', 10, 0)

-- target voltage
STRCTL_VOLT_TARG = bind_add_param('HDG_TARG', 11, 0)

-- maximum slew rate in percent/second for throttle change
STRCTL_SLEW_RATE = bind_add_param('SLEW_RATE', 12, 100)



gcs:send_text(MAV_SEVERITY_INFO,"Loaded gen_control3.lua")

-- a PI controller implemented as a Lua object
local function PI_controller(kP,kI,iMax,min,max)
    -- the new instance. You can put public variables inside this self
    -- declaration if you want to
    local self = {}
 
    -- private fields as locals
    local _kP = kP
    local _kI = kI
    local _iMax = iMax
    local _min = min
    local _max = max
    local _last_t = nil
    local _I = 0
    local _P = 0
    local _total = 0
    local _counter = 0
    local _target = 0
    local _current = 0
 
    -- update the controller.
    function self.update(target, current)
       local now = millis():tofloat() * 0.001
       if not _last_t then
          _last_t = now
       end
       local dt = now - _last_t
       _last_t = now
       local err = target - current
       _counter = _counter + 1
 
       local P = _kP:get() * err
       if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
          _I = _I + _kI:get() * err * dt
       end
       if _iMax:get() > 0 then
          _I = constrain(_I, -_iMax:get(), iMax:get())
       end
       local I = _I
       local ret = P + I
 
       _target = target
       _current = current
       _P = P
       _total = ret
       return ret
    end
 
    -- reset integrator to an initial value
    function self.reset(integrator)
       _I = integrator
    end
 
    -- log the controller internals
    function self.log(name)
       -- allow for an external addition to total
       logger.write(name,'Targ,Curr,P,I,Total','fffff',_target,_current,_P,_I,_total)
    end
 
    -- return the instance
    return self
 end
 
 local str_PI = PI_controller(GENCTL_PID_P, GENCTL_PID_I, GENCTL_PID_IMAX, 0, 1)
 local last_pwm = STRCTL_PWM_MIN:get()
 -- Find the desired heading angle
 local desired_heading = math.pi
 local rudder_out = 0
 
 function update()
   gcs:send_text(MAV_SEVERITY_INFO,"STRCtl: start " .. last_pwm)
    if arming:is_armed() and vehicle:get_mode() == rover_guided_mode_num then
          gcs:send_text(MAV_SEVERITY_INFO,"STRCtl: run")
          STRCTL_ENABLE = 1
    end
    gcs:send_text(MAV_SEVERITY_INFO,"Enable: " .. STRCTL_ENABLE:get())
    if STRCTL_ENABLE == 1 then
      gcs:send_text(MAV_SEVERITY_INFO,"STRCtl: finish ")
      -- Find the actual heading angle
      -- Find the difference and feed it into controller
      rudder_out = str_PI.update(desired_heading, ahrs:get_yaw())
      rudder_out = constrain(rudder_out, 0, 1)
      
      rudder_pwm = STRCTL_PWM_IDLE:get() + rudder_out * (STRCTL_PWM_MAX:get() - STRCTL_PWM_IDLE:get())
      gcs:send_text(MAV_SEVERITY_INFO,"STRCtl: finish " .. rudder_pwm)
      thr_PI.log("STRC")
    end
    local max_change = STRCTL_SLEW_RATE:get() * (STRCTL_PWM_MAX:get() - STRCTL_PWM_MIN:get()) * 0.01 / UPDATE_RATE_HZ
   
    rudder_pwm = constrain(rudder_pwm, last_pwm - max_change, last_pwm + max_change)
    last_pwm = rudder_pwm
    if STRCTL_THR_CHAN:get() > 0 then
       SRV_Channels:set_output_pwm_chan(servo_func, math.floor(rudder_pwm))
    end
 end
 
 
 -- wrapper around update(). This calls update() at 10Hz,
 -- and if update faults then an error is displayed, but the script is not
 -- stopped
 function protected_wrapper()
   local success, err = pcall(update)
   if not success then
      gcs:send_text(MAV_SEVERITY_EMERGENCY, "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      --return protected_wrapper, 1000
      return
   end
   return protected_wrapper, 1000/UPDATE_RATE_HZ
 end
 
 gcs:send_text(MAV_SEVERITY_INFO,"Loaded gen_control.lua")
 
 -- start running update loop
 return protected_wrapper()