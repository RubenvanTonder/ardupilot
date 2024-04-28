--[[
   PI Controller used for testing the controller design of the rudder
--]]





UPDATE_RATE_HZ = 10

-- select the servo powering the rudder
local servo_function = Parameter()
servo_function:init("SERVO1_FUNCTION")
local servo_Original = 26

local K_rudder = 94

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

-- constrain a value between limits
local function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end


local previous_heading = 0.0
local current_heading = 0.0
local previous_desired_heading = 0.0
local desired_yaw = 0.0


-- MIN, MAX and IDLE PWM for throttle output
STRCTL_PWM_MIN = bind_add_param('PWM_MIN', 2, 1000)
STRCTL_PWM_MAX = bind_add_param('PWM_MAX', 3, 2000)
STRCTL_PWM_IDLE = bind_add_param('PWM_IDLE', 4, 1500)

-- P and I gains for controller
STRCTL_PID_P = bind_add_param('PID_P', 5, 0.67)
STRCTL_PID_I = bind_add_param('PID_I', 6, 0.33)

-- maximum I contribution
STRCTL_PID_IMAX = bind_add_param('PID_IMAX', 7, 0.25)

-- RCn_OPTION value for 3 position switch
STRCTL_RC_FUNC  = bind_add_param('RC_FUNC',  8, 300)

-- output servo channel that we will control
STRCTL_THR_CHAN = bind_add_param('THR_CHAN', 9, 1)

-- battery index to monitor, 0 is first battery
STRCTL_BAT_IDX  = bind_add_param('BAT_IDX', 10, 0)

-- maximum slew rate in percent/second for throttle change
STRCTL_SLEW_RATE = bind_add_param('SLEW_RATE', 12, 100)

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
    local _yawrate = 0
 
    -- update the controller.
    function self.update(target, current)
       local now = millis():tofloat() * 0.001
       if not _last_t then
          _last_t = now
       end
       local dt = now - _last_t
       _last_t = now

       -- Get the desired yaw rate
       local desired_yaw = target - current

       -- Multiply the desired yaw rate with a Feedfoward gain and subtract actual yaw rate to get the error
       _yawrate = ahrs:get_gyro():z()
       local err = 1 * desired_yaw - _yawrate
        _counter = _counter + 1
 
       local P = _kP * err
       if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
          _I = _I + _kI * err * dt
       end
       if _iMax > 0 then
          _I = constrain(_I, -_iMax, iMax)
       end
       local I = _I
       local ret = P + I
 
       _target = desired_yaw
       _current = _yawrate
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
 

 local last_pwm = STRCTL_PWM_MIN:get()
 -- Find the desired heading angle
local desired_heading = Parameter()
if not desired_heading:init('PTHCTL_Heading') then
  gcs:send_text(6, 'get PTHCTL_Heading failed')
end

local rudder_out = 0
local STRCTL_ENABLE = 0
local AUX_FUNCTION_NUM = 300
local rudder_pwm
function set_servo()
   SRV_Channels:set_output_pwm(94,math.floor(rudder_pwm))
   return 
end

function update()
   --gcs:send_text(MAV_SEVERITY_INFO,rc:get_aux_cached(AUX_FUNCTION_NUM))
   -- Initialize PI Controler Values to zero
   if rc:get_aux_cached(AUX_FUNCTION_NUM) == 2 then
     -- gcs:send_text(MAV_SEVERITY_INFO,"STRCtl: run")
      STRCTL_ENABLE = 1
      -- Initialize PI Controller
      servo_function:set(K_rudder);
      STR_PI = PI_controller(STRCTL_PID_P:get(), STRCTL_PID_I:get(), STRCTL_PID_IMAX:get(), -1, 1)
   else
      STRCTL_ENABLE = 0
      servo_function:set(servo_Original)
      SRV_Channels:set_output_pwm(94,1500)
   end

   if STRCTL_ENABLE == 1 and arming:is_armed() then
      -- Find the actual heading angle
      -- Find the difference and feed it into controller
      
      -- issue is get_yaw() gets transform to the range of -pi - pi and I need a method to change it to follow the range correctly
      current_heading = ahrs:get_yaw()

      -- Get desired heading
      -- path following contrller or tacking controller
      desired_yaw = desired_heading:get()

      -- Change current yaw from -pi - pi  to -2*pi - 2*pi
      if (previous_heading - current_heading) < -math.pi then
         current_heading = current_heading - 2 * math.pi
      elseif (previous_heading - current_heading) > math.pi then
         current_heading = current_heading + 2 * math.pi
      end

      previous_heading = current_heading
      rudder_out = STR_PI.update(desired_yaw, current_heading)

      rudder_out = constrain(rudder_out, -1, 1)

      rudder_pwm = STRCTL_PWM_IDLE:get() + rudder_out * (STRCTL_PWM_MAX:get() - STRCTL_PWM_IDLE:get())
      STR_PI.log("STRC")
      
   
      local max_change = STRCTL_SLEW_RATE:get() * (STRCTL_PWM_MAX:get() - STRCTL_PWM_MIN:get()) * 0.01 / UPDATE_RATE_HZ
   
      rudder_pwm = constrain(rudder_pwm, last_pwm - max_change, last_pwm + max_change)
      last_pwm = rudder_pwm
      logger.write("STRD",'DesYaw,Yaw,PrevYaw,Rudder','ffff',desired_yaw,current_heading,previous_heading,rudder_pwm)
      if STRCTL_THR_CHAN:get() > 0 then
         --SRV_Channels:set_output_pwm_chan(servo_number, math.floor(rudder_pwm))
         local succes, err = pcall(set_servo)
      end
   end
end
 
 
 -- wrapper around update(). This calls update() at 10Hz,
 -- and if update faults then an error is displayed, but the script is not
 -- stopped
 function protected_wrapper()
   local success, err = pcall(update)
   if not success then
      gcs:send_text(MAV_SEVERITY_EMERGENCY, "Internal Error: ")
      servo_function:set(servo_Original)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
   end
   return protected_wrapper, 1000/UPDATE_RATE_HZ
 end
 
 gcs:send_text(MAV_SEVERITY_INFO,"Loaded gen_control.lua")
 
 -- start running update loop
 return protected_wrapper()