--[[
   Path Following Controller
--]]

local function delay() 
    return UPDATE, 2000
end
delay()
UPDATE_RATE_HZ = 10

-- setup a parameter block
local PARAM_TABLE_KEY = 75
local PARAM_TABLE_PREFIX = 'PTHCTL_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'Heading', 0.0), 'could not add param1')
-- gcs messages
local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_EMERGENCY = 0

-- Path following Controller variables
local x_r = 0.0
local beta = 0.0
local vby = 0.0
local vbx = 0.0
local x_d = 0.0
local R = 5.0
local delta = 0.0

-- returns null if param cant be found
local wind_dir = Parameter()
if not wind_dir:init('SIM_WIND_DIR') then
  gcs:send_text(6, 'get SIM_WIND_DIR failed')
end

-- Desired track heading angle
local x_p = Parameter()
if not x_p:init('Nv_Heading') then
  gcs:send_text(6, 'get Nv_Heading failed')
end
-- Crosstrack Error
local e = Parameter()
if not e:init('Nv_Crosstrack') then
  gcs:send_text(6, 'get Nv_Crosstrack failed')
end
local tack = Parameter('Nv_Tack')

 -- Find the desired heading angle
 local desired_heading = Parameter()
 if not desired_heading:init('PTHCTL_Heading') then
   gcs:send_text(6, 'get PTHCTL_Heading failed')
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


-- P and I gains for controller
local PTHCTL_PID_P = bind_add_param('PID_P', 1, 0.05)
local PTHCTL_PID_I = bind_add_param('PID_I', 2, 0.005)

-- maximum I contribution
local PTHCTL_PID_IMAX = bind_add_param('PID_IMAX', 3, 0.25)

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
    local _err = 0
 
    -- update the controller.
    function self.update(crosstrack)
       local now = millis():tofloat() * 0.001
       if not _last_t then
          _last_t = now
       end
       local dt = now - _last_t
       _last_t = now

       local err = crosstrack
       _counter = _counter + 1
       
       local crosstrack = constrain(e:get(),-R,R)
       delta = math.sqrt(R^2 - crosstrack^2)
       if delta == 0.0 then
         delta = 0.01
       end
       _kp = 1/delta
       local P = _kP * err
       if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
          _I = _I + _kI * err * dt
       end
       if _iMax > 0 then
          _I = constrain(_I, -_iMax, iMax)
       end
       local I = _I
       local ret = P + I
 
       _err = err
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
       logger.write(name,'Error,P,I,Total,xr','fffff',_err,_P,_I,_total,x_r)
    end
 
    -- return the instance
    return self
 end

local PTH_PI = PI_controller(PTHCTL_PID_P:get(), PTHCTL_PID_I:get(), PTHCTL_PID_IMAX:get(), -0.25, 0.25)
local function update()

   if arming:is_armed() then

    -- Find the actual heading angle 
    -- Check if tacking is required
    --LOS Vector
    local body_to_earth = Vector3f()
    body_to_earth = ahrs:get_velocity_NED()
    local yaw = ahrs:get_yaw()
    local roll = ahrs:get_roll()
    local vex = body_to_earth:x()
    local vey = body_to_earth:y()

    vbx = vex * math.cos(yaw) + vey *math.sin(yaw)
    vby = -vex * math.sin(yaw) * math.cos(roll) + vey *math.cos(yaw) * math.cos(roll)
    beta = math.asin(vby/vbx)

    if tack:get() == 0 then
      x_r = constrain(math.atan(PTH_PI.update(e:get()),1), -0.5, 0.5)
      x_d = x_p:get() - x_r - beta
      x_d = x_d 
    else 
      x_d = x_p:get() - beta
      --print("Desired Tack Heading " .. x_d)
    end
    desired_heading:set(x_d)

    --print("Beta " .. beta)
   -- print("Track Heading "  .. x_p:get())
    --print("Path Heading "  .. x_r)
    --print("Combined Heading "  .. x_d)
    -- Log path following controller data
    PTH_PI.log("PTHC")
    logger.write("BXY",'Beta,BodyX,BodyY','fff',beta,vbx,vby)
   end
end
 
 
 -- wrapper around update(). This calls update() at 10Hz,
 -- and if update faults then an error is displayed, but the script is not
 -- stopped
 function protected_wrapper()
   local success, error = pcall(update)
   if not success then
      gcs:send_text(MAV_SEVERITY_EMERGENCY, "Internal Error: " .. error)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      --return protected_wrapper, 1000
      return
   end
   return protected_wrapper, 1000/UPDATE_RATE_HZ
 end
 
 gcs:send_text(MAV_SEVERITY_INFO,"Loaded Path Following Controller")
 
 -- start running update loop
 return protected_wrapper()