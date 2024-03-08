
-- State 0 where the servo is stopped
function stop()
    gcs:send_text(6,"Stop")
    SRV_Channels:set_output_pwm(95,1600)
end

-- State 2 where the servo is winching in the sail
function winch_in()
    gcs:send_text(6,"winch_in")
    SRV_Channels:set_output_pwm(95,1700)
end

-- State 1 where the servo is winching out the sail
function winch_out()
    gcs:send_text(6,"winch_out")
    SRV_Channels:set_output_pwm(95,1500)
end

local choice = 0
local function update()

     -- Wait for ahrs to initialize
     if not ahrs:initialised() then
        return update, 1000
    end 

    if choice == 0 then
        stop()
    elseif choice == 2 then
        winch_in()
    elseif choice == 1 then
        winch_out()
    end 
    local body_to_earth = Vector3f()
    body_to_earth = ahrs:get_velocity_NED()
    local yaw = ahrs:get_yaw()
    local roll = ahrs:get_roll()
    local vex = body_to_earth:x()
    local vey = body_to_earth:y()

    vbx = vex * math.cos(yaw) + vey *math.sin(yaw)
    vby = -vex * math.sin(yaw) * math.cos(roll) + vey *math.cos(yaw) * math.cos(roll)
    print("X: " .. vbx)
    print("Y: ".. vby)
    return update, 1000
end

return update, 1000