-- This scirpt uses Rover's turn rate controller to make the vehicle follow a zigzag step input

-- Fixed variables
local desired_heading_deg = 10
local reference_heading =0;

local target_yaw_rate = 0
local current_yaw = 0
local servo_function = Parameter()

-- Script Start --

gcs:send_text(0,"Script started")
gcs:send_text(0,"Step Input: " .. tostring(desired_heading_deg))
SRV_Channels:set_output_pwm(94,1500)
local AUX_FUNCTION_NUM = 300
local aux_changed = false
local MAV_SEVERITY_INFO = 6
local STRCTL_ENABLE = 0
local left = false
local start_heading = 0

local function wrap_180(heading)
    if heading > 180 then
        heading = heading - 360
    elseif heading < -180 then
        heading = heading + 360
    end
    return heading
end

local function update()
   
    local aux_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
    if aux_pos ~= LAST_AUX_POS then
       LAST_AUX_POS = aux_pos
       aux_changed = not(aux_changed)
       STRCTL_ENABLE = 1
       gcs:send_text(MAV_SEVERITY_INFO, string.format("Aux set to %u", aux_pos))
    end

    if STRCTL_ENABLE == 1 then
        print("Performing Step")
        start_heading = ahrs:get_yaw()
        start_heading  = wrap_180(start_heading/math.pi*180)
        print("Heading: " .. start_heading)
        SRV_Channels:set_output_pwm(94,1500-150)
    return yaw_rate_controller, 500
   end
    return update, 100
end


function yaw_rate_controller()

    local aux_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
    if aux_pos ~= LAST_AUX_POS then
       LAST_AUX_POS = aux_pos
       aux_changed = not(aux_changed)
       gcs:send_text(MAV_SEVERITY_INFO, string.format("Aux set to %u", aux_pos)) 
    end

    if (wrap_180(ahrs:get_yaw()/math.pi*180) - start_heading) > 10 then
        SRV_Channels:set_output_pwm(94,1500+150)
        gcs:send_text(0,"Left")
    elseif (wrap_180(ahrs:get_yaw()/math.pi*180) - start_heading) < -10 then
        SRV_Channels:set_output_pwm(94,1500-150)
        gcs:send_text(0,"Right")
    end
    if not(aux_changed)  then
        STRCTL_ENABLE = 0
        SRV_Channels:set_output_pwm(94,1500)
        return update,500
     end


    return yaw_rate_controller,50
end

return update()
