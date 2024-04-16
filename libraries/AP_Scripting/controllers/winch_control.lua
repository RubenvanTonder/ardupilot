local winch_out_pwm = 1890
local winch_in_pwm = 1710
local rest_pwm = 1800
local winch = false
local sail_servo = 95
local function winch_out()

    if not winch then 
        SRV_Channels:set_output_pwm(95,math.floor(winch_in_pwm))
        winch = true
        return winch_out, 300
    end
    SRV_Channels:set_output_pwm(95,math.floor(rest_pwm))
    return winch_out, 500
end

return winch_out,2000