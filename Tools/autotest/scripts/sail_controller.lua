-- State 0 where the servo is stopped
function stop()
    gcs:send_text(6,"Stop")
    SRV_Channels:set_output_pwm(95,1600)
end

-- State 2 where the servo is winching in the sail
function winch_in()
    gcs:send_text(6,"winch_in")
end

-- State 1 where the servo is winching out the sail
function winch_out()
    gcs:send_text(6,"winch_out")
end

local choice = 0
if choice == 0 then
    stop()
elseif choice == 2 then
    winch_in()
elseif choice == 1 then
    winch_out()
end 
