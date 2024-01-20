-- Reads data in from UART and logs to dataflash

-- find the serial first (0) scripting serial port instance
local port = serial:find_serial(0)
local flag_D = false; -- flag for indication of wind sensor direction
local hundreds = false; -- flag for position in hundreds locaton
local tens = false; -- flag for position in tens locaton
local update_param = false; -- flag used to update wind direction parameter
local wind_direction = 0; -- value used to update wind direction parameter
local value_read = 0; -- value used to stored read byte
local wind_parameter = Parameter()
if not wind_parameter:init('MY_NEW_PARAMETER') then
    gcs:send_text(6, 'get MY_NEW_PARAMETER failed')
  end
if not port or baud == 0 then
    gcs:send_text(6, "No Scripting Serial Port")
    return
end

-- begin the serial port
port:begin(9600)
port:set_flow_control(0)

-- function to decode data on uart
function decode()
    local byte = port:read()

    -- Convert byte from ascii to dec
    if byte == 48 then
        value_read  = 0;
    end
    if byte == 49 then
        value_read  = 1;
    end
    if byte == 50 then
        value_read  = 2;
    end
    if byte == 51 then
        value_read  = 3;
    end
    if byte == 52 then
        value_read  = 4;
    end
    if byte == 53 then
        value_read  = 5;
    end
    if byte == 54 then
        value_read  = 6;
    end
    if byte == 55 then
        value_read  = 7;
    end
    if byte == 56 then
        value_read  = 8;
    end
    if byte == 57 then
        value_read  = 9;
    end
    if update_param then
        flag_D = false;
        hundreds = false;
        tens = false;
        update_param = false;
        if not wind_parameter:set(wind_direction) then
            gcs:send_text(6, string.format('Failed to set wind direction parameter'))
        end
        gcs:send_text(6,wind_direction)
        wind_direction = 0;
    elseif tens then 
        update_param = true;
        wind_direction = wind_direction + value_read*1; -- Add the value in ones position
    elseif hundreds then 
        tens = true;
        wind_direction = wind_direction + value_read*10; -- Add the value in tens position
    elseif flag_D then
        hundreds = true;
        wind_direction = wind_direction + value_read*100; -- Add the value in hundreds position
    elseif byte == 68 then
        flag_D = true;
    end
    
    return decode, 100
end

-- functions to calibrate wind sensor
function calibrate_send()
    port:write("1")
    gcs:send_text(6,"Calibrate Send: 1")
    return calibrate_receive, 1000
end

function calibrate_receive()
    local byte = port:read()
    gcs:send_text(6,"Waiting for wind sensor")
    gcs:send_text(6,byte)
    if byte == 49 then 
        gcs:send_text(6,"Decoding")
        return decode, 500
    end
    return calibrate_receive, 500
end
-- the main update function that is used to check for serial prt
function update()

    if not port then
        gcs:send_text(6, "no Scripting Serial Port")
        return update, 1000
    else
        return decode, 1000
    end

    return update, 1000
end

return update, 1000
