-- Example of receiving MAVLink commands

local COMMAND_ACK = {}
COMMAND_ACK.id = 77
COMMAND_ACK.fields = {
             { "command", "<I2" },
             { "result", "<B" },
             { "progress", "<B" },
             { "result_param2", "<i4" },
             { "target_system", "<B" },
             { "target_component", "<B" },
             }

local COMMAND_LONG = {}
COMMAND_LONG.id = 76
COMMAND_LONG.fields = {
             { "param1", "<f" },
             { "param2", "<f" },
             { "param3", "<f" },
             { "param4", "<f" },
             { "param5", "<f" },
             { "param6", "<f" },
             { "param7", "<f" },
             { "command", "<I2" },
             { "target_system", "<B" },
             { "target_component", "<B" },
             { "confirmation", "<B" },
             }

local msg = {}
msg.messages = {}
msg.messages[1] = COMMAND_ACK
msg.messages[2]  = COMMAND_LONG

function get_msg_message(id)
  local x = 0
  for x=1,2 do 
    if msg.messages[x].id == id then
      return msg.messages[x]
    end
  end
  error("Message ID not found")
end

function decode_header(message)
  -- build up a map of the result
  local result = {}

  local read_marker = 3

  -- id the MAVLink version
  result.protocol_version, read_marker = string.unpack("<B", message, read_marker)
  if (result.protocol_version == 0xFE) then -- mavlink 1
    result.protocol_version = 1
  elseif (result.protocol_version == 0XFD) then --mavlink 2
    result.protocol_version = 2
  else
    error("Invalid magic byte")
  end

  _, read_marker = string.unpack("<B", message, read_marker) -- payload is always the second byte

  -- strip the incompat/compat flags
  result.incompat_flags, result.compat_flags, read_marker = string.unpack("<BB", message, read_marker)

  -- fetch seq/sysid/compid
  result.seq, result.sysid, result.compid, read_marker = string.unpack("<BBB", message, read_marker)

  -- fetch the message id
  result.msgid, read_marker = string.unpack("<I3", message, read_marker)

  return result, read_marker
end

function decode(message, msg_map)
  local result, offset = decode_header(message)
  local message_map = get_msg_message(result.msgid)
  if not message_map then
    -- we don't know how to decode this message, bail on it
    return nil
  end

  -- map all the fields out
  for _,v in ipairs(message_map.fields) do
    if v[3] then
      result[v[1]] = {}
      for j=1,v[3] do
        result[v[1]][j], offset = string.unpack(v[2], message, offset)
      end
    else
      result[v[1]], offset = string.unpack(v[2], message, offset)
    end
  end

  -- ignore the idea of a checksum

  return result;
end

function encode(id, message)
  local message_map = get_msg_message(id)
  if not message_map then
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgname)
  end

  local packString = "<"
  local packedTable = {}
  local packedIndex = 1
  for i,v in ipairs(message_map.fields) do
    if v[3] then
      packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
      for j = 1, v[3] do
        packedTable[packedIndex] = message[message_map.fields[i][1]][j]
        if packedTable[packedIndex] == nil then
          packedTable[packedIndex] = 0
        end
        packedIndex = packedIndex + 1
      end
    else
      packString = (packString .. string.sub(v[2], 2))
      packedTable[packedIndex] = message[message_map.fields[i][1]]
      packedIndex = packedIndex + 1
    end
  end
  return message_map.id, string.pack(packString, table.unpack(packedTable))
end

local COMMAND_ACK_ID = 77
local COMMAND_LONG_ID = 76

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

-- initialize MAVLink rx with number of messages, and buffer depth
mavlink:init(1, 10)

-- register message id to receive
mavlink:register_rx_msgid(COMMAND_LONG_ID)

local MAV_CMD_WIND_DATA = 219
--local MAV_CMD_WAYPOINT_USER_1 = 31000

-- Block AP parsing user1 so we can deal with it in the script
-- Prevents "unsupported" ack
--mavlink:block_command(MAV_CMD_WAYPOINT_USER_1)

function handle_command_long(cmd)
    if (cmd.command == MAV_CMD_WIND_DATA) then
        gcs:send_text(0, "Got WIND DATA")

    elseif (cmd.command == MAV_CMD_WAYPOINT_USER_1) then
        -- return ack from command param value
        return math.min(math.max(math.floor(cmd.param1), 0), 5)
    end
    return nil
end

function update()
    local msg, chan = mavlink:receive_chan()
    if (msg ~= nil) then
        local parsed_msg = decode(msg, msg_map)
        if (parsed_msg ~= nil) then

            local result
            if parsed_msg.msgid == COMMAND_LONG_ID then
                result = handle_command_long(parsed_msg)
                gcs:send_text(6, "Wind Value " .. parsed_msg.param2)
            end

            if (result ~= nil) then
                -- Send ack if the command is one were intrested in
                local ack = {}
                ack.command = parsed_msg.command
                ack.result = result
                ack.progress = 0
                ack.result_param2 = 0
                ack.target_system = parsed_msg.sysid
                ack.target_component = parsed_msg.compid

                mavlink:send_chan(chan, encode(77, ack))
            end
        end
    end

    return update, 1000
end

return update()
