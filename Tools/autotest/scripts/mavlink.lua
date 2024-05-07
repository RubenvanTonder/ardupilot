--[
  --Receiving MAVlink messages from a ground station
--]
UPDATE_RATE_HZ = 1  
local direction = Parameter()
if not direction:init('WND_Direction') then
  gcs:send_text(6, 'get WND_Direction failed')
end

local speed = Parameter()
if not speed:init('WND_Speed') then
  gcs:send_text(6, 'get WND_Speed failed')
end

-- Structure of COMMAND_ACK
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

-- Structure of COMMAND_LONG
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

-- Find the message defined above through its id
function get_msg_message(id)
  local x = 0
  for x=1,2 do 
    if msg.messages[x].id == id then
      return msg.messages[x]
    end
  end
  error("Message ID not found")
end

-- Decode the header of the message
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

-- Decode the message received
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

-- Encode a message to be sent from controller
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

-- message id
local COMMAND_ACK_ID = 77
local COMMAND_LONG_ID = 76

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"

-- initialize MAVLink rx with number of messages, and buffer depth
mavlink:init(1, 10)

-- register message id to receive
mavlink:register_rx_msgid(COMMAND_LONG_ID)

-- Custom cmd of message from base station
local MAV_CMD_WIND_DATA = 219

-- Update function to run every second
function update()
  if arming:is_armed() then
    local msg, chan = mavlink:receive_chan()
    if (msg ~= nil) then
        local parsed_msg = decode(msg, msg_map)
        if (parsed_msg ~= nil) then

            local result
            if parsed_msg.msgid == COMMAND_LONG_ID then
                local spd = parsed_msg.param1
                local dir = parsed_msg.param2
                if not speed:set(spd) then
                  gcs:send_text(6, string.format('Wind Speed set failed'))
                end
                if not direction:set(dir) then
                  gcs:send_text(6, string.format('Wind Direction set failed'))
                end
                gcs:send_text(6,"Weather Station data received")
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
  end

    return update, 1000/UPDATE_RATE_HZ
end

return update()
