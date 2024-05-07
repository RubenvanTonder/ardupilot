--[[
   Create global params to be accessed across scripts
--]]

-- Create parameter table for waypoint navigation
local PARAM_TABLE_KEY1 = 74
assert(param:add_table(PARAM_TABLE_KEY1, "Nv_", 5), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY1, 1, 'Heading', 0.0), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY1, 2, 'Crosstrack', 0.0), 'could not add param2')
assert(param:add_param(PARAM_TABLE_KEY1, 3, 'Tack_Heading', 0.0), 'could not add param3')
assert(param:add_param(PARAM_TABLE_KEY1, 4, 'Tack', 0.0), 'could not add param3')
assert(param:add_param(PARAM_TABLE_KEY1, 5, 'Wind', 0), 'could not add param3')

-- Create parameter table for wind data
local PARAM_TABLE_KEY2 = 77
assert(param:add_table(PARAM_TABLE_KEY2, "WND_", 4), 'could not add param table for wind')
assert(param:add_param(PARAM_TABLE_KEY2, 1, 'Onboard', 0.0), 'could not add onboard wind data')
assert(param:add_param(PARAM_TABLE_KEY2, 2, 'Direction', 90.0), 'could not add wind direction data')
assert(param:add_param(PARAM_TABLE_KEY2, 3, 'Speed', 0.0), 'could not add wind speed data')