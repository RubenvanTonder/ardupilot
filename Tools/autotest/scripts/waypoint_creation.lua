--[[
    create waypoints accurately to test path following controller, tacking controller
    and heading controller    
--]]


local written = false

-- Constants for Earth's radius
local R = 6371e3 -- in meters

local function cartesian_to_geographic(x, y, z)
    local r = math.sqrt(x*x + y*y + z*z) -- radial distance
    local lon = math.atan(y, x) -- longitude
    local lat = math.asin(z / r) -- latitude
    local alt = r - R -- altitude
    return lat, lon, alt
end

local function save_to_sdcard()

end

local function update()

    -- Wait for ahrs to initialize
        --if not ahrs:initialised() then
       --     return update, 1000
       -- end 
              
        -- Test with some Cartesian coordinates
    
        if not written then 

        -- Waypoints
        -- Currently resulting in a waypoint a litte to the east 
        local waypoint_1 = {}
        waypoint_1["x"] = 5 
        waypoint_1["y"] = 15
        local waypoint_2 = {}
        waypoint_2["x"] = -15 
        waypoint_2["y"] = 15

        local lat, lon, alt = cartesian_to_geographic(waypoint_1.x, waypoint_1.y, Location():origin_alt())
        local m = mission:get_item(0)
        print("Lat: " .. lat .. " Lon: " .. lon .. " Alt: " .. alt )
        m:command(16)
        m:x(math.floor(lat))
        m:y(math.floor(lon))
        m:z(math.floor(alt))
        mission:set_item(1,m)
        local lat, lon, alt = cartesian_to_geographic(waypoint_2.x, waypoint_2.y, Location():origin_alt())
        m:command(16)
        m:x(math.floor(lat))
        m:y(math.floor(lon))
        m:z(math.floor(alt))
        mission:set_item(2,m)

        written =true
        end

    return update, 1000
end

return update()
