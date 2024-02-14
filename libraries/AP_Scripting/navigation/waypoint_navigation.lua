-- Script for navigating between waypoint

-- Destination position
local dest_position = {}
dest_position.x = 50.0
dest_position.y = 50.0

-- current position
local current_position = {}
current_position.x = 0.0
current_position.y = 0.0

-- Source poition
local source_position = {}
source_position.x = 0.0
source_position.y = 0.0

-- track heading angle
local track_heading_angle = 0

-- track length
local l_track = 0.0

-- Guidance Axis system
local guidance_axis = {}
guidance_axis.s = 0.0
guidance_axis.e = 0.0

-- Waypoints
local waypoint = {}
waypoint.numberof = 0
waypoint.home = 0
waypoint.mission = 0

local vector3 
vector3 =Vector3f()

-- Constants Used to convert Lat Lng Alt to NED
local wgs84_e = 0.08181919084
local wgs84_a = 6378137.0

-- Convert Latitude, Longitude and Altitude to North, East and Down
function convert_LLA_to_NED(lat, lon, alt)
    local d = wgs84_e * math.sin(lat)
    local n = wgs84_a / math.sqrt(1 - d*d)

    local ned = Vector3f()
    ned.x((n + alt) * math.cos(lat) * math.cos(lon))
    ned.y(((n + alt)* math.cos(lat) * math.sin(lon)))
    ned.z(((1 - wgs84_e*wgs84_e)*n + alt) * math.sin(lat))
    print(ned:x())
    return ned
end


-- Generate a Location
function location(mission)
    local loc  = Location()
    loc:lat(math.floor(mission:x()))
    loc:lng(math.floor(mission:y()))
    loc:alt(math.floor(mission:z()))
    return loc
end



-- Load the waypoints
function load_waypoints()

    -- Get total number of waypoints
    waypoint.numberof = mission:num_commands()-1
    print("Numberof " .. waypoint.numberof)
    waypoint.home = mission:get_item(0)
    waypoint.mission = mission:get_item(1)
    --print(waypoint.mission:x())
    waypoint.home = location(waypoint.home)
    waypoint.mission = location(waypoint.mission)
    
    vector3 = waypoint.home:get_distance_NED(waypoint.mission)
    print("X: " .. vector3:x() .. "Y: " .. vector3:y())
    print("waypoint x: " .. waypoint.home:lat())
    vector3 = convert_LLA_to_NED(waypoint.home:lat(), waypoint.home:lng(), waypoint.home:alt())
    print("X: " .. vector3:x() .. "Y: " .. vector3:y())
end

function update()

    if not ahrs:initialised() then
        return update, 1000
    end

    -- Get Current Coordinates
    local position = ahrs:get_relative_position_NED_origin()
    ahrs:get_origin()

    if position ~= nil then
        -- Initialize Values
        current_position.x = position:x()
        current_position.y = position:y()


        local y_diff_dest = (dest_position.y - current_position.y)
        local x_diff_dest = (dest_position.x - current_position.x)
        track_heading_angle = math.atan(y_diff_dest,x_diff_dest)
        print("Track Heading Angle: " .. track_heading_angle)

        l_track = math.sqrt((x_diff_dest*x_diff_dest) + (y_diff_dest*y_diff_dest))
        print("Track Distance: " .. l_track)

        local y_diff = (current_position.y - source_position.y)
        local x_diff = (current_position.x - source_position.x)

        guidance_axis.s = math.cos(track_heading_angle) * x_diff + math.sin(track_heading_angle) * y_diff
        print("s: " .. guidance_axis.s)

        guidance_axis.e = -math.sin(track_heading_angle) * x_diff + math.cos(track_heading_angle) * y_diff
        print("e: " .. guidance_axis.e)
        
        load_waypoints()
    end
    return update, 1000
end

return update()