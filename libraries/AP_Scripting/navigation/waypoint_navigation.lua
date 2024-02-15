-- Script for navigating between waypoint

-- Destination position
local dest_position = {}
dest_position.x = 50.0
dest_position.y = 50.0

-- current location of the sailboat
local current_location

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

-- Generate a Location
local function location(mission)
    local loc  = Location()
    loc:lat(math.floor(mission:x()))
    loc:lng(math.floor(mission:y()))
    loc:alt(math.floor(mission:z()))
    return loc
end

-- Load the waypoints
local function load_waypoints()
     -- Get total number of waypoints
     waypoint.numberof = mission:num_commands()-1
     --print("Numberof " .. waypoint.numberof)
     waypoint.home = mission:get_item(0)
     waypoint.mission = mission:get_item(1)
     --print(waypoint.mission:x())
     waypoint.home = location(waypoint.home)
     waypoint.mission = location(waypoint.mission)
end

local function bearing_and_length_to_waypoint(dest, src)
    local dest_src = src.get_distance_NE(dest)
    track_heading_angle = math.atan(dest_src:y(),dest_src:x())
    l_track = math.sqrt(dest_src:x()^2 + dest_src:y())
end

function UPDATE()

    -- Wait for ahrs to initialize
    if not ahrs:initialised() then
        return UPDATE, 1000
    end

    -- Wait for sailboat to be armed
    if arming:is_armed() then

        -- Load in the waypoints used for navigation including home
        load_waypoints()
       
        -- Get the Current Location of the sailboat 
        -- Home will only stay fixed when sailboat is armed
        current_location = ahrs:get_location()

        -- Get distance to and angle between home and waypoint 1
        bearing_and_length_to_waypoint(waypoint.mission, waypoint.home)

        print("Track Heading " .. track_heading_angle)

        print("Track Distance " .. l_track)

    end
    return update, 1000
end

return update()