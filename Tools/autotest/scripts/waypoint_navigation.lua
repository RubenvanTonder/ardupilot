-- Script for navigating between waypoints

-- current location of the sailboat
local current_location

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
waypoint.total = 0
waypoint.mission = {Location(), Location()}
local waypoint_reached = 0
local current_waypoint = 0
local loaded = false

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
    waypoint.total = mission:num_commands()-1
    print("Total waypoints " .. waypoint.total)
    -- Iterate through waypoints
    for i=0, (waypoint.total) do
        waypoint.mission[i] = location(mission:get_item(i))
    end
end

local function bearing_and_length_to_waypoint(dest, src)
    local dest_src = src:get_distance_NE(dest)
    track_heading_angle = math.atan(dest_src:y(),dest_src:x())
    l_track = math.sqrt(dest_src:x()^2 + dest_src:y()^2)
end

local function guidance_axis_calc(current, src)
    local current_src = src:get_distance_NE(current)
    print(current_src:x() .. " " .. current_src:y())
    guidance_axis.s = math.cos(track_heading_angle) * current_src:x() + math.sin(track_heading_angle) * current_src:y()
    guidance_axis.e = -math.sin(track_heading_angle) * current_src:x() + math.cos(track_heading_angle) * current_src:y()
end




function UPDATE()

    -- Wait for ahrs to initialize
    if not ahrs:initialised() then
        return UPDATE, 1000
    end 

   


    -- Wait for sailboat to be armed
    if arming:is_armed() then
        
        if not loaded then
            load_waypoints()
            loaded = true
        end

        if (current_waypoint < waypoint.total) then
            -- Get the Current Location of the sailboat 
            -- Home will only stay fixed when sailboat is armed
            current_location = ahrs:get_location()
            --print("Current Location " .. current_location)
            -- Get distance to and angle between source waypoint and destination waypoint
            -- waypoint.mission[current_waypoint] is the source waypoint
            -- waypoint.mission[current_waypoint] is the destination waypoint

            -- Calculate the bearing and length between source and destination waypoint
            bearing_and_length_to_waypoint(waypoint.mission[current_waypoint+1], waypoint.mission[current_waypoint])
            --print(waypoint.mission[0]:lat() .. " " .. waypoint.mission[0]:lng() .. " " ..waypoint.mission[0]:alt())
            --print("\n")
            --print(waypoint.mission[1]:lat() .. " " .. waypoint.mission[1]:lng() .. " " .. waypoint.mission[1]:alt())
            -- Get distance along the track and cross-track error between home and waypoint 1
            guidance_axis_calc(current_location, waypoint.mission[current_waypoint])
            
            print("Track Heading " .. track_heading_angle)
            print("Track Distance " .. l_track)
            print("Track Travelled " .. guidance_axis.s)
            print("Cross-track  " .. guidance_axis.e)
        end
    end
    return UPDATE, 1000
end

return UPDATE()