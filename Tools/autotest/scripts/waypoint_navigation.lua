-- Script for navigating between waypoints

-- Create param to mimic global variables
local PARAM_TABLE_KEY = 74
assert(param:add_table(PARAM_TABLE_KEY, "Nv_", 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'Heading', 0.0), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'Crosstrack', 0.0), 'could not add param2')
local table_track_heading =  Parameter("Nv_Heading")
local table_cross_track =  Parameter("Nv_Crosstrack")

-- Get the wind direction
-- for fast param acess it is better to get a param object,
-- this saves the code searching for the param by name every time
local wind_dir = Parameter()
if not wind_dir:init('SIM_WIND_DIR') then
  gcs:send_text(6, 'get SIM_WIND_DIR failed')
end
print("Wind Direction " .. wind_dir:get())
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
local waypoint_reached = false

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

-- Calculate if waypoint has been reached
local function circle_of_acceptance(current, target_waypoint)
    local distance = current:get_distance_NE(target_waypoint)
    local distance = math.sqrt(distance:x()^2 + distance:y()^2)
    if distance < 5 then
        return true
    else 
        return false
    end
end


-- Calculate the bearing to and the length between waypoints
local function bearing_and_length_to_waypoint(dest, src)
    local dest_src = src:get_distance_NE(dest)
    track_heading_angle = math.atan(dest_src:y(),dest_src:x())
    l_track = math.sqrt(dest_src:x()^2 + dest_src:y()^2)
end

-- Calculate the guidance axis denoted as s and e(cross-track)
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
        
        -- Load in the waypoints into array
        if not loaded then
            load_waypoints()
            loaded = true
        end

        

        if (current_waypoint < waypoint.total) then
            -- Get the Current Location of the sailboat 
            -- Home will only stay fixed when sailboat is armed
            current_location = ahrs:get_location()

            -- Check if waypoint has been reached
            waypoint_reached = circle_of_acceptance(current_location, waypoint.mission[current_waypoint+1])
            if waypoint_reached then
                print("Waypoint Reached heading to waypoint number " .. (current_waypoint+1))
                current_waypoint = current_waypoint + 1
                waypoint_reached = false
                if current_waypoint == waypoint.total then
                    print("Final waypoint reached")
                    return UPDATE()
                end
            end

            --print("Current Location " .. current_location)
            -- Get distance to and angle between source waypoint and destination waypoint
            -- waypoint.mission[current_waypoint] is the source waypoint
            -- waypoint.mission[current_waypoint] is the destination waypoint

            -- Calculate the bearing and length between source and destination waypoint
            bearing_and_length_to_waypoint(waypoint.mission[current_waypoint+1], waypoint.mission[current_waypoint])

            -- Get distance along the track and cross-track error between home and waypoint 1
            guidance_axis_calc(current_location, waypoint.mission[current_waypoint])

            -- Calculate if a tack is required
            if math.abs(math.rad(wind_dir:get()) - track_heading_angle) < math.pi/4 then
                print("Tack Required")
            end
            
            print("Track Heading " .. track_heading_angle)
            print("Track Distance " .. l_track)
            print("Track Travelled " .. guidance_axis.s)
            print("Cross-track  " .. guidance_axis.e)

            if not table_track_heading:set(track_heading_angle) then
                print("Could not update track heading to table")
            end

            if not table_cross_track:set(guidance_axis.e) then
                print("Could not update crosstrack error to table")
            end
        end
    end
    return UPDATE, 1000
end

return UPDATE()