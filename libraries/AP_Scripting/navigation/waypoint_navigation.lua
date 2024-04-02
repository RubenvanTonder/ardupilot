-- Script for navigating between waypoints

-- Global parameters
local table_track_heading =  Parameter("Nv_Heading")
local table_cross_track =  Parameter("Nv_Crosstrack")
local tack_heading = Parameter('Nv_Tack_Heading')
local tack_require = Parameter('Nv_Tack')

-- Get the wind direction
-- for fast param acess it is better to get a param object,
-- this saves the code searching for the param by name every time
local wind_dir = Parameter()
if not wind_dir:init('SIM_WIND_DIR') then
  gcs:send_text(6, 'get SIM_WIND_DIR failed')
end

-- Tacking and Indirect waypoint approach
local apparent_wind_angle
local tack_right = true
local max_tack_distance = 10.0
local tack_heading = math.pi/4
local no_go_zone = math.pi/4
local tacking = 0
local going_home
local radius_of_acceptance = 5
-- current location of the sailboat
local current_location

-- track heading angle
local track_heading_angle = 0

local heading_to_waypoint = 0.0
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
local current_waypoint = 0
local loaded = false
local waypoint_reached = false

-- Desired Heading Angle
local desired_heading = 0.0

local started = false
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
    --print("Total waypoints " .. waypoint.total)

    -- Iterate through waypoints
    for i=0, (waypoint.total) do
        waypoint.mission[i] = location(mission:get_item(i))
    end
end

-- Calculate if waypoint has been reached
local function circle_of_acceptance(current, target_waypoint)
    local distance = current:get_distance_NE(target_waypoint)
  local x = distance:x()
    local y =  distance:y()
    local distance = math.sqrt(x^2 + y^2)
    local in_track_distance = math.cos(track_heading_angle) * x + math.sin(track_heading_angle) * y
   -- print("in track distance" .. in_track_distance)
    if distance < radius_of_acceptance  then
        return true
    elseif (in_track_distance) < 0 then
        return true
    else 
        return false
    end
end

-- Flip angle to range fro,m -pi -> pi
local function flip(angle)
    if angle > math.pi then
        angle = angle -2*math.pi
    elseif angle <-math.pi then
        angle = angle + 2*math.pi
    end
    return angle
end

-- Calculate the bearing to and the length between waypoints
local function bearing_and_length_to_waypoint(dest, src)
    local dest_src = src:get_distance_NE(dest)
    local E_diff = dest_src:y()
    local N_diff = dest_src:x()
    track_heading_angle = math.atan(E_diff,N_diff)
    l_track = math.sqrt(N_diff^2 + E_diff^2)
end

-- Calculate the guidance axis denoted as s and e(cross-track)
local function guidance_axis_calc(current, src)
    local current_src = src:get_distance_NE(current)
    --print(current_src:x() .. " " .. current_src:y())
    guidance_axis.s = math.cos(track_heading_angle) * current_src:x() + math.sin(track_heading_angle) * current_src:y()
    guidance_axis.e = -math.sin(track_heading_angle) * current_src:x() + math.cos(track_heading_angle) * current_src:y()
end

local function write_to_dataflash()
    logger:write('NAV','s,e,tack,waypoint,heading','fffff',tostring(guidance_axis.s),tostring(guidance_axis.e), tostring(tacking), tostring(current_waypoint), tostring(track_heading_angle))
    logger:write('NAV1','N,E','ff',tostring(waypoint.mission[0]:get_distance_NE(waypoint.mission[current_waypoint+1]):x()),tostring(waypoint.mission[0]:get_distance_NE(waypoint.mission[current_waypoint+1]):y()))
  end

local function delay() 

    return UPDATE, 500
end

local function tack()
    -- Calculate if a tack is required
    -- Wind Angle measured on sailboat will be apparent wind angle so change this when working with the real sailboat
    local wind_angle_radians = math.rad(wind_dir:get())
    apparent_wind_angle = -wind_angle_radians + math.abs(track_heading_angle)
    --print("Apparent Wind Angle " .. apparent_wind_angle)
    if math.abs(apparent_wind_angle) < no_go_zone then
        --print("Tack Required")
        tacking = 1
                
        -- Perform tack right
        if tack_right then 
            track_heading_angle = wind_angle_radians + tack_heading
            --print("Desired Heading Angle " .. track_heading_angle)
            --print("Tack Right")
            -- check if crosstrack error has been reached then switch tack
            if guidance_axis.e > max_tack_distance then
                tack_right = false
                track_heading_angle = wind_angle_radians - tack_heading
            end
            -- Perform tack left
        else
            track_heading_angle = wind_angle_radians - tack_heading
            --print("Desired Heading Angle " .. track_heading_angle)
            --print("Tack Left")
            -- check if crosstrack error has been reached then switch tack
            if guidance_axis.e < -max_tack_distance then
                --print("Switch Tack")
                tack_right = true
                track_heading_angle = wind_angle_radians + tack_heading
            end
        end

    else
    --print("Tack not Required")
    tacking = 0
    end
    -- Set global tack parameter 
    tack_require:set(tacking)
end
 
function UPDATE()

    -- Wait for ahrs to initialize
    if not ahrs:initialised() then
        return UPDATE, 1000
    end 

    -- Wait for sailboat to be armed
    if arming:is_armed() then
        --if not started then
         --   delay()
         --   started = true
       -- end
        
        
        -- Load in the waypoints into array
        if not loaded then
            load_waypoints()
            loaded = true
        end

        -- Log data
        write_to_dataflash()


        if (current_waypoint < waypoint.total) then
            going_home = false
            -- Get the Current Location of the sailboat 
            -- Home will only stay fixed when sailboat is armed
            current_location = ahrs:get_location()

            -- Check if waypoint has been reached
            waypoint_reached = circle_of_acceptance(current_location, waypoint.mission[current_waypoint+1])
            if waypoint_reached then
                current_waypoint = current_waypoint + 1
                --print("Waypoint Reached heading to waypoint number " .. (current_waypoint+1))
                waypoint_reached = false
                if current_waypoint == waypoint.total then
                    --print("Final waypoint reached")
                    return UPDATE()
                end
            end

            -- Calculate the bearing and length between source and destination waypoint
            bearing_and_length_to_waypoint(waypoint.mission[current_waypoint+1], waypoint.mission[current_waypoint])

            -- Get distance along the track and cross-track error between home and waypoint 1
            guidance_axis_calc(current_location, waypoint.mission[current_waypoint])

            -- Check if tack is required
            --tack()

        else 
            going_home = true
            -- Update Current Position
            current_location = ahrs:get_location()

            -- Returning home
            --print("Returning to Home, Current Waypoint: " .. current_waypoint)
            -- Calculate the bearing and length between source and destination waypoint
            bearing_and_length_to_waypoint(waypoint.mission[0], waypoint.mission[waypoint.total])
            --print("Track Heading Angle: " .. track_heading_angle)
            -- Get distance along the track and cross-track error between home and waypoint 1
            guidance_axis_calc(current_location, waypoint.mission[waypoint.total])
            -- Check if tack is required
            --tack()

            -- Check if waypoint has been reached
            waypoint_reached = circle_of_acceptance(current_location, waypoint.mission[0])
            if waypoint_reached then
                current_waypoint = current_waypoint + 1
                --print("Waypoint Reached heading to waypoint number " .. (current_waypoint+1))
                current_waypoint = 0;
                waypoint_reached = false
            end
        end
        -- Print if global param change failed
        if not table_track_heading:set(track_heading_angle) then
            --print("Could not update track heading to table")
        end

        if not table_cross_track:set(guidance_axis.e) then
            --print("Could not update crosstrack error to table")
        end
    end
    return UPDATE, 250
end

return UPDATE()