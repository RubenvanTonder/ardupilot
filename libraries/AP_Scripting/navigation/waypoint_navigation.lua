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
local going_home = false
local radius_of_acceptance = 5
-- current location of the sailboat
local current_location
local last_location
local location1
local location2
local location3

local location_counter = 0

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
local current_waypoint = 1
local loaded = false
local waypoint_reached = false
local waypoint_passed = flse

-- Time keeping for logging at different dates

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
    waypoint.total = mission:num_commands()
    gcs:send_text(6,"Mission Commands " .. mission:num_commands())

    -- Iterate through waypoints
    for i=0, (waypoint.total-1) do
        waypoint.mission[i] = location(mission:get_item(i))
    end
end

-- Calculate if waypoint has been reached
local function passed_waypoint()
    local src_dest
    if not going_home then
        src_dest = waypoint.mission[current_waypoint]:get_distance_NE(waypoint.mission[current_waypoint-1])
    else
        src_dest = waypoint.mission[waypoint.total-1]:get_distance_NE(waypoint.mission[0])
    end
    local x = src_dest:x()
    local y = src_dest:y()
    distance = math.sqrt(x*x + y*y)
    if (distance-l_track) < 0 then
        gcs:send_text(6,"Sailboat passed waypoint")
        return true
    else 
        return false
    end
end
local function circle_of_acceptance(current, target_waypoint)
    local distance = current:get_distance_NE(target_waypoint)
    local x = distance:x()
    local y = distance:y()
    local distance = math.sqrt(x^2 + y^2)
    if distance < radius_of_acceptance  then
        gcs:send_text(6,"Sailboat Within Radius of Acceptance")
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
    if going_home then
        logger:write('NAV1','N,E,Ns,Es','ffff',tostring(0),tostring(0),tostring(waypoint.mission[waypoint.total-1]:get_distance_NE(current_location):x()),tostring(waypoint.mission[waypoint.total-1]:get_distance_NE(current_location):y()))
    else
        logger:write('NAV1','N,E,Ns,Es','ffff',tostring(waypoint.mission[0]:get_distance_NE(waypoint.mission[current_waypoint]):x()),tostring(waypoint.mission[0]:get_distance_NE(waypoint.mission[current_waypoint]):y()),tostring(waypoint.mission[0]:get_distance_NE(current_location):x()),tostring(waypoint.mission[0]:get_distance_NE(current_location):y()))
    end
  end

local function delay() 

    return UPDATE, 500
end

local function tack()
    -- Calculate if a tack is required
    -- Wind Angle measured on sailboat will be apparent wind angle so change this when working with the real sailboat
    local wind_angle_radians = math.rad(wind_dir:get())
    apparent_wind_angle = -wind_angle_radians + math.abs(track_heading_angle)
    if math.abs(apparent_wind_angle) < no_go_zone then

        tacking = 1
                
        -- Perform tack right
        if tack_right then 
            track_heading_angle = wind_angle_radians + tack_heading
            -- check if crosstrack error has been reached then switch tack
            if guidance_axis.e > max_tack_distance then
                tack_right = false
                track_heading_angle = wind_angle_radians - tack_heading
            end
            -- Perform tack left
        else
            track_heading_angle = wind_angle_radians - tack_heading

            -- check if crosstrack error has been reached then switch tack
            if guidance_axis.e < -max_tack_distance then
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
    if arming:is_armed()then      
        
        -- Load in the waypoints into array and initialize location array and last location
        if not loaded then
            load_waypoints()
            -- Save home location 
            last_location = ahrs:get_location()
            -- Initialize location array
            location1 = ahrs:get_location()
            location2 = ahrs:get_location()
            location3 = ahrs:get_location()
            loaded = true
        end

        if (current_waypoint < waypoint.total) then
            going_home = false
            -- Get the Current Location of the sailboat 
            -- Home will only stay fixed when sailboat is armed
            
            -- Update Current Position
            if (gps:num_sats(0) > 6) then
                -- Save current location
                current_location = ahrs:get_location()

                -- Update counter so that oldest sample of location is swopped out with newest
                if location_counter == 4 then
                   location_counter = 0 
                elseif location_counter == 1 then
                    location1:lat(current_location:lat())
                    location1:lng(current_location:lng())
                elseif location_counter == 2 then
                    location2:lat(current_location:lat())
                    location2:lng(current_location:lng())
                elseif location_counter == 3 then
                    location3:lat(current_location:lat())
                    location3:lng(current_location:lng())
                end
                location_counter = location_counter +1
                -- Get the average movement of the last three locations
                local lat_avg = location1:lat()/3 + location2:lat()/3 + location3:lat()/3
                local lng_avg = location1:lng()/3 + location2:lng()/3 + location3:lng()/3
                current_location:lat(lat_avg)
                current_location:lng(lng_avg)

                -- Save current location as last active location
                last_location = current_location
                
            else 
                current_location = last_location
            end

            

            -- Calculate the bearing and length between source and destination waypoint
            bearing_and_length_to_waypoint(waypoint.mission[current_waypoint], waypoint.mission[current_waypoint-1])

            -- Get distance along the track and cross-track error between home and waypoint 1
            guidance_axis_calc(current_location, waypoint.mission[current_waypoint-1])

            -- Check if waypoint has been reached
            waypoint_passed = passed_waypoint()
            waypoint_reached = circle_of_acceptance(current_location,waypoint.mission[current_waypoint])
            if waypoint_reached or waypoint_passed then
                current_waypoint = current_waypoint + 1
                waypoint_reached = false
                if current_waypoint == waypoint.total then
                    return UPDATE()
                end
            end

            -- Check if tack is required
            --tack()

        else 
            --Heading Home
            going_home = true
            gcs:send_text(6, "Going Home")
            -- Update Current Position
            -- Update Current Position
            if (gps:num_sats(0) > 6) then
                -- Save current location
                current_location = ahrs:get_location()

                -- Update counter so that oldest sample of location is swopped out with newest
                if location_counter == 4 then
                   location_counter = 0 
                elseif location_counter == 1 then
                    location1:lat(current_location:lat())
                    location1:lng(current_location:lng())
                elseif location_counter == 2 then
                    location2:lat(current_location:lat())
                    location2:lng(current_location:lng())
                elseif location_counter == 3 then
                    location3:lat(current_location:lat())
                    location3:lng(current_location:lng())
                end
                location_counter = location_counter +1
                -- Get the average movement of the last three locations
                local lat_avg = location1:lat()/3 + location2:lat()/3 + location3:lat()/3
                local lng_avg = location1:lng()/3 + location2:lng()/3 + location3:lng()/3
                current_location:lat(lat_avg)
                current_location:lng(lng_avg)

                -- Save current location as last active location
                last_location = current_location
                
            else 
                current_location = last_location
            end

            -- Calculate the bearing and length between source and destination waypoint
            bearing_and_length_to_waypoint(waypoint.mission[0], waypoint.mission[waypoint.total-1])

            -- Get distance along the track and cross-track error between home and waypoint 1
            guidance_axis_calc(current_location, waypoint.mission[waypoint.total-1])
            -- Check if tack is required
            --tack()

            -- Check if waypoint has been reached
            waypoint_passed = passed_waypoint()
            waypoint_reached = circle_of_acceptance(current_location, waypoint.mission[0])

            if waypoint_reached or waypoint_passed then
                current_waypoint = 1;
                waypoint_reached = false
            end
        end
    
        -- Log data
        write_to_dataflash()

        -- Print if global param change failed
        if not table_track_heading:set(track_heading_angle) then
            gcs:send_text(6, "Could not update track heading to table")
        end

        if not table_cross_track:set(guidance_axis.e) then
            gcs:send_text(6,"Could not update crosstrack error to table")
        end
    end
    return UPDATE, 250
end

return UPDATE()