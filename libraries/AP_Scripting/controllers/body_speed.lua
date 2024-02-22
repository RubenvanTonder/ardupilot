local function update()

    -- Wait for ahrs to initialize
    if not ahrs:initialised() then
        return update, 1000
    end 

    local body_to_earth = Vector3f()
    body_to_earth = ahrs:get_velocity_NED()
    local yaw = ahrs:get_yaw()
    local roll = ahrs:get_roll()
    local vex = body_to_earth:x()
    local vey = body_to_earth:y()

    local vbx = vex * math.cos(yaw) + vey *math.sin(yaw)
    local vby = -vex * math.sin(yaw) * math.cos(roll) + vey *math.cos(yaw) * math.cos(roll)

    print("Body-x: " .. vbx .. "Body-y: " .. vby)
    print("\n")
    print("Beta: " .. math.asin(vby/vbx))

    return update, 1000
end

return update()