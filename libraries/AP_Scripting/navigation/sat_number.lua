local function update()

    -- Wait for ahrs to initialize
    if not ahrs:initialised() then
        return UPDATE, 1000
    end 

    local sat_num = gps:num_sats(0)
    gcs:send_text(6,"Number of sats " .. sat_num)

    return update, 1000
end

return update()