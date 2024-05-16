function update()
      --LOS Vector
      local body_to_earth = Vector3f()
      body_to_earth = ahrs:get_velocity_NED()
      local yaw = ahrs:get_yaw()
      local roll = ahrs:get_roll()
      local vex = gps:velocity(0):x()
      local vey = gps:velocity(0):y()

      vbx = vex * math.cos(yaw) + vey *math.sin(yaw)
      vby = -vex * math.sin(yaw) * math.cos(roll) + vey *math.cos(yaw) * math.cos(roll)
      -- Check if vbx ~= 0 
      --if vbx~=0 then
      --   beta = math.asin(constrain(vby/vbx,-0.2,0.2))
         --beta=0
      --else
      --   beta=0
     -- end
      gcs:send_text(6,"X " ..vbx)
      gcs:send_text(6,"Y " ..vby)
      return update, 500
end
return update()