/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  sailboat simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a sailboat simulator
 */
class Sailboat : public Aircraft {
public:
    Sailboat(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Sailboat(frame_str);
    }

    bool on_ground() const override {return true;};
  

protected:
    bool motor_connected;       // true if this frame has a motor
    float sail_area; // 1.0 for normal area

private:

    // calculate the lift and drag as values from 0 to 1 given an apparent wind speed in m/s and angle-of-attack in degrees
    void calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const;

    // calculate the lift and drag values for the rudder for a given boat speed and angle of attack in degrees
    void calc_lift_and_drag_rudder(float boat_speed, float angle_of_attack_deg, float& lift, float& drag) const;

    // calculate the lift and drag values for the rudder for a given boat speed and angle of attack in degrees
    void calc_lift_and_drag_keel(float boat_speed, float angle_of_attack_deg, float& lift, float& drag) const;

    // return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
    float get_turn_circle(float steering) const;

    // return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
    float get_yaw_rate(float steering, float speed) const;

    // return yaw rate in rad/sec given a steering input (in the range -30 to +30) and speed in m/s
    float get_yaw_rate_df(float rf, float u, float v, float dt, float vdot, float& yaw_acc) const;

     // return yaw rate in rad/sec given a steering input (in the range -30 to +30) and speed in m/s
    float get_roll_rate(float sf, float kf, float& roll_rate, float roll_angle, float &roll_acc, float dt) const;

    // return sway rate deg/sec
    float get_sway_velocity(float Yrud, float Ysail, float Ykeel, float& yawrate, float surge_vel, float sway_vel, float Yhull, float& sway_acc, float dt) const;

    // return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
    float get_lat_accel(float steering, float speed) const;

    // simulate waves and swell
    void update_wave(float delta_time);

    // send message to SITL terminal
    void send_message(char msg[], float data);

    float steering_angle_max;   // vehicle steering mechanism's max angle in degrees
    float turning_circle;       // vehicle minimum turning circle diameter in meters
    
    float starting_sail_angle = 30.0f; //starting sail angle for winch controlled sail
    float main_sail_angle =  30.0f; //starting angle of the main sail
    float current_sail_angle = starting_sail_angle; // current sail angle for winch controlled sail
    float previous_sail_angle = 30.0f; // previous sail angle for winch controlled sail

    float timer; // timer to count to 1s and then send message to terminal in SITL

    // lift and drag curves for sail.  index is angle/10deg
    // angle-of-attack            0 -> 180
    //const float lift_curve[18] = {0.00f, 0.32f, 0.70f, 1.15f, 1.28f, 1.10f, 1.00f, 0.82f, 0.68f, 0.48f, 0.21f, -0.06f, -0.30f, -0.53f, -0.72f, -0.95f, -1.1f, -1.08f};
    //const float drag_curve[18] = {0.10f, 0.14f, 0.19f, 0.35f, 0.54f, 0.70f, 0.9f, 1.04f, 1.16f, 1.24f,  1.28f,  1.36f,  1.33f,  1.28f,  1.25f,  1.10f,  0.88f,  0.64f};

    // lift and drag curves for sail.  index is angle/10deg
    // angle-of-attack            0 -> 180
    const float lift_curve[37] = {0.00f, 0.15f, 0.32f, 0.48f, 0.7f, 0.94f, 1.15f, 1.3f, 1.28f, 1.15f, 1.1f, 1.05f, 1.0f, 0.9f, 0.82f, 0.72f, 0.68f, 0.56f, 0.48f, 0.32f,0.21f,0.08f,-0.06f, -0.18f, -0.3f, -0.4f, -0.53f, -0.64f, -0.72f, -0.84f, -0.95f, -1.04f, -1.1f, -1.14f, -1.08f, -0.76f, 0.0f};
    const float drag_curve[37] = {0.10f, 0.12f, 0.14f, 0.16f, 0.19f, 0.26f, 0.35f, 0.46f, 0.54f, 0.62f,  0.7f,  0.78f,  0.9f,  0.98f,  1.04f,  1.08f,  1.16f,  1.2f, 1.24f, 1.26f, 1.28f, 1.34f, 1.36f, 1.37f, 1.33f, 1.31f, 1.28f, 1.26f, 1.25f, 1.2f, 1.1f, 1.04f, 0.88f, 0.8f, 0.64f, 0.38f, 0.1f};


    // lift and drag curves for rudder
    // angle-of-attack                   0 -> 180
    const float lift_curve_rudder[37] = { 0.00f, 0.42f, 0.73f, 0.95f, 1.10f, 1.17f, 1.18f, 1.16f, 1.12f, 1.07f, 1.00f, 0.92f, 0.83f, 0.72f, 0.62f, 0.48f, 0.33f, 0.16f, 0.00f, -0.16f, -0.33f, -0.48f, -0.62f, -0.72f, -0.83f, -0.92f, -1.00f, -1.07f, -1.12f, -1.16f, -1.18f, -1.17f, -1.10f, -0.95f, -0.73f, -0.42f, 0.00f};
                                        
    // angle-of-attack                   0 -> 180
    const float drag_curve_rudder[37] = {0.00f, 0.03f, 0.06f, 0.10f, 0.17f, 0.30f, 0.48f, 0.74f, 0.98f, 1.18f, 1.34f, 1.50f, 1.65f, 1.76f, 1.89f, 1.97f, 2.01f, 2.05f, 2.08f, 2.05f, 2.01f, 1.97f, 1.89f, 1.76f, 1.65f, 1.50f, 1.34f, 1.18f, 0.98f, 0.74f, 0.48f, 0.30f, 0.17f, 0.10f, 0.06f, 0.03f ,0.00f};

    // lift and drag curves for rudder
    // angle-of-attack                   0 -> 180
    const float lift_curve_keel[37] = { 0.00f, 0.42f, 0.73f, 0.95f, 1.10f, 1.17f, 1.18f, 1.16f, 1.12f, 1.07f, 1.00f, 0.92f, 0.83f, 0.72f, 0.62f, 0.48f, 0.33f, 0.16f, 0.00f, -0.16f, -0.33f, -0.48f, -0.62f, -0.72f, -0.83f, -0.92f, -1.00f, -1.07f, -1.12f, -1.16f, -1.18f, -1.17f, -1.10f, -0.95f, -0.73f, -0.42f, 0.00f};
                                        
    // angle-of-attack                   0 -> 180
    const float drag_curve_keel[37] = {0.00f, 0.03f, 0.06f, 0.10f, 0.17f, 0.30f, 0.48f, 0.74f, 0.98f, 1.18f, 1.34f, 1.50f, 1.65f, 1.76f, 1.89f, 1.97f, 2.01f, 2.05f, 2.08f, 2.05f, 2.01f, 1.97f, 1.89f, 1.76f, 1.65f, 1.50f, 1.34f, 1.18f, 0.98f, 0.74f, 0.48f, 0.30f, 0.17f, 0.10f, 0.06f, 0.03f ,0.00f};

    const float mass = 2.7f;

    // Hull drag
    float hull_x = 0.0f;
    float hull_y = 0.0f;
    //float hull_roll = 0.0f;
    //float hull_yaw = 0.0f;

    // Hull Dimensions
    float xh = 0.02f;

    // Keel Dimensions
    float xk = 0.0f;
    float yk = 0.0f;
    float zk = 0.25f;
    float keel_area = 0.02f;
    float zeta_k = 1.0f;
    float d_k = 0.4f;

    // Sail Dimensions
    float zs = 0.6f;
    
    // Rudder Dimensions
    float xr = 0.55f;
    float zr = 0.2f;
    float rudder_area = 0.008f;
    float zeta_r = 1.0f;        // Rudder Efficiency
    float d_r = 0.2f;           // Rudder Draft

    float rho_w = 1025.0f;       // Water Density

    float sim_time = 0.0f;

    float roll_angle = 0.0f;
    float yaw_rate = 0.0f;       // rad/s
    float yaw_accel = 0.0f;      // rad/s
    float sway_rate = 0.0f;      // rad/s
    float sway_accel = 0.0f;      // rad/s
    float roll_accel = 0.0f;    // rad/s
    float roll_rate = 0.0f;     // rad
    float sway_velocity =0.0f; //m/s

    Vector3f velocity_ef_water;  // m/s
    Vector3f wind_ef_sailboat;                    // m/s, earth frame wind
    Vector3f wave_gyro;          // rad/s
    float wave_heave;            // m/s/s
    float wave_phase;            // rads

    // Matrices for Calculating Position, Velocity and Acceleration in body form
};

} // namespace SITL
