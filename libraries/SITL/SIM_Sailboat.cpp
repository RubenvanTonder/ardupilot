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
    Sailboat simulator class

    see explanation of lift and drag explained here: https://en.wikipedia.org/wiki/Forces_on_sails

    To-Do: add heel handling by calculating lateral force from wind vs gravity force from heel to arrive at roll rate or acceleration
*/

#include "SIM_Sailboat.h"
#include <AP_Math/AP_Math.h>
#include <string.h>
#include <stdio.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

#define STEERING_SERVO_CH   0   // steering controlled by servo output 1
#define MAINSAIL_SERVO_CH   3   // main sail controlled by servo output 4
#define THROTTLE_SERVO_CH   2   // throttle controlled by servo output 3
#define DIRECT_WING_SERVO_CH 4

    // very roughly sort of a stability factors for waves
#define WAVE_ANGLE_GAIN 1
#define WAVE_HEAVE_GAIN 1

Sailboat::Sailboat(const char *frame_str) :
    Aircraft(frame_str),
    steering_angle_max(30),
    turning_circle(1.8),
    sail_area(0.68)
{
    motor_connected = (strcmp(frame_str, "sailboat-motor") == 0);
    lock_step_scheduled = true;
}

// calculate the lift and drag as values from 0 to 1
// given an apparent wind speed in m/s and angle-of-attack in degrees
void Sailboat::calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const
{
    const uint16_t index_width_deg = 10;
    const uint8_t index_max = ARRAY_SIZE(lift_curve) - 1;

    // Convert to expected range
    angle_of_attack_deg = wrap_180(angle_of_attack_deg);

    // assume a symmetrical airfoil
    const float aoa = fabs(angle_of_attack_deg);

    // check extremes
    if (aoa <= 0.0f) {
        lift = lift_curve[0];
        drag = drag_curve[0];
    } else if (aoa >= index_max * index_width_deg) {
        lift = lift_curve[index_max];
        drag = drag_curve[index_max];
    } else {
        uint8_t index = constrain_int16(aoa / index_width_deg, 0, index_max);
        float remainder = aoa - (index * index_width_deg);
        lift = linear_interpolate(lift_curve[index], lift_curve[index+1], remainder, 0.0f, index_width_deg);
        drag = linear_interpolate(drag_curve[index], drag_curve[index+1], remainder, 0.0f, index_width_deg);
    }

    // apply scaling by wind speed
    lift *= wind_speed * sail_area;
    drag *= wind_speed * sail_area;

    if (is_negative(angle_of_attack_deg)) {
        // invert lift for negative aoa
        lift *= -1;
    }
}

void Sailboat::calc_lift_and_drag_rudder(float boat_speed, float angle_of_attack_deg, float& lift, float& drag) const
{
     const uint16_t index_width_deg = 5;
    const uint8_t index_max = ARRAY_SIZE(lift_curve_rudder) -1;

    // Convert to expected range
    angle_of_attack_deg = wrap_180(angle_of_attack_deg);

    // assume a symmetrical airfoil
    float sign = 1;
    if (angle_of_attack_deg < 0 )
    {
        sign = -1;
    }
    const float aoa = fabs(angle_of_attack_deg);

     // check extremes
    if (aoa <= 0.0f) {
        lift = lift_curve_rudder[0];
        drag = drag_curve_rudder[0];
    } else if (aoa >= index_max * index_width_deg) {
        lift = lift_curve_rudder[index_max];
        drag = drag_curve_rudder[index_max];
    } else {
        uint8_t index = constrain_int16(aoa / index_width_deg, 0, index_max);
        float remainder = aoa - (index * index_width_deg);
        lift = sign * linear_interpolate(lift_curve_rudder[index], lift_curve_rudder[index+1], remainder, 0.0f, index_width_deg);
        drag = linear_interpolate(drag_curve_rudder[index], drag_curve_rudder[index+1], remainder, 0.0f, index_width_deg);
    }
}

// return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
float Sailboat::get_turn_circle(float steering) const
{
    if (is_zero(steering)) {
        return 0;
    }
    return turning_circle * sinf(radians(steering_angle_max)) / sinf(radians(steering * steering_angle_max));
}

// return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
float Sailboat::get_yaw_rate(float steering, float speed) const
{
    if (is_zero(steering) || is_zero(speed)) {
        return 0;
    }
    float d = get_turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}

// return yaw rate in rad/sec given a steering input (in the range -1 to +1) and speed in m/s
// without environmental forces
float Sailboat::get_yaw_rate_df(float rf, float u, float v, float dt, float vdot, float& yaw_acc) const
{
    yaw_acc = -(yaw_rate * -0.05 + v) * u  + rf - 0.47 * yaw_rate -vdot * 0.56938 ;
    return (yaw_acc / 0.15 * dt + yaw_rate);
}

// return the sway rate in rad/sec
float Sailboat::get_sway_velocity(float rf, float& sway_acc, float u, float r, float dt) const
{
    sway_acc = rf - 0.56938 * u - sway_acc * 2.5 - 0.08 * u *r-sway_rate*1.5;
    sway_acc /= 4.8532;
    constrain_float(sway_acc,-0.1,-0.1);
    return constrain_float(sway_acc * dt + sway_rate,-0.1,0.1);
}

// return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
float Sailboat::get_lat_accel(float steering, float speed) const
{
    float yawrate = get_yaw_rate(steering, speed);
    float accel = radians(yawrate) * speed;
    return accel;
}

// simulate basic waves / swell
void Sailboat::update_wave(float delta_time)
{
    const float wave_heading = sitl->wave.direction;
    const float wave_speed = sitl->wave.speed;
    const float wave_lenght = sitl->wave.length;
    const float wave_amp = sitl->wave.amp;

    // apply rate propositional to error between boat angle and water angle
    // this gives a 'stability' effect
    float r, p, y;
    dcm.to_euler(&r, &p, &y); 

    // if not armed don't do waves, to allow gyro init
    if (sitl->wave.enable == 0 || !hal.util->get_soft_armed() || is_zero(wave_amp) ) { 
        wave_gyro = Vector3f(-r,-p,0.0f) * WAVE_ANGLE_GAIN;
        wave_heave = -velocity_ef.z * WAVE_HEAVE_GAIN;
        wave_phase = 0.0f;
        return;
    }

    // calculate the sailboat speed in the direction of the wave
    const float boat_speed = velocity_ef.x * sinf(radians(wave_heading)) + velocity_ef.y * cosf(radians(wave_heading));

    // update the wave phase
    const float aprarent_wave_distance = (wave_speed - boat_speed) * delta_time;
    const float apparent_wave_phase_change = (aprarent_wave_distance / wave_lenght) * M_2PI;

    wave_phase += apparent_wave_phase_change;
    wave_phase = wrap_2PI(wave_phase);

    // calculate the angles at this phase on the wave
    // use basic sine wave, dy/dx of sine = cosine
    // atan( cosine ) = wave angle
    const float wave_slope = (wave_amp * 0.5f) * (M_2PI / wave_lenght) * cosf(wave_phase);
    const float wave_angle = atanf(wave_slope);

    // convert wave angle to vehicle frame
    const float heading_dif = wave_heading - y;
    float angle_error_x = (sinf(heading_dif) * wave_angle) - r;
    float angle_error_y = (cosf(heading_dif) * wave_angle) - p;

    // apply gain
    wave_gyro.x = angle_error_x * WAVE_ANGLE_GAIN;
    wave_gyro.y = angle_error_y * WAVE_ANGLE_GAIN;
    wave_gyro.z = 0.0f;

    // calculate wave height (NED)
    if (sitl->wave.enable == 2) {
        wave_heave = (wave_slope - velocity_ef.z) * WAVE_HEAVE_GAIN;
    } else {
        wave_heave = 0.0f;
    }
}

// Send message to SITL terminal for debugging 1s
void Sailboat::send_message(char msg[], float data){
    printf("%s %f",msg, data);
}

/*
  update the sailboat simulation by one time step
 */
void Sailboat::update(const struct sitl_input &input)
{
    // update wind
    update_wind(input);

    // in sailboats the steering controls the rudder, the throttle controls the main sail position
    float steering = ((input.servos[STEERING_SERVO_CH]-1500)/500.0f*32.0f);
    //char str[15] = "Rudder Angle";
    //send_message(str, steering);
    // calculate apparent wind in earth-frame (this is the direction the wind is coming from)
    // Note than the SITL wind direction is defined as the direction the wind is travelling to
    // This is accounted for in these calculations
    Vector3f wind_apparent_ef = velocity_ef - wind_ef;
    const float wind_apparent_dir_ef = degrees(atan2f(wind_apparent_ef.y, wind_apparent_ef.x));
    const float wind_apparent_speed = safe_sqrt(sq(wind_apparent_ef.x)+sq(wind_apparent_ef.y));

    float roll, pitch, yaw;
    dcm.to_euler(&roll, &pitch, &yaw);

    const float wind_apparent_dir_bf = wrap_180(degrees(yaw)-wind_apparent_dir_ef);

    // set RPM and airspeed from wind speed, allows to test RPM and Airspeed wind vane back end in SITL
    rpm[0] = wind_apparent_speed;
    airspeed_pitot = wind_apparent_speed;

    float aoa_deg = 0.0f;
    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    
    
    

    if (sitl->sail_type.get() == 1) {
        // directly actuated wing
        float wing_angle_bf = constrain_float((input.servos[DIRECT_WING_SERVO_CH]-1500)/500.0f * 90.0f, -90.0f, 90.0f);

        aoa_deg = wind_apparent_dir_bf - wing_angle_bf;

    } else if (sitl->sail_type.get() == 3){
        // mainsail with sheet but controller with winch in and out of sail not angle controller

    	// servo rate of change in angle
        float servo_rate = 10.0f;

        // calculate rate from servo output 
        float servo_rate_set = constrain_float(((input.servos[MAINSAIL_SERVO_CH]-1600)/100) * servo_rate, -servo_rate, servo_rate);

        // angle change due to time and servo rate
        float servo_angle_change = servo_rate_set * delta_time;

        // update current sail angle
        current_sail_angle = constrain_float(abs(previous_sail_angle) + servo_angle_change, 30.0f, 60.0f);
        current_sail_angle = abs(previous_sail_angle);
        if (wind_apparent_dir_bf <0){
            current_sail_angle = M_PI/6;
        }else if (wind_apparent_dir_bf > 0){
            current_sail_angle = -M_PI/6;
        }
        


       // char msg[] = " Current Sail Angle: ";
        //float data = current_sail_angle;

        timer = timer + delta_time;
        if (timer >= 1.0f) {
        //send_message(msg, data);
       // printf(" Servo rate %f ", servo_angle_change);
        timer = 0.0f;
        }

        // calculate angle-of-attack from wind to mainsail, cannot have negative angle of attack, sheet would go slack
        aoa_deg = MAX(fabsf(wind_apparent_dir_bf) - current_sail_angle, 0);

        if (is_negative(wind_apparent_dir_bf)) {
            // take into account the current tack
            aoa_deg *= -1;
            
        }
       // printf("%f", aoa_deg);

    } else {
        // mainsail with sheet

        // calculate mainsail angle from servo output 4, 0 to 90 degrees
        int sail_angle_60 = 1800;
        int sail_angle_45 = 1700;
        int sail_angle_30 = 1600;
        int main_sail_pwm = input.servos[MAINSAIL_SERVO_CH];
        if (main_sail_pwm == sail_angle_60) {
            main_sail_angle = 60;
        }else if (main_sail_pwm == sail_angle_45) {
            main_sail_angle = 45;
        }else if(main_sail_pwm == sail_angle_30) {
            main_sail_angle = 30;
        }
        main_sail_angle = constrain_float(main_sail_angle,0,90);

        // calculate angle-of-attack from wind to mainsail, cannot have negative angle of attack, sheet would go slack
        aoa_deg = MAX(fabsf(wind_apparent_dir_bf) - main_sail_angle, 0);

        if (is_negative(wind_apparent_dir_bf)) {
            // take into account the current tack
            aoa_deg *= -1;
        }

    }

    // calculate Lift force (perpendicular to wind direction) and Drag force (parallel to wind direction)
    float lift_wf, drag_wf;
    calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);

    // rotate lift and drag from wind frame into body frame
    const float sin_rot_rad = sinf(radians(wind_apparent_dir_bf));
    const float cos_rot_rad = cosf(radians(wind_apparent_dir_bf));
    const float force_fwd = (lift_wf * sin_rot_rad) - (drag_wf * cos_rot_rad);

    

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // calculate lift and drag force for a rudder 
    float lift_rudder, drag_rudder;
    float boat_speed = sqrt((double)(velocity_body.x)*(double)(velocity_body.x) + (double)(velocity_body.y)*(double)(velocity_body.y));
    
    // Calculate Angle of Attack
    float v_aru = -velocity_body.x ;
    float v_arv =-yaw_rate*xr; // -yaw_rate*xr
    float alpha_ar = atan2(v_arv, v_aru);
    float alpha_a = alpha_ar - radians(steering);

    calc_lift_and_drag_rudder(boat_speed, degrees(alpha_a), lift_rudder, drag_rudder);

    // Rudder lift and drag force 
    drag_rudder = drag_rudder + lift_rudder * rudder_area * (M_PI * zeta_r * d_r * d_r);
    float lift_force_rudder = 0.5f * rho_w * rudder_area * lift_rudder * (v_aru*v_aru + velocity_body.y*velocity_body.y);
    float drag_force_rudder = 0.5f * rho_w * rudder_area * drag_rudder * (v_aru*v_aru + velocity_body.y*velocity_body.y);
    
    // Rudder Forces and Moments
    float Nr =  xr * (lift_force_rudder * cos(alpha_ar) + drag_force_rudder * sin(alpha_ar));
    float Yr = (lift_force_rudder * cos(alpha_ar) + drag_force_rudder * sin(alpha_ar));

    // Acceleration and Velocity Calculation
    yaw_rate = get_yaw_rate_df(Nr, velocity_body.x, velocity_body.y, delta_time, accel_body.y, yaw_accel);

    sway_rate = get_sway_velocity(Yr, sway_accel, velocity_body.x, yaw_rate, delta_time);

    sim_time = sim_time + delta_time;
    if (sim_time > 0.1 )
    {
        //char str[15] = " R: ";
        //send_message(str,  rudder_force);
        //char str1[15] = " Yaw: ";
        //send_message(str1, yaw_rate);
        sim_time = 0;
    }
    
    // yaw rate in degrees/s
    //yaw_rate = get_yaw_rate(steering, speed);

    gyro = Vector3f(0,0,yaw_rate) + wave_gyro;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // hull drag
    float hull_drag = sq(speed) * 0.5f;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // throttle force (for motor sailing)
    // gives throttle force == hull drag at 10m/s
    float throttle_force = 0.0f;
    if (motor_connected) {
        const uint16_t throttle_out = constrain_int16(input.servos[THROTTLE_SERVO_CH], 1000, 2000);
        throttle_force = (throttle_out-1500) * 0.1f;
    }

    // accel in body frame due acceleration from sail and deceleration from hull friction
    accel_body = Vector3f((throttle_force + force_fwd) - hull_drag, 0, 0);
    accel_body /= mass;
    
    // add in accel due to direction change
    accel_body.y += yaw_rate * speed ;

    // now in earth frame
    // remove roll and pitch effects from waves
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    Matrix3f temp_dcm;
    temp_dcm.from_euler(0.0f, 0.0f, y);
    Vector3f accel_earth = temp_dcm * accel_body;

    // we are on the ground, so our vertical accel is zero
   // accel_earth.z = 0 + wave_heave;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));
    
    // tide calcs
    Vector3f tide_velocity_ef;
     if (hal.util->get_soft_armed() && !is_zero(sitl->tide.speed) ) {
        tide_velocity_ef.x = -cosf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.y = -sinf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.z = 0.0f;
     }

    // new velocity vector
    velocity_ef_water += accel_earth * delta_time;
    velocity_ef = velocity_ef_water; // +tide velocity

    // new position vector
    position += (velocity_ef * delta_time).todouble();

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

    // update wave calculations
    update_wave(delta_time);

    // Log the velocity in the body frame from SITL
    // @LoggerMessage: SIM2
    // @Description: Additional simulator state
    // @Field: TimeUS: Time since system startup
    // @Field: Bx: x velocity in bf
    // @Field: By: y velocity in bf
    // @Field: Rf: Rudder Force
    // @Field: Ya: Yaw acceleration
    // @Field: Yv: Yaw Velocity

    AP::logger().WriteStreaming("SIM3", "TimeUS,Bx,By,Rf,Na,Nv,Ra",
                                    "Qffffff",
                                    AP_HAL::micros64(),
                                    velocity_body.x, velocity_body.y, Nr, yaw_accel/M_PI*180, yaw_rate/M_PI*180,steering);
    AP::logger().WriteStreaming("SIM4", "TimeUS,L,D,Ya,Yv,Sa",
                                    "Qfffff",
                                    AP_HAL::micros64(),
                                    lift_wf, drag_wf, sway_accel, sway_rate,main_sail_angle);
}

} // namespace SITL
