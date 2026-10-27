/**
 * @file sil_harness.c
 * @brief Software-in-the-Loop Simulation Harness Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 Compliance
 */

#include "sil_harness.h"
#include <string.h>

bool sil_harness_init(sil_harness_t           *p_harness,
                      float                    init_roll_deg,
                      float                    init_pitch_deg,
                      float                    hover_throttle,
                      const pid_axis_config_t *p_pid_cfg) {
    if (p_harness == NULL) {
        return false;
    }

    memset(p_harness, 0, sizeof(sil_harness_t));
    p_harness->hover_throttle = hover_throttle;

    /* 1. Reset hardware seam test doubles */
    mock_imu_comm_reset();
    mock_pwm_driver_reset();
    mock_imu_comm_set_whoami(0x68);

    /* 2. Setup PID configuration */
    pid_axis_config_t default_cfg = {
        .kp           = SIL_DEFAULT_KP,
        .ki           = SIL_DEFAULT_KI,
        .kd           = SIL_DEFAULT_KD,
        .windup_limit = SIL_DEFAULT_WINDUP_LIMIT,
        .dt_s         = QFC_SAMPLE_PERIOD_S
    };

    const pid_axis_config_t *p_cfg = (p_pid_cfg != NULL) ? p_pid_cfg : &default_cfg;

    /* 3. Initialize top-level flight controller */
    if (!flight_controller_init(&p_harness->fc, p_cfg, p_cfg)) {
        return false;
    }

    /* 4. Execute 512-sample boot zero-rate gyro calibration */
    mock_imu_comm_set_canned_raw(0, 0, 16384, 0, 0, 0, 0);
    bool b_calib_done = false;
    for (uint32_t i = 0; i < QFC_GYRO_CALIBRATION_SAMPLES; i++) {
        (void)flight_controller_calibrate_step(&p_harness->fc, &b_calib_done);
    }

    if (!b_calib_done) {
        return false;
    }

    /* 5. Initialize 6-DOF physics engine with initial attitude */
    if (!quad_physics_init(&p_harness->physics, init_roll_deg, init_pitch_deg, QFC_SAMPLE_PERIOD_S)) {
        return false;
    }

    /* 6. Arm flight controller and set baseline hover throttle */
    if (!flight_controller_arm(&p_harness->fc)) {
        return false;
    }

    if (!flight_controller_set_throttle(&p_harness->fc, hover_throttle)) {
        return false;
    }

    p_harness->b_armed = true;

    /* 7. Prime mock with initial synthetic IMU reading */
    int16_t ax, ay, az, gx, gy, gz;
    quad_physics_get_raw_imu_counts(&p_harness->physics, &ax, &ay, &az, &gx, &gy, &gz);
    mock_imu_comm_set_canned_raw(ax, ay, az, gx, gy, gz, 0);

    return true;
}

bool sil_harness_step(sil_harness_t *p_harness) {
    if (p_harness == NULL) {
        return false;
    }

    /* 1. Feed true physics sensor measurements into IMU hardware seam */
    int16_t ax, ay, az, gx, gy, gz;
    quad_physics_get_raw_imu_counts(&p_harness->physics, &ax, &ay, &az, &gx, &gy, &gz);
    mock_imu_comm_set_canned_raw(ax, ay, az, gx, gy, gz, 0);

    /* 2. Execute 1 ms deterministic super loop control tick */
    bool ok = flight_controller_process_tick(&p_harness->fc);

    /* 3. Read active motor commands from PWM driver seam */
    motor_commands_t active_motors;
    pwm_driver_get_active_thrust(&active_motors);

    /* 4. Advance physics simulation one time step */
    (void)quad_physics_step(&p_harness->physics, &active_motors);

    /* 5. Query active target orientation */
    setpoint_deg_t target;
    (void)setpoint_manager_get(&p_harness->fc.setpoint_mgr, &target);

    /* 6. Capture telemetry snapshot */
    p_harness->last_telemetry.time_s           = p_harness->physics.time_s;
    p_harness->last_telemetry.true_roll_deg    = p_harness->physics.roll_deg;
    p_harness->last_telemetry.true_pitch_deg   = p_harness->physics.pitch_deg;
    p_harness->last_telemetry.est_roll_deg     = p_harness->fc.current_att.roll_deg;
    p_harness->last_telemetry.est_pitch_deg    = p_harness->fc.current_att.pitch_deg;
    p_harness->last_telemetry.target_roll_deg  = target.roll_deg;
    p_harness->last_telemetry.target_pitch_deg = target.pitch_deg;
    p_harness->last_telemetry.m1_thrust        = active_motors.m1;
    p_harness->last_telemetry.m2_thrust        = active_motors.m2;
    p_harness->last_telemetry.m3_thrust        = active_motors.m3;
    p_harness->last_telemetry.m4_thrust        = active_motors.m4;
    p_harness->last_telemetry.sys_state        = system_state_manager_get_state(&p_harness->fc.state_mgr);
    p_harness->last_telemetry.fault            = system_state_manager_get_fault(&p_harness->fc.state_mgr);

    return ok;
}

void sil_harness_get_telemetry(const sil_harness_t *p_harness, sil_telemetry_t *p_telem) {
    if ((p_harness != NULL) && (p_telem != NULL)) {
        *p_telem = p_harness->last_telemetry;
    }
}

void sil_harness_set_setpoint(sil_harness_t *p_harness, float roll_deg, float pitch_deg) {
    if (p_harness != NULL) {
        (void)setpoint_manager_set(&p_harness->fc.setpoint_mgr, roll_deg, pitch_deg);
    }
}

void sil_harness_apply_disturbance(sil_harness_t *p_harness, float roll_torque_nm, float pitch_torque_nm) {
    if (p_harness != NULL) {
        quad_physics_apply_disturbance(&p_harness->physics, roll_torque_nm, pitch_torque_nm);
    }
}
