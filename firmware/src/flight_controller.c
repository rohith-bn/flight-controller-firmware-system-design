/**
 * @file flight_controller.c
 * @brief Top-Level 1 ms Deterministic Super Loop Coordinator Implementation
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 */

#include "flight_controller.h"
#include <stddef.h>

/**
 * @brief Observer callback invoked automatically whenever any fault is asserted.
 */
static void on_emergency_fault(fault_code_t code) {
    (void)code;
    /* Immediately command hardware timer compare channels to zero duty cycle */
    pwm_driver_zero_all();
}

bool flight_controller_init(flight_controller_t     *p_fc,
                            const pid_axis_config_t *p_roll_cfg,
                            const pid_axis_config_t *p_pitch_cfg) {
    if ((p_fc == NULL) || (p_roll_cfg == NULL) || (p_pitch_cfg == NULL)) {
        return false;
    }

    p_fc->b_initialized = false;
    p_fc->base_throttle = 0.0f;

    /* 1. Initialize safety state machine */
    if (!system_state_manager_init(&p_fc->state_mgr)) {
        return false;
    }

    /* 2. Register emergency fault observer */
    if (!system_state_manager_register_fault_callback(&p_fc->state_mgr, on_emergency_fault)) {
        return false;
    }

    /* 3. Initialize hardware PWM timer driver */
    if (!pwm_driver_init()) {
        return false;
    }

    /* 4. Initialize target orientation setpoint manager */
    if (!setpoint_manager_init(&p_fc->setpoint_mgr)) {
        return false;
    }

    /* 5. Initialize attitude estimator subsystem */
    if (!attitude_estimator_init(&p_fc->attitude_est, COMPLEMENTARY_FILTER_DEFAULT_ALPHA, QFC_SAMPLE_PERIOD_S)) {
        return false;
    }

    /* 6. Initialize dual-axis PID controller */
    if (!pid_controller_init(&p_fc->pid_ctrl, p_roll_cfg, p_pitch_cfg)) {
        return false;
    }

    /* 7. Initialize inertial sensor subsystem */
    if (!imu_driver_init(&p_fc->imu_drv)) {
        return false;
    }

    p_fc->current_att.roll_deg  = 0.0f;
    p_fc->current_att.pitch_deg = 0.0f;
    p_fc->last_commands.m1      = 0.0f;
    p_fc->last_commands.m2      = 0.0f;
    p_fc->last_commands.m3      = 0.0f;
    p_fc->last_commands.m4      = 0.0f;

    p_fc->b_initialized = true;
    return true;
}

bool flight_controller_calibrate_step(flight_controller_t *p_fc, bool *p_b_complete) {
    if ((p_fc == NULL) || (p_b_complete == NULL) || (!p_fc->b_initialized)) {
        return false;
    }

    return imu_driver_calibrate_step(&p_fc->imu_drv, p_b_complete);
}

bool flight_controller_arm(flight_controller_t *p_fc) {
    if ((p_fc == NULL) || (!p_fc->b_initialized)) {
        return false;
    }

    /* Strict safety precondition: refuse arming if calibration incomplete */
    if (!imu_driver_is_calibrated(&p_fc->imu_drv)) {
        return false;
    }

    return system_state_manager_arm(&p_fc->state_mgr);
}

bool flight_controller_disarm(flight_controller_t *p_fc) {
    if ((p_fc == NULL) || (!p_fc->b_initialized)) {
        return false;
    }

    return system_state_manager_disarm(&p_fc->state_mgr);
}

bool flight_controller_set_throttle(flight_controller_t *p_fc, float throttle) {
    if ((p_fc == NULL) || (!p_fc->b_initialized)) {
        return false;
    }

    if ((throttle < QFC_MOTOR_MIN_COMMAND) || (throttle > QFC_MOTOR_MAX_COMMAND)) {
        return false;
    }

    p_fc->base_throttle = throttle;
    return true;
}

bool flight_controller_process_tick(flight_controller_t *p_fc) {
    if ((p_fc == NULL) || (!p_fc->b_initialized)) {
        return false;
    }

    /* Step 1: Read calibrated IMU sensor frame */
    imu_scaled_data_t imu_data;
    if (!imu_driver_read_calibrated(&p_fc->imu_drv, &imu_data)) {
        flight_controller_assert_fault(p_fc, FAULT_IMU_COMM_FAIL);
        return false;
    }

    /* Step 2: Update orientation estimate & check divergence tripwire */
    if (!attitude_estimator_update(&p_fc->attitude_est, &imu_data.accel_mps2, &imu_data.gyro_dps, &p_fc->current_att)) {
        flight_controller_assert_fault(p_fc, FAULT_ATTITUDE_DIVERGENCE);
        return false;
    }

    /* Step 3: Query target orientation setpoints */
    setpoint_deg_t target;
    (void)setpoint_manager_get(&p_fc->setpoint_mgr, &target);

    /* Step 4: Compute dual-axis stabilization demands */
    pid_demands_t demands;
    (void)pid_controller_update(&p_fc->pid_ctrl, &target, &p_fc->current_att, &demands);

    /* Step 5: Mix demands across Quad-X geometry */
    motor_commands_t mixed;
    if (!motor_mixer_compute(p_fc->base_throttle, demands.roll_demand, demands.pitch_demand, &mixed)) {
        flight_controller_assert_fault(p_fc, FAULT_MIXER_OUT_OF_BOUNDS);
        return false;
    }

    /* Step 6: Safety state gating (passes thrust in ARMED; forces 0.0 in DISARMED/FAULT) */
    motor_commands_t gated;
    system_state_manager_gate_motors(&p_fc->state_mgr, &mixed, &gated);

    /* Step 7: Actuation write to hardware PWM registers */
    (void)pwm_driver_set_thrust(&gated);
    p_fc->last_commands = gated;

    return true;
}

void flight_controller_assert_fault(flight_controller_t *p_fc, fault_code_t code) {
    if ((p_fc != NULL) && (p_fc->b_initialized)) {
        (void)system_state_manager_assert_fault(&p_fc->state_mgr, code);
    }
}
