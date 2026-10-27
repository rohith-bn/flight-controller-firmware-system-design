/**
 * @file qfc_types.h
 * @brief Master System Types and Physical Constants for QFC-1 Flight Controller
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Defines all hardware-independent data structures, vector representations,
 * system state enums, fault codes, and physical scaling constants used across
 * all 15 firmware modules.
 */

#ifndef QFC_TYPES_H
#define QFC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* PHYSICAL & SENSOR CONSTANTS                                                */
/* ========================================================================== */

/** Standard Earth Gravitational Acceleration (m/s^2) */
#define QFC_GRAVITY_MSS                  (9.80665f)

/** MPU6050 Accelerometer Sensitivity at +/- 2g range (LSB / g) */
#define QFC_MPU6050_ACCEL_LSB_PER_G      (16384.0f)

/** MPU6050 Gyroscope Sensitivity at +/- 250 deg/s range (LSB / (deg/s)) */
#define QFC_MPU6050_GYRO_LSB_PER_DPS     (131.0f)

/** MPU6050 Temperature Sensitivity (LSB / deg C) */
#define QFC_MPU6050_TEMP_LSB_PER_C       (340.0f)
#define QFC_MPU6050_TEMP_OFFSET_LSB      (36.53f)

/** Fixed Super Loop Sample Period (1 ms = 1000 Hz) */
#define QFC_SAMPLE_PERIOD_S              (0.001f)

/** Mathematical Unit Conversions */
#define QFC_PI                           (3.14159265358979323846f)
#define QFC_RAD_TO_DEG                   (57.2957795130823208768f)
#define QFC_DEG_TO_RAD                   (0.01745329251994329577f)

/** Output Safety Bounds for Motor Normalized Commands */
#define QFC_MOTOR_MIN_COMMAND            (0.0f)
#define QFC_MOTOR_MAX_COMMAND            (1.0f)

/** Safety Limits */
#define QFC_MAX_ALLOWABLE_TILT_DEG       (45.0f)
#define QFC_DIVERGENCE_THRESHOLD_DEG     (90.0f)
#define QFC_GYRO_CALIBRATION_SAMPLES     (512U)

/* ========================================================================== */
/* SYSTEM STATES AND FAULT CODES                                              */
/* ========================================================================== */

/**
 * @brief Safety lifecycle states of the flight controller.
 */
typedef enum {
    SYSTEM_STATE_DISARMED = 0, /**< Boot default: motors forced 0.0 */
    SYSTEM_STATE_ARMED    = 1, /**< Active flight: motors driven by PID */
    SYSTEM_STATE_FAULT    = 2  /**< Emergency latch: motors zeroed permanently */
} system_state_t;

/**
 * @brief Categorized system fault codes.
 */
typedef enum {
    FAULT_NONE                 = 0x00,
    FAULT_IMU_COMM_FAIL        = 0x01, /**< I2C bus error or WHO_AM_I mismatch */
    FAULT_SENSOR_SATURATION    = 0x02, /**< Accelerometer or Gyro hit ADC rail */
    FAULT_CALIBRATION_TIMEOUT  = 0x03, /**< Gyro zero-rate calibration variance high */
    FAULT_ATTITUDE_DIVERGENCE  = 0x04, /**< Roll or Pitch exceeded +/- 90 degrees */
    FAULT_MIXER_OUT_OF_BOUNDS  = 0x05, /**< Motor mixing pre-clamp exceeded range */
    FAULT_SUPERLOOP_OVERRUN    = 0x06  /**< Control cycle execution exceeded 1 ms */
} fault_code_t;

/* ========================================================================== */
/* VECTOR AND SENSOR DATA STRUCTURES                                          */
/* ========================================================================== */

/**
 * @brief 3D Cartesian floating-point vector.
 */
typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

/**
 * @brief 3D Cartesian signed 16-bit integer vector (raw ADC counts).
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector3i16_t;

/**
 * @brief Raw sensor measurement frame from IMU burst read.
 */
typedef struct {
    vector3i16_t accel_raw;
    vector3i16_t gyro_raw;
    int16_t      temp_raw;
} imu_raw_data_t;

/**
 * @brief Physical engineering unit measurement frame.
 */
typedef struct {
    vector3f_t accel_mps2; /**< Accelerometer measurements in m/s^2 */
    vector3f_t gyro_dps;   /**< Gyroscope measurements in deg/s */
    float      temp_c;     /**< Temperature in degrees Celsius */
} imu_scaled_data_t;

/**
 * @brief Gyroscope zero-rate bias offsets.
 */
typedef struct {
    float x_dps;
    float y_dps;
    float z_dps;
} gyro_bias_t;

/**
 * @brief Euler attitude angles in degrees.
 */
typedef struct {
    float roll_deg;  /**< Rotation about X axis (-180 to +180 deg) */
    float pitch_deg; /**< Rotation about Y axis (-90 to +90 deg) */
} attitude_deg_t;

/**
 * @brief Target orientation setpoint.
 */
typedef struct {
    float roll_deg;
    float pitch_deg;
} setpoint_deg_t;

/**
 * @brief Dynamic stabilization demands computed by PID controllers.
 */
typedef struct {
    float roll_demand;
    float pitch_demand;
} pid_demands_t;

/**
 * @brief Quad-X normalized motor commands [0.0 to 1.0].
 */
typedef struct {
    float m1; /**< Front-Right (Counter-Clockwise) */
    float m2; /**< Rear-Right  (Clockwise) */
    float m3; /**< Rear-Left   (Counter-Clockwise) */
    float m4; /**< Front-Left  (Clockwise) */
} motor_commands_t;

#ifdef __cplusplus
}
#endif

#endif /* QFC_TYPES_H */
