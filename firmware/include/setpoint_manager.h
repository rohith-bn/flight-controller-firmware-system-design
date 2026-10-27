/**
 * @file setpoint_manager.h
 * @brief Target Orientation Setpoint Manager (Opaque ADT)
 * @author Rohith B Narasimhamurthy
 * @standard ISO C99 / MISRA-C Compliance
 * 
 * Category A module. Encapsulates target attitude setpoints with boundary validation
 * against maximum allowable tilt angles (+/- 45.0 degrees). Provides a clean decoupled
 * seam between pilot input sources and the inner-loop PID attitude controller.
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

#include "qfc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Setpoint manager state instance.
 */
typedef struct {
    float roll_target_deg;  /**< Target roll orientation in degrees */
    float pitch_target_deg; /**< Target pitch orientation in degrees */
    bool  b_initialized;    /**< State initialization flag */
} setpoint_manager_t;

/**
 * @brief Initializes the setpoint manager with default level hover setpoints (0.0, 0.0 deg).
 * 
 * @param[out] p_mgr Pointer to setpoint manager instance
 * @return bool true on success, false if p_mgr is NULL
 */
bool setpoint_manager_init(setpoint_manager_t *p_mgr);

/**
 * @brief Sets new target attitude angles with safety boundary clamping.
 * 
 * If requested setpoint exceeds QFC_MAX_ALLOWABLE_TILT_DEG (+/- 45 deg),
 * the value is clamped to the boundary and false is returned to signal saturation.
 * 
 * @param[in,out] p_mgr     Pointer to setpoint manager instance
 * @param[in]     roll_deg  Requested roll setpoint in degrees
 * @param[in]     pitch_deg Requested pitch setpoint in degrees
 * @return bool true if targets were within limits, false if clamped or p_mgr is NULL
 */
bool setpoint_manager_set(setpoint_manager_t *p_mgr, float roll_deg, float pitch_deg);

/**
 * @brief Retrieves current target attitude angles.
 * 
 * @param[in]  p_mgr      Pointer to setpoint manager instance
 * @param[out] p_setpoint Destination structure for retrieved setpoints
 * @return bool true on success, false if any pointer is NULL or uninitialized
 */
bool setpoint_manager_get(const setpoint_manager_t *p_mgr, setpoint_deg_t *p_setpoint);

/**
 * @brief Resets target setpoints back to default level hover (0.0, 0.0 deg).
 * 
 * @param[in,out] p_mgr Pointer to setpoint manager instance
 * @return bool true on success, false if p_mgr is NULL or uninitialized
 */
bool setpoint_manager_reset(setpoint_manager_t *p_mgr);

#ifdef __cplusplus
}
#endif

#endif /* SETPOINT_MANAGER_H */
