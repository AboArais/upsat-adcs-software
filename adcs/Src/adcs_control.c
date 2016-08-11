/*
 * adcs_control.c
 *
 *  Created on: Jul 16, 2016
 *      Author: azisi
 */
#include "stm32f4xx_hal.h"

#include "adcs_control.h"
#include "adcs_configuration.h"
#include "adcs_common.h"

#include "sysview.h"

_adcs_control control = { .b_dot[0] = 0, .b_dot[1] = 0, .b_dot[2] = 0,
                          .b_dot_prev[0] = 0, .b_dot_prev[1] = 0,
                          .b_dot_prev[2] = 0,.k_bdot = BDOT_GAIN,
                          .k_pointing[0] = POINTING_GAIN_1,
                          .k_pointing[1] = POINTING_GAIN_2,
                          .sp_yaw = 0, .sp_pitch = 0, .sp_roll = 0,
                          .Ix = 0, .Iy = 0, .Iz = 0 ,
                          .k_spin = SPIN_TORQUER_GAIN,
                          .sp_rpm = 0, .const_rmp = SPIN_TORQUER_REF_RPM };

void b_dot(float b[3], float b_prev[3], float b_norm,
        _adcs_control *control_struct) {

    float b_dot_x, b_dot_y, b_dot_z = 0;

    control_struct->b_dot_prev[0] = control_struct->b_dot[0];
    control_struct->b_dot_prev[1] = control_struct->b_dot[1];
    control_struct->b_dot_prev[2] = control_struct->b_dot[2];
    /* Calculate B-dot */
    control_struct->b_dot[0] = (b[0] - b_prev[0]) / LOOP_TIME;
    control_struct->b_dot[1] = (b[1] - b_prev[1]) / LOOP_TIME;
    control_struct->b_dot[2] = (b[2] - b_prev[2]) / LOOP_TIME;
    /* Moving average for B-dot */
    b_dot_x = BDOT_FILTER * control_struct->b_dot[0]
            + (1 - BDOT_FILTER) * control_struct->b_dot_prev[0];
    b_dot_y = BDOT_FILTER * control_struct->b_dot[1]
            + (1 - BDOT_FILTER) * control_struct->b_dot_prev[1];
    b_dot_z = BDOT_FILTER * control_struct->b_dot[2]
            + (1 - BDOT_FILTER) * control_struct->b_dot_prev[2];
    /* Calculate the currents of coils in A */
    control_struct->Ix = -((float) (control_struct->k_bdot) * 0.1 / A_COIL) * b_dot_x
            * (1 / b_norm);
    control_struct->Iy = -((float) (control_struct->k_bdot) * 0.1 / A_COIL) * b_dot_y
            * (1 / b_norm);
    control_struct->Iz = -((float) (control_struct->k_bdot) * 0.1 / A_COIL) * b_dot_z
            * (1 / b_norm);

}

static float rpm_in_prev = 0;
static float rpm_out_prev = 0;
static float rpm_sum = 0;

void spin_torquer_controller(float w, _adcs_control *control_struct) {

    rpm_in_prev = rpm_sum;
    rpm_out_prev = control_struct->sp_rpm;
    /* Integration of RPM */

    rpm_sum += (-(float) (control_struct->k_spin) * 0.001)
            * (w * RAD2RPM / I_SPIN_TORQUER) * LOOP_TIME;

    SYSVIEW_PRINT("SPIN CONTROL1 %.2f", rpm_sum);

    /* Check for saturation */
    if (rpm_sum > SATURATION_RPM) {
        rpm_sum = SATURATION_RPM;
    } else if (rpm_sum < -SATURATION_RPM) {
        rpm_sum = -SATURATION_RPM;
    }

    SYSVIEW_PRINT("SPIN CONTROL2 %.2f", rpm_sum);

    /* Filter the output of RPM */
    control_struct->sp_rpm = rpm_sum - SPIN_TORQUER_FILTER_Z *
            rpm_in_prev + SPIN_TORQUER_FILTER_P * rpm_out_prev;

    SYSVIEW_PRINT("SPIN CONTROL3 %.2f", control_struct->sp_rpm);

    /* Check for saturation if output RPM */
    if (control_struct->sp_rpm > SATURATION_RPM) {
        control_struct->sp_rpm = SATURATION_RPM;
    } else if (control_struct->sp_rpm < -SATURATION_RPM) {
        control_struct->sp_rpm = -SATURATION_RPM;
    }

}
