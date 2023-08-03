/******************************************************************************
* File Name: config.h
*
* Description: This header file defines the application configuration for the PMG1
*              MCU USBPD DRP EPR example for ModusToolBox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "cybsp.h"
#include "ncp81239.h"
#include "cy_pdutils_sw_timer.h"

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#ifndef NO_OF_TYPEC_PORTS
#define NO_OF_TYPEC_PORTS                         (CY_IP_MXUSBPD_INSTANCES)
#endif /* NO_OF_TYPEC_PORTS */

/*******************************************************************************
 * Enable PD spec Rev 3 support
 ******************************************************************************/
#if CY_PD_REV3_ENABLE
    #define CY_PD_FRS_RX_ENABLE                   (0u)
    #define CY_PD_FRS_TX_ENABLE                   (0u)
    #define CY_PD_PPS_SRC_ENABLE                  (0u)
#endif /* CY_PD_REV3_ENABLE */

#define CCG_PROG_SOURCE_ENABLE                    (1u)


/* Enable hardware based DRP toggle for additional power saving. */
#define CY_PD_HW_DRP_TOGGLE_ENABLE                (0u)

/*
 * Macro defines additional delay in milliseconds before the PD stack starts sending
 * SRC_CAP message. This may be required to work with some non-compliant sink devices
 * which require more start up time for PD.
 */
#define DELAY_SRC_CAP_START_MS                  (100u)

#define PD_PDO_SEL_ALGO                         (0u)

/* VBUS PGDO FET Control selection based on the FET control pin used in the system hardware */
#define VBUS_FET_CTRL_0                             (1u)
#define VBUS_FET_CTRL_1                             (0u)

#define VBUS_FET_CTRL                               (VBUS_FET_CTRL_0)

/*******************************************************************************
 * USB-PD SAR ADC Configurations
 ******************************************************************************/

#define APP_VBUS_POLL_ADC_ID                    (CY_USBPD_ADC_ID_0)
#if defined(CY_DEVICE_CCG3)
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_A)
#else
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_B)
#endif /* defined(CY_DEVICE_CCG3) */

/*******************************************************************************
 * Power Sink (PSINK) controls
 ******************************************************************************/

/* VBUS in discharge enable */
#define VBUS_IN_DISCHARGE_EN                    (0u)

/* 
 * Allow VBUS_IN discharge below 5V.
 * When VBUS_IN_DISCHARGE_EN macro is enabled, VBUS_IN discharge is enabled for all
 * VBUS downward transitions above 5V, but is disabled for transitions below 5V.
 * Because, for VBUS_IN powered solutions, VBUS_IN should not be accidently
 * brought to the low voltage where system behavior is undefined. 
 * VBUS_IN discharge below 5V may be required for solutions where regulator
 * requires higher discharge strength to set voltage below 5V.
 * It is recommended to enable this feature only for solution which are not
 * VBUS_IN powered.
 */
#define VBUS_IN_DISCH_BELOW_5V_EN               (0u)

/*******************************************************************************
 * Power Source (PSOURCE) Configuration.
 ******************************************************************************/

/* Time (in ms) allowed for source voltage to become valid. */
#define APP_PSOURCE_EN_TIMER_PERIOD             (250u)

/* Period (in ms) of VBus validity checks after enabling the power source. */
#define APP_PSOURCE_EN_MONITOR_TIMER_PERIOD     (10u)

/* Time (in ms) between VBus valid and triggering of PS_RDY. */
#define APP_PSOURCE_EN_HYS_TIMER_PERIOD         (10u)

/* Time (in ms) for which the VBus_Discharge path will be enabled when turning power source OFF. */
#define APP_PSOURCE_DIS_TIMER_PERIOD            (600u)

/* Period (in ms) of VBus drop to VSAFE0 checks after power source is turned OFF. */
#define APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD    (100u)

/* VBus Monitoring is done using internal resistor divider. */
#define VBUS_MON_INTERNAL                       (1u)

/* Period in ms for turning on VBus FET. */
#define APP_VBUS_FET_ON_TIMER_PERIOD           (10u)

/* Period in ms for turning off VBus FET. */
#define APP_VBUS_FET_OFF_TIMER_PERIOD           (1u)

/* Time (in ms) for which the VBus_Discharge path will be enabled when turning power source OFF. */
#define APP_PSOURCE_EPR_DIS_TIMER_PERIOD        (1260u)

/* Time (in ms) allowed for source voltage to become valid. */
#define APP_PSOURCE_EPR_EN_TIMER_PERIOD         (860u)

#if (defined (CY_DEVICE_PMG1S3))
/* Enable / Disable VBUS soft start feature */
#define VBUS_SOFT_START_ENABLE                  (0u)
#endif /* (defined (CY_DEVICE_PMG1S3)) */

#if (defined (CY_DEVICE_PMG1S3))
/* Function/Macro to set P1 source voltage to contract value. */
#define APP_VBUS_SET_VOLT_P1(mV)                    \
{                                                   \
    set_pd_ctrl_voltage(TYPEC_PORT_0_IDX, mV);      \
}
/* Function/Macro to set P2 source voltage to contract value. */
#define APP_VBUS_SET_VOLT_P2(mV)                    \
{                                                   \
    set_pd_ctrl_voltage(TYPEC_PORT_1_IDX, mV);      \
}
#endif /* (defined (CY_DEVICE_PMG1S3)) */

/*******************************************************************************
 * VBus monitor configuration.
 ******************************************************************************/

/* Allowed VBus valid margin as percentage of expected voltage. */
#define VBUS_TURN_ON_MARGIN                     (-20)

/* Allowed VBus valid margin (as percentage of expected voltage) before detach detection is triggered. */
#define VBUS_TURN_OFF_MARGIN                    (-20)

/* Allowed margin over expected voltage (as percentage) for negative VBus voltage transitions. */
#define VBUS_DISCHARGE_MARGIN                   (10)

/* Allowed margin over 5V before the provider FET is turned OFF when discharging to VSAFE0. */
#define VBUS_DISCHARGE_TO_5V_MARGIN             (10)

/* Allowed margin over expected voltage (as percentage). */
#define VBUS_NEW_VALID_MARGIN                   (5)

/*******************************************************************************
 * System fault configuration features.
 ******************************************************************************/

/* 
 * Enable/Disable delay between fault retries for Type-C/PD faults.
 */
#define FAULT_RETRY_DELAY_EN                        (0u)

#if FAULT_RETRY_DELAY_EN

/* 
 * Delay between fault retries in mS.
 */
#define FAULT_RETRY_DELAY_MS                        (500u)

#endif /* FAULT_RETRY_DELAY_EN */

/* 
 * Enable/Disable delayed infinite fault recovery for Type-C/PD faults.
 * Fault recovery shall be tried with a fixed delay after configured 
 * fault retry count is elapsed. 
 */
#define FAULT_INFINITE_RECOVERY_EN                  (0u)

#if FAULT_INFINITE_RECOVERY_EN

/* 
 * Delayed fault recovery period in mS.
 */
#define FAULT_INFINITE_RECOVERY_DELAY_MS            (5000u)

#endif /* FAULT_INFINITE_RECOVERY_EN */

/* Enable watchdog hardware reset for CPU lock-up recovery */
#define WATCHDOG_HARDWARE_RESET_ENABLE              (1u)

/* Disable device reset on error (watchdog expiry or hard fault). */
#define RESET_ON_ERROR_ENABLE                       (1u)

/*
 * Watchdog reset period in ms. This should be set to a value greater than
 * 500 ms to avoid significant increase in power consumption.
 */
#define WATCHDOG_RESET_PERIOD_MS                    (750u)

/* Enable tracking of maximum stack usage. */
#define STACK_USAGE_CHECK_ENABLE                    (0u)

/*******************************************************************************
 * Firmware feature configuration.
 ******************************************************************************/
/* Enable saving only SVIDs which are supported by CCG. */
#define SAVE_SUPP_SVID_ONLY                         (1u)

/** Timer period in ms for providing delay for VConn Gate Pull Up enable. */
#define APP_VCONN_TURN_ON_DELAY_PERIOD              (1u)

#define APP_EPR_SNK_ENTRY_TIMER_PERIOD              (100u)
#define APP_EPR_SNK_EXIT_TIMER_PERIOD               (10u)

/***********************************************************************************/

/* Enable selection of data/power role preference. */
#define ROLE_PREFERENCE_ENABLE                      (0u)
#define POWER_ROLE_PREFERENCE_ENABLE                (0u)

#endif /* _CONFIG_H_ */

/* End of file [] */
