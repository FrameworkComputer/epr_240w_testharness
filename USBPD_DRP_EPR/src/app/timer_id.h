/***************************************************************************//**
* File Name: timer_id.h
*
* Description: Provides Application Software Timer Identifier definitions.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _TIMER_ID_H_
#define _TIMER_ID_H_

#include <stdint.h>

#define GET_APP_TIMER_ID(context, id)                                 \
    (uint16_t)(((context)->port != 0U) ? ((((uint16_t)id) & 0x00FFU) + (uint16_t)CY_PDUTILS_TIMER_APP_PORT1_START_ID) : (uint16_t)(id))

typedef enum {
    APP_TIMERS_START_ID = CY_PDUTILS_TIMER_APP_PORT0_START_ID,
    /**< Start index for Application level timers. */

    APP_PSOURCE_EN_TIMER = APP_TIMERS_START_ID,
    /**< Timer used to ensure timely completion of power source enable operation. */

    APP_PSOURCE_EN_MONITOR_TIMER,
    /**< Timer used to monitor voltage during power source enable operation. */

    APP_PSOURCE_EN_HYS_TIMER,
    /**< Timer used to add hysteresis at the end of a power source enable operation. */

    APP_PSOURCE_DIS_TIMER,
    /**< Timer used to ensure timely completion of power source disable operation. */

    APP_PSOURCE_DIS_MONITOR_TIMER,
    /**< Timer used to monitor voltage during power source disable operation. */

    APP_PSOURCE_CF_TIMER,
    /**< Power source Current foldback restart timer ID. */

    APP_PSOURCE_DIS_EXT_DIS_TIMER,
    /**< Timer used to discharge VBus for some extra time at the end of a power source disable operation. */

    APP_DB_SNK_FET_DIS_DELAY_TIMER,
    /**< Dead battery Sink Fet disable delay timer. */

    APP_PSINK_DIS_TIMER,
    /**< Timer used to ensure timely completion of power sink disable operation. */

    APP_PSINK_DIS_MONITOR_TIMER,
    /**< Timer used to monitor voltage during power sink disable operation. */

    APP_VDM_BUSY_TIMER,
    /**< Timer used to delay retry of VDMs due to BUSY responses or errors. */

    APP_AME_TIMEOUT_TIMER,
    /**< Timer used to implement AME timeout. */

    APP_VBUS_OCP_OFF_TIMER,
    /**< Timer used to disable VBus supply after OC fault. */

    APP_VBUS_OVP_OFF_TIMER,
    /**< Timer used to disable VBus supply after OV fault. */

    APP_VBUS_UVP_OFF_TIMER,
    /**< Timer used to disable VBus supply after UV fault. */

    APP_VBUS_SCP_OFF_TIMER,
    /**< Timer used to disable VBus supply after SC fault. */

    APP_FAULT_RECOVERY_TIMER,
    /**< App timer used to delay port enable after detecting a fault. */

    APP_SBU_DELAYED_CONNECT_TIMER,
    /**< Timer used to do delayed SBU connection in Thunderbolt mode. */

    APP_MUX_DELAY_TIMER,
    /**< Timer used to delay VDM response. */

    APP_MUX_POLL_TIMER,
    /**< Timer used to MUX status. */

    APP_CBL_DISC_TRIGGER_TIMER,
    /**< Timer used to trigger cable discovery after a V5V supply change. */

    APP_V5V_CHANGE_DEBOUNCE_TIMER,
    /**< Timer used to debounce V5V voltage changes. */

    APP_VCONN_RECOVERY_TIMER,
    /**< Timer used to run Vconn swap after V5V was lost and recovered while UFP. */

    APP_OT_DETECTION_TIMER,
    /**< Timer used to call OT measurement handler. */

    APP_CHUNKED_MSG_RESP_TIMER,
    /**< Timer ID used to respond to chunked messages with NOT_SUPPORTED. */

    APP_RESET_VDM_LAYER_TIMER,
    /**< Timer used to run reset of VDM layer. */

    APP_BB_ON_TIMER,
    /**< Timer used to provide delay between disabling the Billboard device and re-enabling it. */

    APP_BB_OFF_TIMER,
    /**< Timer used to display USB billboard interface to save power. */

    APP_INITIATE_SWAP_TIMER,
    /**< Timer used to initiate SWAP operations in DRP applications with a power/data role preference. */

    APP_VDM_NOT_SUPPORT_RESP_TIMER_ID,
    /**< VDM Not supported response timer. */

    APP_BC_TIMERS_START_ID,
    /**< Start of Battery Charging State Machine timers. */

    APP_BC_GENERIC_TIMER1,
    /**< Generic timer #1 used by the BC state machine. */

    APP_BC_GENERIC_TIMER2,
    /**< Generic timer #2 used by the BC state machine. */

    APP_BC_DP_DM_DEBOUNCE_TIMER,
    /**< Timer used to debounce voltage changes on DP and DM pins. */

    APP_BC_DETACH_DETECT_TIMER,
    /**< Timer used to detect detach of a BC 1.2 sink while functioning as a CDP. */

    APP_CDP_DP_DM_POLL_TIMER,
    /**< Timer used to initiate DP/DM voltage polling while connected as a CDP. */

    APP_EPR_MODE_TIMER,
    /**< Timer used by EPR state machine. */

    APP_EPR_EXT_CMD_TIMER,
    /**< Timer used to send enter/exit EPR mode events to EPR state machine. */

    APP_TIMER_HPD_DELAY_TIMER,
    /**< This timer is used to delay HPD events. */

    APP_PSOURCE_VBUS_SET_TIMER_ID,
    /**< Power source VBUS set timer ID. */

    APP_PSOURCE_SAFE_FET_ON_MONITOR_TIMER_ID,
    /**< Timer to monitor voltage during FET On operation. */

    APP_PSOURCE_SAFE_FET_ON_TIMER_ID,
    /**< Timeout timer to set safe voltage during FET On operation. */

    VBUS_DISCHARGE_SCHEDULE_TIMER,
    /**< Timer for VBUS SLow Discharge */

    CCG_LS_MASTER_PORT_DEBOUNCE_TIMER_ID,
    /**< Macro defines Master Debounce Timer ID. */

    CCG_LS_SLAVE_PORT_DEBOUNCE_TIMER_ID,
    /**< Macro defines Slave Debounce Timer ID. */

    CCG_LS_MASTER_WRITE_TIMER_ID,
    /**< Macro defines Master Write Timer ID. */

    CCG_LS_HEART_BEAT_TIMER_ID,
    /**< Macro defines Heart Beat Timer ID. */

    THROTTLE_TIMER_ID,
    /**< Power Throttling timer ID. */

    THROTTLE_WAIT_FOR_PD_TIMER_ID,
    /**< Power Throttling timer ID. */

    APP_RESERVED_TIMER_ID,
    /**< Timer ID reserved for future use. */

    LINS_BUS_INACTIVE_TIMER,
    /**< Bus Inactivity Timeout for LIN. */
    
    LINS_BUS_LISTEN_TIMER,
    /**< Nominal Time for Reception of a single frame from BREAK */
    
    LINS_MULTIFRAME_DROP_TIMER,
    /**< Multiframe timer to drop the frame upon late reception. */

    APP_FET_SOFT_START_TIMER_ID,
    /**< Timer used to control soft turn-on of power FET gate drivers. */

    APP_HAL_VREG_TIMER,
    /**< Timer that can be used for Vreg fault handling. */

    APP_HAL_GENERIC_TIMER,
    /**< Timer that can be used for generic HAL functions. */

    APP_REGULATOR_STARTUP_MONITOR_TIMER,             
    /**< Timer ID reserved for regulator startup monitoring. */

    APP_DATA_RESET_TIMER,
    /**< Timer ID for DATA Reset handling. */

    SYS_BLACK_BOX_TIMER_ID,
    /**< Timer ID reserved for blackbox. */

    APP_PSOURCE_REGULATOR_MON_TIMER,
    /**< Timer ID used to monitor regulator enable status periodically. */

    APP_BAD_SINK_TIMEOUT_TIMER,
    /**< PD bad sink timeout timer ID. */

    APP_VBAT_GND_SCP_TIMER_ID,
    /**< VBAT-GND SCP recovery timer. */

    APP_VCONN_OCP_TIMER,
    /**< Timer to perform delayed start for VCONN OCP. */
    
    CCG_LS_SNK_CAP_TIMEOUT_TIMER_ID,
    /**< PD Timeout Timer for LS Slave. */

    APP_GPIO_HPD_TIMER_ID,
    /**< GPIO based HPD timer. */
    
    APP_VBUS_FET_ON_TIMER,
    /** Timer used for providing delay for VBUS FET ON. */

    APP_VBUS_FET_OFF_TIMER,
    /** Timer used for providing delay for VBUS FET OFF. */

    APP_VCONN_TURN_ON_DELAY_TIMER,
    /** Timer used for providing delay for VConn Gate Pull Up enable. */

} timer_id_t;

#endif /* _TIMER_ID_H_ */

/* [] END OF FILE */
