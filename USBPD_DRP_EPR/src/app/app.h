/******************************************************************************
* File Name: app.h
*
* Description: This is header file for the PMG1 MCU USB-PD DRP Code Example.
*              This file defines data structures and function prototypes
*              associated with application level management of the USB-C Port.
*
* Related Document: See README.md
*
*******************************************************************************
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

#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>

#include "config.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/
#define APP_MAX_SWAP_ATTEMPT_COUNT                      (10u)
/**< Number of swap attempts to be made when port partner is sending a WAIT response. */

/*************** Timer duration settings. Changes not recommended. ********************/

#define APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD            (10u)
/**< Duration of extra VBus discharge after voltage drops below desired level (in ms). */

#define APP_PSINK_DIS_TIMER_PERIOD                      (250u)
/**< Maximum time allowed for power sink disable operation (in ms). */

#define APP_PSINK_DIS_MONITOR_TIMER_PERIOD              (1u)
/**< Period of VBus voltage checks performed during power sink disable operation. */

#define APP_FAULT_RECOVERY_TIMER_PERIOD                 (100u)
/**< Period of VBus presence checks after a fault (Over-Voltage) detection while in a sink contract. */

#define APP_FAULT_RECOVERY_MAX_WAIT                     (500u)
/**< Time for which VBus will be monitored to ensure removal of VBus by a faulty power source. */

#define APP_CBL_DISC_TIMER_PERIOD                       (100u)
/**< Delay to be used between cable discovery init commands. */

#define APP_VDM_BUSY_TIMER_PERIOD                       (50u)
/**< VDM busy timer period (in ms). */

#define APP_VDM_FAIL_RETRY_PERIOD                       (100u)
/**< VDM retry (on failure) timer period in ms. */

#define APP_CABLE_POWER_UP_DELAY                        (55u)
/**< Time allowed for cable power up to be complete. */

#define APP_CABLE_VDM_START_DELAY                       (5u)
/**< Cable query delay period in ms. */

#define APP_AME_TIMEOUT_TIMER_PERIOD                    (800u)
/**< tAME timer period (in ms). */

#define APP_INITIATE_DR_SWAP_TIMER_PERIOD               (5u)
/**< Time delay between successive DR_SWAP attempts (in ms). */

#define APP_INITIATE_PR_SWAP_TIMER_PERIOD               (500u)
/**< Time delay between successive PR_SWAP attempts (in ms). PR_SWAP attempts are kept slow to allow
     other sequences such as alternate mode negotiation to complete. */

#define APP_RESET_VDM_TIMER_PERIOD                      (2u)
/**< Reset VDM layer time period. */

#define APP_VCONN_SWAP_PENDING                          (1u)
/**< VConn swap pending bit in the app. level pending swaps flag. */

#define APP_DR_SWAP_PENDING                             (2u)
/**< Data Role swap pending bit in the app. level pending swaps flag. */

#define APP_PR_SWAP_PENDING                             (4u)
/**< Power Role swap pending bit in the app. level pending swaps flag. */

#define APP_VCONN_RECOVERY_PERIOD                       (500u)
/**< Delay between VConn fault and recovery attempt. */

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
 * @typedef app_port_fault_status_mask_t
 * @brief Fault detection and handling related status bits tracked in the fault_status field.
 */
typedef enum {
    APP_PORT_FAULT_NONE                 = 0x00, /**< System functioning without any fault. */
    APP_PORT_VCONN_FAULT_ACTIVE         = 0x01, /**< Status bit that indicates VConn fault is active. */
    APP_PORT_SINK_FAULT_ACTIVE          = 0x02, /**< Status bit that indicates sink fault handling is pending. */
    APP_PORT_SRC_FAULT_ACTIVE           = 0x04, /**< Status bit that indicates source fault handling is pending. */
    APP_PORT_VBUS_DROP_WAIT_ACTIVE      = 0x08, /**< Status bit that indicates wait for VBus drop is pending. */
    APP_PORT_V5V_SUPPLY_LOST            = 0x10, /**< Status bit that indicates that V5V supply (for VConn) has been lost. */
    APP_PORT_DISABLE_IN_PROGRESS        = 0x80  /**< Port disable operation is in progress. */
} app_port_fault_status_mask_t;

/**
   @brief This structure hold all VDM Configuration Info.
 */
typedef struct
{
    uint32_t discId[7];
    uint16_t dIdLength;
    uint32_t sVid[7];
    uint16_t sVidLength;
    uint32_t discMode[7];
    uint16_t disModeLength;
} vdm_info_config_t;

/**
   @brief This structure hold all variables related to application layer functionality.
 */
typedef struct
{
    cy_pdstack_pwr_ready_cbk_t pwr_ready_cbk;        /**< Registered Power source callback. */
    cy_pdstack_sink_discharge_off_cbk_t snk_dis_cbk; /**< Registered Power sink callback. */
    app_resp_t appResp;                  /**< Buffer for APP responses. */
    vdm_resp_t vdmResp;                  /**< Buffer for VDM responses. */
    uint16_t psrc_volt;                   /**< Current Psource voltage in mV */
    uint16_t psrc_volt_old;               /**< Old Psource voltage in mV */
    uint16_t psnk_volt;                   /**< Current PSink voltage in mV units. */
    uint16_t psnk_cur;                    /**< Current PSink current in 10mA units. */
    uint8_t disc_cbl_pending;             /**< Flag to indicate is cable discovery is pending. */
    uint8_t cbl_disc_id_finished;         /**< Flag to indicate that cable disc id finished. */
    uint8_t vdm_version;                  /**< Live VDM major version. */
    volatile uint8_t fault_status;        /**< Fault status bits for this port. */
    bool is_vbus_on;                      /**< Is supplying VBUS flag. */
    bool is_vconn_on;                     /**< Is supplying VCONN flag. */
    bool vdm_retry_pending;               /**< Whether VDM retry on timeout is pending. */
    bool psrc_rising;                     /**< Voltage ramp up/down. */
    cy_pdstack_vdm_resp_cbk_t vdm_resp_cbk;          /**< VDM response handler callback. */
    bool is_vdm_pending;                  /**< VDM handling flag for MUX callback. */
    uint8_t app_pending_swaps;            /**< Variable denoting the types of swap operation that are pending. */
    uint8_t actv_swap_type;               /**< Denotes the active Swap operation. */
    uint8_t actv_swap_count;              /**< Denotes number of active swap attempts completed. */
    uint16_t actv_swap_delay;             /**< Delay to be applied between repeated swap attempts. */
    bool debug_acc_attached;              /**< Debug accessory attach status */
    uint8_t vdm_minor_version;            /**< Live VDM minor version. */
} app_status_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Application level init function.
 *
 * This function performs any Application level initialization required
 * for the CCG solution. This should be called before calling the
 * dpmInit function.
 *
 * @return None.
 *
 */
void app_init(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Handler for application level asynchronous tasks.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return 1 in case of success, 0 in case of task handling error.
 */
uint8_t app_task(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function return the App callback structure pointer
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return  Application callback structure pointer
 */
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context);

/**
 * @brief Handler for event notifications from the PD stack.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param evt Type of event to be handled.
 * @param dat Data associated with the event.
 * @return None
 */
void app_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, 
               cy_en_pdstack_app_evt_t evt, const void* dat);

/**
 * @brief Get a handle to the application provide PD command response buffer.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return Pointer to the response buffer.
 */
app_resp_t* app_get_resp_buf(uint8_t port);

/**
 * @brief Get handle to structure containing information about the system status for a PD port.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return Pointer to the system information structure.
 */
app_status_t* app_get_status(uint8_t port);

/**
 * @brief Check whether the APP handlers are ready to allow device deep sleep.
 * @return true if APP handler is idle, false otherwise.
 */
bool app_sleep (void);

/**
 * @brief Restore the APP handler state after CCG device wakes from deep-sleep.
 * @return None
 */
void app_wakeup (void);

/**
 * @brief Function to place CCG device in power saving mode if possible.
 *
 * This function places the CCG device in power saving deep sleep mode
 * if possible. The function checks for each interface (PD, HPI etc.)
 * being idle and then enters sleep mode with the appropriate wake-up
 * triggers. If the device enters sleep mode, the function will only
 * return after the device has woken up.
 *
 * @return true if the device went into sleep, false otherwise.
 */
bool system_sleep(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pdstack_context_t *ptrPdStack1Context);

/*****************************************************************************
  Functions related to power
 *****************************************************************************/

/**
 * @brief This function enables VCONN power
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param channel Selected CC line.
 *
 * @return True if VConn was turned ON; false if NOT.
 */

bool vconn_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel);

/**
 * @brief This function disables VCONN power
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param channel Selected CC line.
 *
 * @return None
 */
void vconn_disable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel);

/**
 * @brief This function checks if power is present on VConn
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return true if power is present on VConn, else returns false
 */
bool vconn_is_present(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function checks if power is present on VBus
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param volt Voltage in mV units.
 * @param per  Threshold margin.
 *
 * @return true if power is present on VBus, else returns false
 */
bool vbus_is_present(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt, int8_t per);

/**
 * @brief This function return current VBUS voltage in mV
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return VBUS voltage in mV
 */
uint16_t vbus_get_value(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function turns on dischange FET on selected port
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return None
 */
void vbus_discharge_on(cy_stc_pdstack_context_t* context);

/**
 * @brief This function turns off dischange FET on selected port
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 *
 * @return None
 */
void vbus_discharge_off(cy_stc_pdstack_context_t* context);

/**
 * @brief Enable and configure the Over-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param ovp_cb Callback function to be triggered when there is an OV event.
 * @return None
 */
void app_ovp_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t ovp_cb);

/**
 * @brief Disable the Over-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_ovp_disable(cy_stc_pdstack_context_t *ptrPdStackContext, bool pfet);

/**
 * @brief Enable and configure the Under-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param uvp_cb Callback function to be triggered when there is an UV event.
 * @return None
 */
void app_uvp_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt_mV, bool pfet, cy_cb_vbus_fault_t uvp_cb);

/**
 * @brief Disable the Under-Voltage protection circuitry.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_uvp_disable(cy_stc_pdstack_context_t *ptrPdStackContext, bool pfet);

/*****************************************************************************
  Functions to be provided at the solution level.
 *****************************************************************************/

/**
 * @brief Solution handler for PD events reported from the stack.
 *
 * The function provides all PD events to the solution. For a solution
 * supporting HPI, the solution function should re-direct the calls to
 * hpi_pd_event_handler.
 *
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that
 * needs to be de-referenced based on event type.
 *
 * @return None
 */
void sln_pd_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, 
               cy_en_pdstack_app_evt_t evt, const void *data);

/**
 * @brief Initialize fault-handling related variables from the configuration table.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return false if the configuration table is invalid, true otherwise.
 */
bool fault_handler_init_vars (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Clear the fault occurrence counts after a Type-C detach is detected.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return None.
 */
void fault_handler_clear_counts (uint8_t port);

/**
 * @brief Perform any fault handler related tasks.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return None
 */
void fault_handler_task (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Handle any application events associated with fault handling logic.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param evt Event code.
 * @param dat Data associated with the event code.
 * @return true if the event does not need to be passed up to the solution layer.
 */
bool fault_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_app_evt_t evt, const void *dat);

/**
 * @brief Prepare the CCG to wait for physical detach of faulty port partner.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return None
 */
void app_conf_for_faulty_dev_removal(cy_stc_pdstack_context_t * ptrPdStackcontext);

/**
 * @brief Check whether any fault count has exceeded limit for the specified PD port. 
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return true if fault count has exceeded limit, false otherwise.
 */    
bool app_port_fault_count_exceeded(cy_stc_pdstack_context_t * ptrPdStackcontext);

/**
 * @brief Function to handle VConn supply state change due to OCP condition or V5V supply change.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @param vconn_on Whether VConn has just turned ON or OFF.
 * @return None
 */
void vconn_change_handler(cy_stc_pdstack_context_t * ptrPdStackcontext, bool vconn_on);

/**
 * @brief This function is called at the end of a PD contract to check whether any
 * role swaps need to be triggered. 
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return void.
 */

void app_contract_handler (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief This function is called whenever there is a change in the connection.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return void.
 */
void app_connect_change_handler (cy_stc_pdstack_context_t *ptrPdStackContext);

#if ((CY_PD_EPR_ENABLE) && (!CY_PD_SINK_ONLY))
bool eval_epr_mode(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_eprmdo_action_t eprModeState, cy_pdstack_app_resp_cbk_t app_resp_handler);
bool send_epr_cap(cy_stc_pdstack_context_t *ptrPdStackContext, cy_pdstack_app_resp_cbk_t app_resp_handler);
#endif /* ((CY_PD_EPR_ENABLE) && (!CY_PD_SINK_ONLY)) */

/**
 * @brief This function is called whenever there is a get source info message is received.
 * @param ptrPdStackContext Pointer to the pdstack context.
 * @return true if source info can be send
 */
bool send_src_info (struct cy_stc_pdstack_context *ptrPdStackContext);

#endif /* _APP_H_ */

/* End of File */
