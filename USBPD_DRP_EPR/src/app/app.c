/******************************************************************************
* File Name: app.c
*
* Description: This is source code for the PMG1 MCU USB-PD DRP Code Example.
*              This file implements functions for handling of PDStack event
*              callbacks and power saving.
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

#include "cybsp.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "config.h"
#include "psource.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "app.h"
#include "cy_pdutils_sw_timer.h"
#include "timer_id.h"
#include "cy_pdstack_timer_id.h"
#include "cy_gpio.h"

extern cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx);

app_status_t app_status[NO_OF_TYPEC_PORTS];

#if (!CY_PD_CBL_DISC_DISABLE)
static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext);
static void app_cbl_dsc_callback (cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    /* Keep repeating the DPM command until we succeed. */
    if (resp == CY_PDSTACK_SEQ_ABORTED)
    {
        Cy_PdUtils_SwTimer_Start (ptrPdStackContext->ptrTimerContext, ptrPdStackContext , GET_APP_TIMER_ID(ptrPdStackContext, APP_CBL_DISC_TRIGGER_TIMER), APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}


static void app_cbl_dsc_timer_cb (cy_timer_id_t id, void *callbackContext)
{
    cy_stc_pdstack_context_t *pdstack_context = callbackContext;

    if (Cy_PdStack_Dpm_SendPdCommand(pdstack_context, CY_PDSTACK_DPM_CMD_INITIATE_CBL_DISCOVERY, NULL, false, app_cbl_dsc_callback) != CY_PDSTACK_STAT_SUCCESS )
    {
        /* Start timer which will send initiate the DPM command after a delay. */
        app_cbl_dsc_callback(pdstack_context, CY_PDSTACK_SEQ_ABORTED, 0);
    }
}
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

uint8_t app_task(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    fault_handler_task (ptrPdStackContext);

    return true;
}

#if SYS_DEEPSLEEP_ENABLE

bool app_deepsleep_allowed(void)
{
    return true;
}

bool app_sleep(void)
{
    bool stat = true;
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        /* Don't go to sleep while CC/SBU fault handling is pending. */
        if ((app_get_status(port)->fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
        {
            stat = false;
            break;
        }
   }

    return stat;
}

void app_wakeup(void)
{
    /* Application level wake up code to be placed here. */
}
#endif /* SYS_DEEPSLEEP_ENABLE */

#if CY_PD_REV3_ENABLE

/* Temporary storage for ongoing AMS type while handling chunked extended messages. */
static cy_en_pdstack_ams_type_t gl_extd_amsType[NO_OF_TYPEC_PORTS]; 

/* Global variable used as dummy data buffer to send Chunk Request messages. */
static uint32_t gl_extd_dummy_data;

void extd_msg_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_resp_status_t resp,
                                                    const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    (void)pkt_ptr;

    if(resp == CY_PDSTACK_RES_RCVD)
    {
        ptrPdStackContext->dpmStat.nonIntrResponse = gl_extd_amsType[ptrPdStackContext->port];
    }
    if(resp == CY_PDSTACK_CMD_SENT)
    {
        gl_extd_amsType[ptrPdStackContext->port] = ptrPdStackContext->dpmStat.nonIntrResponse;
    }
}

bool app_extd_msg_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pd_packet_extd_t *pd_pkt_p)
{
    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == true) && (pd_pkt_p->hdr.hdr.dataSize >
               ((pd_pkt_p->hdr.hdr.chunkNum + 1) * CY_PD_MAX_EXTD_MSG_LEGACY_LEN)))
    {
        cy_stc_pdstack_dpm_pd_cmd_buf_t extd_dpm_buf;

        extd_dpm_buf.cmdSop = (cy_en_pd_sop_t)pd_pkt_p->sop;
        extd_dpm_buf.extdType = (cy_en_pdstack_extd_msg_t)pd_pkt_p->msg;
        extd_dpm_buf.extdHdr.val = 0;
        extd_dpm_buf.extdHdr.extd.chunked = true;
        extd_dpm_buf.extdHdr.extd.request = true;
        extd_dpm_buf.extdHdr.extd.chunkNum = pd_pkt_p->hdr.hdr.chunkNum + 1;
        extd_dpm_buf.datPtr = (uint8_t*)&gl_extd_dummy_data;
        extd_dpm_buf.timeout = ptrPdStackContext->senderRspTimeout;

        /* Send next chunk request */
        Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,CY_PDSTACK_DPM_CMD_SEND_EXTENDED,
                                     &extd_dpm_buf, true, extd_msg_cb); 

     }
    else
    {
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext,
                                         CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL); 
        }
    }

    return true;
}
#endif /* CCG_PD_REV3_ENABLE */

uint32_t get_bat_status[NO_OF_TYPEC_PORTS];

#if (ROLE_PREFERENCE_ENABLE)

/* Variable storing current preference for data role. */
volatile uint8_t app_pref_data_role[NO_OF_TYPEC_PORTS];

#if (POWER_ROLE_PREFERENCE_ENABLE)
/* Variable storing current preference for power role. */
volatile uint8_t app_pref_power_role[NO_OF_TYPEC_PORTS];
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

/* Forward declaration of function to trigger swap operations. */
static void app_initiate_swap (cy_timer_id_t id, void *context);

static void app_role_swap_resp_cb (cy_stc_pdstack_context_t *ptrPdStackContext, 
        cy_en_pdstack_resp_status_t resp,
        const cy_stc_pdstack_pd_packet_t *pkt_ptr)
{
    uint8_t port = ptrPdStackContext->port;
    app_status_t *app_stat = &app_status[port];

#if (POWER_ROLE_PREFERENCE_ENABLE)
    bool next_swap = false;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

    if (resp == CY_PDSTACK_RES_RCVD)
    {
        if (pkt_ptr->hdr.hdr.msgType == CY_PD_CTRL_MSG_WAIT)
        {
            app_stat->actv_swap_count++;
            if (app_stat->actv_swap_count < APP_MAX_SWAP_ATTEMPT_COUNT)
            {
                Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                        GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER), app_stat->actv_swap_delay, app_initiate_swap);
            }
            else
            {
#if (POWER_ROLE_PREFERENCE_ENABLE)
                /* Swap attempts timed out. Proceed with next swap. */
                next_swap = true;
#else
                app_stat->app_pending_swaps = 0;
                app_stat->actv_swap_type  = 0;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
            }
        }
        else
        {
#if (POWER_ROLE_PREFERENCE_ENABLE)
            /* Swap succeeded or failed. Proceed with next swap. */
            next_swap = true;
#else
            app_stat->app_pending_swaps = 0;
            app_stat->actv_swap_type  = 0;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        }
    }
    else if ((resp == CY_PDSTACK_CMD_FAILED) || (resp == CY_PDSTACK_SEQ_ABORTED) || (resp == CY_PDSTACK_RES_TIMEOUT))
    {
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER), app_stat->actv_swap_delay, app_initiate_swap);
    }

#if (POWER_ROLE_PREFERENCE_ENABLE)
    if (next_swap)
    {
        /* Clear the LS bit in the app_pending_swaps flag as it has completed or failed. */
        app_stat->app_pending_swaps &= (app_stat->app_pending_swaps - 1);

        app_stat->actv_swap_type  = 0;
        app_stat->actv_swap_count = 0;
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER), APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
    }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
}

static void app_initiate_swap (cy_timer_id_t id, void *context)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)context;

    uint8_t port = ptrPdStackContext->port;
    app_status_t *app_stat_p = app_get_status(port);

    cy_stc_pdstack_dpm_pd_cmd_buf_t pd_cmd_buf;

    uint8_t actv_swap     = app_stat_p->actv_swap_type;
    uint8_t swaps_pending = app_stat_p->app_pending_swaps;

    /* Stop the timer that triggers swap operation. */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER));

    /* Nothing to do if we are not in PD contract. */
    if (!ptrPdStackContext->dpmConfig.contractExist)
        return;

    if (actv_swap == 0)
    {
#if (POWER_ROLE_PREFERENCE_ENABLE)
        /* No ongoing swap operation. Pick the next pending swap from the list. */
        if ((swaps_pending & APP_VCONN_SWAP_PENDING) != 0)
        {
            actv_swap = CY_PDSTACK_DPM_CMD_SEND_VCONN_SWAP;
            app_stat_p->actv_swap_delay = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
        }
        else
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        {
            if ((swaps_pending & APP_DR_SWAP_PENDING) != 0)
            {
                actv_swap = CY_PDSTACK_DPM_CMD_SEND_DR_SWAP;
                app_stat_p->actv_swap_delay = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
            }
#if (POWER_ROLE_PREFERENCE_ENABLE)
            else
            {
                if (swaps_pending != 0)
                {
                    actv_swap = CY_PDSTACK_DPM_CMD_SEND_PR_SWAP;
                    app_stat_p->actv_swap_delay = APP_INITIATE_PR_SWAP_TIMER_PERIOD;
                }
            }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
        }

        app_stat_p->actv_swap_count = 0;
    }

    if (actv_swap != 0)
    {
        /* Check whether the selected swap is still valid. */
        switch (actv_swap)
        {
#if (POWER_ROLE_PREFERENCE_ENABLE)
            case CY_PDSTACK_DPM_CMD_SEND_VCONN_SWAP:
                if (ptrPdStackContext->dpmConfig.vconnLogical)
                {
                    app_stat_p->app_pending_swaps &= ~APP_VCONN_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

            case CY_PDSTACK_DPM_CMD_SEND_DR_SWAP:
                if (ptrPdStackContext->dpmConfig.curPortType == app_pref_data_role[port])
                {
                    app_stat_p->app_pending_swaps &= ~APP_DR_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;

#if (POWER_ROLE_PREFERENCE_ENABLE)
            case CY_PDSTACK_DPM_CMD_SEND_PR_SWAP:
                if (ptrPdStackContext->dpmConfig.curPortRole == app_pref_power_role[port])
                {
                    app_stat_p->app_pending_swaps &= ~APP_PR_SWAP_PENDING;
                    actv_swap = 0;
                }
                break;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

            default:
                actv_swap = 0;
                break;
        }

        if (actv_swap == 0)
        {
            /*
             * Currently selected SWAP is no longer relevant. Re-run function to identify the next swap to be
             * performed.
             */
            if (app_stat_p->app_pending_swaps != 0)
            {
                app_stat_p->actv_swap_type = 0;
                Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                        GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER), APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
            }
        }
        else
        {
            /* Store the swap command for use in the callback. */
            app_stat_p->actv_swap_type = actv_swap;

            /* Only packet type needs to be set when initiating swap operations. */
            pd_cmd_buf.cmdSop = CY_PD_SOP;

            /* Try to trigger the selected swap operation. */
            if (Cy_PdStack_Dpm_SendPdCommand(ptrPdStackContext, (cy_en_pdstack_dpm_pd_cmd_t)actv_swap, &pd_cmd_buf, 
                        false, app_role_swap_resp_cb) != CY_PDSTACK_STAT_SUCCESS)
            {
                /* Retries in case of AMS failure can always be done with a small delay. */
                Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                        GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER), APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_swap);
            }
        }
    }
}

/* This function is called at the end of a PD contract to check whether any role swaps need to be triggered. */
void app_contract_handler (cy_stc_pdstack_context_t *ptrPdStackContext)
{
    app_status_t *app_stat = &app_status[ptrPdStackContext->port];
    //const cy_stc_pdstack_dpm_status_t* dpm = &ptrPdStackContext->dpmStat;

    uint16_t delay_reqd = APP_INITIATE_PR_SWAP_TIMER_PERIOD;

    uint8_t port = ptrPdStackContext->port;

#if (POWER_ROLE_PREFERENCE_ENABLE)
    /* Check if we need to go ahead with PR-SWAP. */
    if (
            (app_pref_power_role[port] == CY_PD_PRT_DUAL) ||
            (ptrPdStackContext->dpmConfig.curPortRole == app_pref_power_role[port])
       )
    {
        app_stat->app_pending_swaps &= ~APP_PR_SWAP_PENDING;
    }
    else
    {
        /* If we are about to swap to become source, ensure VConn Swap is done as required. */
        if ((app_pref_power_role[port] == CY_PD_PRT_ROLE_SOURCE) && (ptrPdStackContext->dpmConfig.vconnLogical == 0))
        {
            app_stat->app_pending_swaps |= APP_VCONN_SWAP_PENDING;
        }
    }
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

    /* Check if we need to go ahead with DR-SWAP. */
    if (
            (app_pref_data_role[port] == CY_PD_PRT_TYPE_DRP) ||
            (ptrPdStackContext->dpmConfig.curPortType == app_pref_data_role[port])
       )
    {
        app_stat->app_pending_swaps &= ~APP_DR_SWAP_PENDING;
    }
    else
    {
        /* DR-SWAPs need to be initiated as soon as possible. VConn swap will be triggered after DR_SWAP as needed. */
        delay_reqd = APP_INITIATE_DR_SWAP_TIMER_PERIOD;
    }

    /* Start a timer that will kick off the Swap state machine. */
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER), delay_reqd, app_initiate_swap);
}

void app_connect_change_handler (cy_stc_pdstack_context_t *ptrPdStackContext)
{
    /* Stop all timers used to trigger swap operations. */
    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER));
    uint8_t port = ptrPdStackContext->port;

#if (POWER_ROLE_PREFERENCE_ENABLE)
    /* Assume that PR_SWAP and DR_SWAP are pending. The actual status will be updated on contract completion. */
    app_status[port].app_pending_swaps = APP_PR_SWAP_PENDING | APP_DR_SWAP_PENDING;
#else
    /* Assume that DR_SWAP is pending. The actual status will be updated on contract completion. */
    app_status[port].app_pending_swaps = APP_DR_SWAP_PENDING;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

    app_status[port].actv_swap_type    = 0;
    app_status[port].actv_swap_count   = 0;
}

#endif /* (ROLE_PREFERENCE_ENABLE) */

static uint8_t gl_app_previous_polarity[NO_OF_TYPEC_PORTS];

#if (CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY))
void epr_enter_mode_timer_cb (
        cy_timer_id_t id,
        void *ptrContext)
{
    (void)id;
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)ptrContext;

    if (Cy_PdStack_Dpm_SendPdCommand (ptrPdStackContext, CY_PDSTACK_DPM_CMD_SNK_EPR_MODE_ENTRY, NULL, false, NULL) != CY_PDSTACK_STAT_SUCCESS)
    {
        /* Retry the EPR Entry. */
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                GET_APP_TIMER_ID(ptrPdStackContext, APP_EPR_MODE_TIMER), APP_EPR_SNK_ENTRY_TIMER_PERIOD, epr_enter_mode_timer_cb);
    }
}
#endif /* CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY) */


#if (!CY_PD_SINK_ONLY)
/* Dummy callback used to ensure VBus discharge happens after debug accessory sink is disconnected. */
static void debug_acc_src_disable_cbk(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    (void)ptrPdStackContext;
}

static void debug_acc_src_psrc_enable_cb(cy_timer_id_t id,  void *context)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)context;
    ptrPdStackContext->ptrAppCbk->psrc_enable(ptrPdStackContext, NULL);
}
#endif /* CY_PD_SINK_ONLY */

#if (CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY))
bool gl_typec_reset_epr[NO_OF_TYPEC_PORTS];
#endif /* CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY) */

void app_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, 
               cy_en_pdstack_app_evt_t evt, const void* dat)
{
    const cy_en_pdstack_app_req_status_t* result;
    const cy_stc_pdstack_pd_contract_info_t* contract_status;
    bool  skip_soln_cb = false;
    bool  hardreset_cplt = false;
    bool  typec_only = false;
    uint8_t port = ptrPdStackContext->port;

    if (port >= NO_OF_TYPEC_PORTS)
    {
        return;
    }
#if 0
    char ret[] = {'E', '0' + evt/10, '0' + (evt%10), '\r', '\n', 0};
    Cy_SCB_UART_PutString(UART_HW, ret);
#endif
    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
            break;

        case APP_EVT_TYPEC_ATTACH:
            Cy_SCB_UART_PutString(UART_HW, "ATTCH\r\n");
            /* Clear all fault counters if we have seen a change in polarity from previous connection. */
            if (ptrPdStackContext->dpmConfig.polarity != gl_app_previous_polarity[port])
            {
                fault_handler_clear_counts (ptrPdStackContext->port);
            }
            gl_app_previous_polarity[port] = ptrPdStackContext->dpmConfig.polarity ;
#if (CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY))
            gl_typec_reset_epr[port] = false;
#endif /* CY_PD_EPR_ENABLE && (!CY_PD__SOURCE_ONLY) */
            break;

        case APP_EVT_CONNECT:
#if (!CY_PD_CBL_DISC_DISABLE)
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

            if (ptrPdStackContext->dpmConfig.attachedDev == CY_PD_DEV_DBG_ACC)
            {
                if(ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SOURCE)
                {
#if (!CY_PD_SINK_ONLY)
                    uint16_t mux_delay = ptrPdStackContext->ptrDpmParams->muxEnableDelayPeriod;
                    if(0U == mux_delay)
                    {
                        mux_delay = 1U;
                    }

                    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext, APP_PSOURCE_EN_TIMER, mux_delay,
                                debug_acc_src_psrc_enable_cb);
#endif /* (!CY_PD_SINK_ONLY) */
                }

                app_status[port].debug_acc_attached = true;
            }

#if (ROLE_PREFERENCE_ENABLE)
            app_connect_change_handler (ptrPdStackContext);
#endif /* (ROLE_PREFERENCE_ENABLE) */
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
            hardreset_cplt = true;
            /* Intentional fall-through. */

        case APP_EVT_HR_SENT_RCVD_DEFERRED:
        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
            typec_only = ((ptrPdStackContext->dpmStat.pdConnected == false) || (evt == APP_EVT_PE_DISABLED));
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_DISCONNECT:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
#if (!CY_PD_CBL_DISC_DISABLE)
            app_status[port].cbl_disc_id_finished = false;
            app_status[port].disc_cbl_pending = false;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

            if (evt == APP_EVT_DISCONNECT)
            {
#if (!CY_PD_SINK_ONLY)
                /* Mark debug accessory detached. */
                {
                    bool debug_acc_attached = app_status[port].debug_acc_attached;
                    app_status[port].debug_acc_attached = false;
                    if((false != debug_acc_attached)&&
                        (ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SOURCE))
                    {
                        ptrPdStackContext->ptrAppCbk->psrc_disable(ptrPdStackContext, debug_acc_src_disable_cbk);
                    }
                }
#endif /* (!CY_PD_SINK_ONLY) */
            }

            if (!hardreset_cplt)
            {
                /*
                 * Isolate the data lines if this is a PD connection.
                 */
                if (!typec_only)
                {
                    Cy_PdUtils_SwTimer_Stop (ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_AME_TIMEOUT_TIMER));
                }
            }
#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE || VBUS_OVP_ENABLE || VBUS_UVP_ENABLE)
            if(evt == APP_EVT_TYPE_C_ERROR_RECOVERY)
            {
                /* Clear port-in-fault flag if all fault counts are within limits. */
                if (!app_port_fault_count_exceeded(ptrPdStackContext))
                {
                    if ((app_get_status(port)->fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                    {
                        ptrPdStackContext->dpmStat.faultActive = false;
                    }
                }
            }
#endif /* VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE */

            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
#if VCONN_OCP_ENABLE
                /* Clear the VConn fault status. */
                app_status[port].fault_status &= ~APP_PORT_VCONN_FAULT_ACTIVE;
#endif /* VCONN_OCP_ENABLE */
            }

#if ((!CY_PD_SINK_ONLY) || (ROLE_PREFERENCE_ENABLE))
            if (
                    (evt == APP_EVT_HARD_RESET_COMPLETE) ||
                    (evt == APP_EVT_TYPE_C_ERROR_RECOVERY) ||
                    (evt == APP_EVT_DISCONNECT)
               )
            {
#if (ROLE_PREFERENCE_ENABLE)
                /* Stop the DR-Swap and PR-Swap trigger timers.  Assume that
                 * PR_SWAP and DR_SWAP are pending. The actual status will be
                 * updated on contract completion.
                 */
                app_connect_change_handler (ptrPdStackContext);
#endif /* (ROLE_PREFERENCE_ENABLE) */
            }
#endif /* ((!CY_PD_SINK_ONLY) || (ROLE_PREFERENCE_ENABLE)) */

            break;

#if (!CY_PD_CBL_DISC_DISABLE)
        case APP_EVT_EMCA_NOT_DETECTED:
        case APP_EVT_EMCA_DETECTED:
            app_status[port].cbl_disc_id_finished = true;
            app_status[port].disc_cbl_pending = false;
            break;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const cy_en_pdstack_app_req_status_t*)dat;
            if(*result == CY_PDSTACK_REQ_ACCEPT)
            {
#if (ROLE_PREFERENCE_ENABLE)
                app_status[port].app_pending_swaps &= ~APP_DR_SWAP_PENDING;
                if (app_status[port].actv_swap_type == CY_PDSTACK_DPM_CMD_SEND_DR_SWAP)
                {
                    Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER));
                    app_contract_handler (ptrPdStackContext);
                }
#endif /* (ROLE_PREFERENCE_ENABLE) */
            }
            break;

        case APP_EVT_VENDOR_RESPONSE_TIMEOUT:
            /* If the APP layer is going to retry the VDM, do not send the event. */
            if (app_status[port].vdm_retry_pending)
                skip_soln_cb = true;
            break;
        case APP_EVT_BAD_SINK_APDO_SEL:
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            Cy_SCB_UART_PutString(UART_HW, "CTRCT\r\n");

            contract_status = (cy_stc_pdstack_pd_contract_info_t*)dat;

            /* Set VDM version based on active PD revision. */
#if CY_PD_REV3_ENABLE
            if (ptrPdStackContext->dpmConfig.specRevSopLive >= CY_PD_REV3)
            {
                app_status[port].vdm_version = CY_PD_STD_VDM_VERSION_REV3;
                app_status[port].vdm_minor_version = CY_PDSTACK_STD_VDM_MINOR_VER1;
            }
            else
#endif /* CCG_PD_REV3_ENABLE */
            {
                app_status[port].vdm_version = CY_PD_STD_VDM_VERSION_REV2;
                app_status[port].vdm_minor_version = CY_PDSTACK_STD_VDM_MINOR_VER0;
            }

            if ((contract_status->status ==CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == CY_PDSTACK_CONTRACT_CAP_MISMATCH_DETECTED))
            {
#if (ROLE_PREFERENCE_ENABLE)
                app_contract_handler (ptrPdStackContext);
#endif /* (ROLE_PREFERENCE_ENABLE) */
            }

#if (CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY))
            if(!gl_typec_reset_epr[port])
            {
                /* Start a timer to attempt EPR entry if the current role is Sink. */
                if ( (ptrPdStackContext->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK) &&
                        (ptrPdStackContext->dpmStat.srcCapP->dat[0].fixed_src.eprModeCapable == true) && 
                            (ptrPdStackContext->dpmExtStat.epr.snkEnable == true))
                {
                    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                            GET_APP_TIMER_ID(ptrPdStackContext, APP_EPR_MODE_TIMER), APP_EPR_SNK_ENTRY_TIMER_PERIOD, epr_enter_mode_timer_cb);

                    gl_typec_reset_epr[port] = true;
                }
            }
#endif /* CY_PD_EPR_ENABLE && (!CY_PD_SOURCE_ONLY) */
            break;

#if CY_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
            {
                if(!(app_extd_msg_handler(ptrPdStackContext, (cy_stc_pd_packet_extd_t *)dat)))
                {
                    skip_soln_cb  = false;
                }
                else
                {
                    skip_soln_cb  = true;
                }
            }
            break;
#endif /* CCG_PD_REV3_ENABLE */

        case APP_EVT_TYPEC_ATTACH_WAIT:
            break;

        case APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED:
            break;

#if (POWER_ROLE_PREFERENCE_ENABLE)
        case APP_EVT_PR_SWAP_COMPLETE:
            app_status[port].app_pending_swaps &= ~APP_PR_SWAP_PENDING;
            if (app_status[port].actv_swap_type == CY_PDSTACK_DPM_CMD_SEND_PR_SWAP)
            {
                Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER));
                app_contract_handler (ptrPdStackContext);
            }
            break;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */

        case APP_EVT_DATA_RESET_ACCEPTED:
#if (!CY_PD_CBL_DISC_DISABLE)
            app_status[port].cbl_disc_id_finished = false;
#endif /* (!CY_PD_CBL_DISC_DISABLE) */

            break;

        case APP_EVT_DATA_RESET_CPLT:
            break;

        case APP_EVT_PD_SINK_DEVICE_CONNECTED:
            break;

        case APP_EVT_PKT_RCVD:
            break;

        case APP_EVT_CBL_RESET_SENT:
            break;

#if (!CY_PD_SINK_ONLY)
        case APP_EVT_HR_PSRC_ENABLE:
            break;
#endif /* (!CY_PD_SINK_ONLY) */

#if CY_PD_EPR_ENABLE
        case APP_EVT_EPR_MODE_ENTER_FAILED:
            break;
        case APP_EVT_EPR_MODE_ENTER_RECEIVED:
#if ROLE_PREFERENCE_ENABLE
            Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, GET_APP_TIMER_ID(ptrPdStackContext, APP_INITIATE_SWAP_TIMER));
#endif /* ROLE_PREFERENCE_ENABLE */
            break;
        case APP_EVT_EPR_MODE_ENTER_SUCCESS:
            Cy_SCB_UART_PutString(UART_HW, "EPRE\r\n");

            break;
#endif /* CY_PD_EPR_ENABLE */

        default:
            /* Nothing to do. */
            break;
    }

    /* Pass the event notification to the fault handler module. */
    if (fault_event_handler (ptrPdStackContext, evt, dat))
    {
        skip_soln_cb = true;
    }

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
       sln_pd_event_handler(ptrPdStackContext, evt, dat) ;
    }
}

app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &app_status[port].appResp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &app_status[port];
}

void app_init(cy_stc_pdstack_context_t *ptrPdStackContext)
{
#if (ROLE_PREFERENCE_ENABLE)
    uint8_t port = ptrPdStackContext->port;
#if (POWER_ROLE_PREFERENCE_ENABLE)
    app_pref_power_role[port] = CY_PD_PRT_ROLE_SOURCE;
#endif /* (POWER_ROLE_PREFERENCE_ENABLE) */
    app_pref_data_role[port] = CY_PD_PRT_TYPE_DFP;
#endif /* (ROLE_PREFERENCE_ENABLE) */

    /* For now, only the VDM handlers require an init call. */

    /* Initialize the VDM responses from the configuration table. */
    vdm_data_init(ptrPdStackContext);

    /* Update custom host config settings to the stack. */
    ptrPdStackContext->dpmStat.typecAccessorySuppDisabled = !(ptrPdStackContext->ptrPortCfg->accessoryEn);
    ptrPdStackContext->dpmStat.typecRpDetachDisabled = !(ptrPdStackContext->ptrPortCfg->rpDetachEn);
}

#if SYS_DEEPSLEEP_ENABLE
/* Implements PMG1 deep sleep functionality for power saving. */
bool system_sleep(cy_stc_pdstack_context_t *ptrPdStackContext, cy_stc_pdstack_context_t *ptrPdStack1Context)
{
    uint32_t intr_state;
    bool dpm_slept = false;
    bool retval = false;
#if PMG1_PD_DUALPORT_ENABLE
    bool dpm_port1_slept = false;
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Do one DPM sleep capability check before locking interrupts out. */
    if (
            (Cy_PdStack_Dpm_IsIdle (ptrPdStackContext, &dpm_slept) != CY_PDSTACK_STAT_SUCCESS) ||
            (!dpm_slept)
       )
    {
        return retval;
    }

#if PMG1_PD_DUALPORT_ENABLE
    if (
            (Cy_PdStack_Dpm_IsIdle (ptrPdStack1Context, &dpm_port1_slept) != CY_PDSTACK_STAT_SUCCESS) ||
            (!dpm_port1_slept)
       )
    {
        return retval;
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */

    intr_state = Cy_SysLib_EnterCriticalSection();

    if (
            ((Cy_PdStack_Dpm_PrepareDeepSleep(ptrPdStackContext, &dpm_slept) == CY_PDSTACK_STAT_SUCCESS) &&
            (dpm_slept))
#if PMG1_PD_DUALPORT_ENABLE
            &&
            ((Cy_PdStack_Dpm_PrepareDeepSleep(ptrPdStack1Context, &dpm_port1_slept) == CY_PDSTACK_STAT_SUCCESS) &&
            (dpm_port1_slept))
#endif /* PMG1_PD_DUALPORT_ENABLE */
           )
        {
            Cy_PdUtils_SwTimer_EnterSleep(ptrPdStackContext->ptrTimerContext);

            Cy_USBPD_SetReference(ptrPdStackContext->ptrUsbPdContext, true);
#if PMG1_PD_DUALPORT_ENABLE
            Cy_USBPD_SetReference(ptrPdStack1Context->ptrUsbPdContext, true);
#endif /* PMG1_PD_DUALPORT_ENABLE */

            /* Device sleep entry. */
            Cy_SysPm_CpuEnterDeepSleep();

            Cy_USBPD_SetReference(ptrPdStackContext->ptrUsbPdContext, false);
#if PMG1_PD_DUALPORT_ENABLE
            Cy_USBPD_SetReference(ptrPdStack1Context->ptrUsbPdContext, false);
#endif /* PMG1_PD_DUALPORT_ENABLE */
            retval = true;
        }

    Cy_SysLib_ExitCriticalSection(intr_state);

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if (dpm_slept)
    {
        Cy_PdStack_Dpm_Resume(ptrPdStackContext, &dpm_slept);
    }

#if PMG1_PD_DUALPORT_ENABLE
    if (dpm_port1_slept)
    {
        Cy_PdStack_Dpm_Resume(ptrPdStack1Context, &dpm_port1_slept);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */

    (void)ptrPdStack1Context;
    return retval;
}
#endif /* SYS_DEEPSLEEP_ENABLE */

#if VCONN_OCP_ENABLE
void app_vconn_ocp_tmr_cbk(cy_timer_id_t id,  void *context)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)context;

    /* Disable VConn since we hit a fault. */
    vconn_disable(ptrPdStackContext, ptrPdStackContext->dpmConfig.revPol);

    /* Notify application layer about fault. */
    app_event_handler(ptrPdStackContext, APP_EVT_VCONN_OCP_FAULT, NULL);
}

bool app_vconn_ocp_cbk(void *context, bool comp_out)
{
    cy_stc_usbpd_context_t * ptrUsbPdContext = (cy_stc_usbpd_context_t *)context;
    cy_stc_pdstack_context_t * ptrPdStackContext = get_pdstack_context(ptrUsbPdContext->port);
    bool retval = false;

    if (comp_out)
    {
        /* Start a VConn OCP debounce timer */
        Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
                CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext, CY_PDSTACK_PD_VCONN_OCP_DEBOUNCE_TIMER),
                ptrPdStackContext->ptrUsbPdContext->usbpdConfig->vconnOcpConfig->debounce,
                app_vconn_ocp_tmr_cbk);
    }
    else
    {
        /* Negative edge. Check if the timer is still running. */
        retval = Cy_PdUtils_SwTimer_IsRunning(ptrPdStackContext->ptrTimerContext, CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext, CY_PDSTACK_PD_VCONN_OCP_DEBOUNCE_TIMER));
        if (retval)
        {
            Cy_PdUtils_SwTimer_Stop(ptrPdStackContext->ptrTimerContext, CY_PDSTACK_GET_PD_TIMER_ID(ptrPdStackContext, CY_PDSTACK_PD_VCONN_OCP_DEBOUNCE_TIMER));
        }
    }

    return retval;
}

#endif /* VCONN_OCP_ENABLE */

void app_vconn_tmr_cbk(cy_timer_id_t id,  void *context)
{
    cy_stc_pdstack_context_t *ptrPdStackContext = (cy_stc_pdstack_context_t *)context;
    Cy_USBPD_Vconn_GatePullUp_Enable(ptrPdStackContext->ptrUsbPdContext);
}

bool vconn_enable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
#if VCONN_OCP_ENABLE

    /* Do not attempt to enable VConn if fault was detected in present connection. */
    if ((app_get_status(ptrPdStackContext->port)->fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0)
    {
        return false;
    }

    Cy_USBPD_Fault_Vconn_OcpEnable (ptrPdStackContext->ptrUsbPdContext, app_vconn_ocp_cbk);
#endif /* VCONN_OCP_ENABLE */

    /* Reset RX Protocol for cable */
    Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_PRIME);
    Cy_PdStack_Dpm_ProtResetRx(ptrPdStackContext, CY_PD_SOP_DPRIME);

    if (Cy_USBPD_Vconn_Enable(ptrPdStackContext->ptrUsbPdContext, channel) != CY_USBPD_STAT_SUCCESS)
    {
#if (defined(CY_DEVICE_PMG1S3))
        app_status[ptrPdStackContext->port].fault_status |= APP_PORT_V5V_SUPPLY_LOST;
#endif /* PMG1 */
        return false;
    }
#if defined(CY_DEVICE_PMG1S3)
    /* Start a VConn OCP debounce timer */
    Cy_PdUtils_SwTimer_Start(ptrPdStackContext->ptrTimerContext, ptrPdStackContext,
            GET_APP_TIMER_ID(ptrPdStackContext, APP_VCONN_TURN_ON_DELAY_TIMER),
            APP_VCONN_TURN_ON_DELAY_PERIOD,
            app_vconn_tmr_cbk);
#endif /* CY_DEVICE_PMG1S3 */
    return true;
}

void vconn_disable(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t channel)
{
    Cy_USBPD_Vconn_Disable(ptrPdStackContext->ptrUsbPdContext, channel);
#if VCONN_OCP_ENABLE
    Cy_USBPD_Fault_Vconn_OcpDisable(ptrPdStackContext->ptrUsbPdContext);
#endif /* VCONN_OCP_ENABLE */
}

bool vconn_is_present(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    return Cy_USBPD_Vconn_IsPresent(ptrPdStackContext->ptrUsbPdContext, ptrPdStackContext->dpmConfig.revPol);
}

bool vbus_is_present(cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t volt, int8_t per)
{
    uint8_t level;
    uint8_t retVal;
    uint16_t v = volt/1000;
    /*
     * Re-run calibration every time to ensure that VDDD or the measurement
     * does not break.
     */
    Cy_USBPD_Adc_Calibrate(ptrPdStackContext->ptrUsbPdContext, APP_VBUS_POLL_ADC_ID);
    level =  Cy_USBPD_Adc_GetVbusLevel(ptrPdStackContext->ptrUsbPdContext,
                           APP_VBUS_POLL_ADC_ID,
                           volt, per);

#if CY_PD_VMON_TYPEC_VBUS
    /* using the port parameter as the ADC input as the input is selected as that of port number.*/
    retVal = Cy_USBPD_Adc_CompSample(ptrPdStackContext->ptrUsbPdContext,
         APP_VBUS_POLL_ADC_ID,
         ptrPdStackContext->port, level);
#else
    retVal = Cy_USBPD_Adc_CompSample(ptrPdStackContext->ptrUsbPdContext,
                     APP_VBUS_POLL_ADC_ID,
                     APP_VBUS_POLL_ADC_INPUT, level);
#endif /* CY_PD_VMON_TYPEC_VBUS */

    char ret[] = {'V', '0' + v/10, '0' + (v%10), ' ', '0' + retVal, '\r', '\n', 0};
    //Cy_SCB_UART_PutString(UART_HW, ret);
    //Cy_SysLib_DelayUs(5000);

    return retVal;
}

void vbus_discharge_on(cy_stc_pdstack_context_t* context)
{
    Cy_USBPD_Vbus_DischargeOn(context->ptrUsbPdContext);
}

void vbus_discharge_off(cy_stc_pdstack_context_t* context)
{
    Cy_USBPD_Vbus_DischargeOff(context->ptrUsbPdContext);
}

uint16_t vbus_get_value(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint16_t retVal;
#if CY_PD_VMON_TYPEC_VBUS
    /* using the port parameter as the ADC input as the input is selected as that of port number.*/
    retVal = Cy_USBPD_Adc_MeasureVbus(ptrPdStackContext->ptrUsbPdContext,
          APP_VBUS_POLL_ADC_ID,
          ptrPdStackContext->port);
#else
    /* Measure the actual VBUS voltage. */
    retVal = Cy_USBPD_Adc_MeasureVbus(ptrPdStackContext->ptrUsbPdContext,
                          APP_VBUS_POLL_ADC_ID,
                          APP_VBUS_POLL_ADC_INPUT);
#endif /* CY_PD_VMON_TYPEC_VBUS */
    char ret[] = {'0' + retVal/100, '0' + (retVal/10), '\r', '\n', 0};
    Cy_SCB_UART_PutString(UART_HW, ret);
    return retVal;
}

#if ((CY_PD_EPR_ENABLE) && (!CY_PD_SINK_ONLY))
bool eval_epr_mode(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_eprmdo_action_t eprModeState, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    (void) ptrPdStackContext;
    (void) eprModeState;
    (void) app_resp_handler;

    /* Place holder for customer specific EPR Source preparation
     * It is possible to provide additional checking before sends EPR Mode Enter Acknowledged  */

    return true;
}

bool send_epr_cap(cy_stc_pdstack_context_t *ptrPdStackContext, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
    (void) ptrPdStackContext;
    (void) app_resp_handler;

    /* Place holder for customer specific EPR Source preparation
     * It is possible to provide additional checking before sends EPR SRC CAP  */

    app_resp_handler(ptrPdStackContext, NULL);

    return true;
}
#endif /* ((CY_PD_EPR_ENABLE) && (!CY_PD_SINK_ONLY)) */

bool send_src_info (struct cy_stc_pdstack_context *ptrPdStackContext)
{
    /* Place holder for customer specific preparation
     * It is possible to provide additional checking before sends source info message */

    return true;
}

/* End of File */
