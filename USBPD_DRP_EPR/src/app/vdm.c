/******************************************************************************
* File Name: vdm.c
*
* Description: This source file implements handlers for Vendor Defined Messages (VDMs)
*              as part of the PMG1 MCU USB-PD DRP Code Example for ModusToolBox.
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

#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "vdm.h"
#include "cy_pdstack_common.h"
#include "cy_pdutils.h"
#include "app.h"

/* Stores Discover ID response VDO count */
static uint8_t  gl_vdm_id_vdo_cnt[NO_OF_TYPEC_PORTS];

/* Stores Discover SVID response VDO count */
static uint8_t  gl_vdm_svid_vdo_cnt[NO_OF_TYPEC_PORTS];

/* Stores Discover Modes response VDO count */
static uint16_t gl_vdm_mode_data_len[NO_OF_TYPEC_PORTS];

/* Stores the actual Discover ID response data */
static cy_pd_pd_do_t gl_vdm_id_vdo_resp[NO_OF_TYPEC_PORTS][CY_PD_MAX_NO_OF_DO];

/* Stores pointer to Discover ID response data */
static cy_pd_pd_do_t *gl_vdm_id_vdo_p[NO_OF_TYPEC_PORTS];

/* Stores pointer to Discover SVID response data */
static cy_pd_pd_do_t *gl_vdm_svid_vdo_p[NO_OF_TYPEC_PORTS];

/* Stores pointer to Discover Modes response data */
static uint8_t *gl_vdm_mode_data_p[NO_OF_TYPEC_PORTS];

const vdm_info_config_t vdm_info[NO_OF_TYPEC_PORTS] =
{
    {
        .discId = {0xFF00A041, 0x19C004B4, 0x00000000, 0xF5040000, 0x40000000},
        .dIdLength = 0x18,
        .sVidLength = 0,
        .disModeLength = 0,
    },
#if PMG1_PD_DUALPORT_ENABLE
    {
        .discId = {0xFF00A041, 0x19C004B4, 0x00000000, 0xF5040000, 0x40000001},
        .dIdLength = 0x18,
        .sVidLength = 0,
        .disModeLength = 0,
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

/* Store VDM information from the config table in the RAM variables. */
void vdm_data_init(cy_stc_pdstack_context_t * context)
{
    uint16_t size = 0;
    uint8_t port = context->port;

    /* Calculate the number of VDOs in the D_ID response. */
    size = vdm_info[port].dIdLength;
    /* Subtract 4 bytes for the header and reduce to number of VDOs. */
    if (size > 4)
        gl_vdm_id_vdo_cnt[port] = (uint8_t)((size - 4) >> 2);
    else
        gl_vdm_id_vdo_cnt[port] = 0;

    /* Copy the actual Discover Identity response. */
    memcpy ((uint8_t *)gl_vdm_id_vdo_resp[port], (const uint8_t *)vdm_info[port].discId,
            gl_vdm_id_vdo_cnt[port] * 4u);

    /* Update the vendor and product IDs from the configuration data. */
    gl_vdm_id_vdo_resp[port][1].std_id_hdr.usbVid = context->ptrPortCfg->mfgVid;
    gl_vdm_id_vdo_resp[port][3].std_prod_vdo.usbPid = context->ptrPortCfg->mfgPid;

    /* Update the D_ID response pointer. */
    gl_vdm_id_vdo_p[port] = (cy_pd_pd_do_t *)(gl_vdm_id_vdo_resp[port]);

    /* Calculate the number of VDOs in the D_SVID response. */
    size = vdm_info[port].sVidLength;
    /* Subtract 4 bytes for the header and reduce to number of VDOs. */
    if (size > 4)
    {
        gl_vdm_svid_vdo_cnt[port] = (uint8_t)((size - 4) >> 2);

        /* Update the D_SVID response pointer. */
        gl_vdm_svid_vdo_p[port] = (cy_pd_pd_do_t *)((uint8_t *)(vdm_info[port].sVid));


        /* Store the D_MODE response length from configuration table. */
        gl_vdm_mode_data_len[port] = vdm_info[port].disModeLength;

        /* Store pointer to the D_MODE responses. */
        gl_vdm_mode_data_p[port] = (uint8_t *)(vdm_info[port].discMode);
    }
    else
    {
        gl_vdm_svid_vdo_cnt[port] = 0;
        gl_vdm_mode_data_len[port] = 0;
    }
}

void vdm_update_svid_resp(cy_stc_pdstack_context_t * context, uint8_t svid_vdo_cnt, uint8_t *svid_vdo_p)
{
    gl_vdm_svid_vdo_cnt[context->port]  = svid_vdo_cnt;
    gl_vdm_svid_vdo_p[context->port]    = (cy_pd_pd_do_t *)svid_vdo_p;
}

void vdm_update_data(cy_stc_pdstack_context_t * context, uint8_t id_vdo_cnt, uint8_t *id_vdo_p,
        uint8_t svid_vdo_cnt, uint8_t *svid_vdo_p, uint16_t mode_resp_len, uint8_t *mode_resp_p)
{
    uint8_t port = context->port;

    if (port < NO_OF_TYPEC_PORTS)
    {
        gl_vdm_id_vdo_cnt[port]    = id_vdo_cnt;
        gl_vdm_id_vdo_p[port]      = (cy_pd_pd_do_t *)id_vdo_p;
        gl_vdm_svid_vdo_cnt[port]  = svid_vdo_cnt;
        gl_vdm_svid_vdo_p[port]    = (cy_pd_pd_do_t *)svid_vdo_p;
        gl_vdm_mode_data_len[port] = mode_resp_len;
        gl_vdm_mode_data_p[port]   = mode_resp_p;
    }
}

bool get_modes_vdo_info(cy_stc_pdstack_context_t * context, uint16_t svid, cy_pd_pd_do_t **temp_p, uint8_t *no_of_vdo)
{
    uint8_t d_mode_resp_size_total;
    uint8_t d_mode_resp_size;
    cy_pd_pd_do_t *header;
    uint8_t* resp_p = gl_vdm_mode_data_p[context->port];

    /* Parse all responses based on SVID */
    d_mode_resp_size_total = gl_vdm_mode_data_len[context->port];

    /* If size is less than or equal to 4, return NACK. */
    if (d_mode_resp_size_total <= 4)
    {
        return false;
    }

    while (d_mode_resp_size_total)
    {
        /* Get the size of packet. */
        d_mode_resp_size = *resp_p;

        /* Read the SVID from header. */
        header = (cy_pd_pd_do_t *)(resp_p + 4);
        if (header->std_vdm_hdr.svid == svid)
        {
            *no_of_vdo = ((d_mode_resp_size-4) >> 2);
            *temp_p = header;
            return true;
        }
        /* Move to next packet. */
        resp_p += d_mode_resp_size;
        d_mode_resp_size_total -= d_mode_resp_size;
    }
    return false;
}

void eval_vdm(cy_stc_pdstack_context_t * context, const cy_stc_pdstack_pd_packet_t *vdm, cy_pdstack_vdm_resp_cbk_t vdm_resp_handler)
{
    app_status_t* app_stat = app_get_status(context->port);
    uint8_t port = context->port;

    cy_pd_pd_do_t* dobj = NULL;
    uint8_t i, count = 0u;
    bool pd3_live = false;

    bool eval_rslt;

#if CY_PD_REV3_ENABLE
    if (context->dpmConfig.specRevSopLive >= CY_PD_REV3)
    {
        pd3_live = true;
    }
#endif /* CY_PD_REV3_ENABLE */

    /* By Default assume we will respond */
    app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_READY;

    if (
            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.vdmType == CY_PDSTACK_VDM_TYPE_STRUCTURED) &&
            (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType == CY_PDSTACK_CMD_TYPE_INITIATOR)
       )
    {
        /* Copy received VDM Header data to VDM response Header*/
        app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].val = vdm->dat[CY_PD_VDM_HEADER_IDX].val;

#if CY_PD_REV3_ENABLE
        /* Use the minimum VDM version from among the partner's revision and the live revision. */
        app_stat->vdm_version = CY_PDUTILS_GET_MIN (app_stat->vdm_version, vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer);
        app_stat->vdm_minor_version = CY_PDUTILS_GET_MIN (app_stat->vdm_minor_version, vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stMinVer);
#endif /* CY_PD_REV3_ENABLE */

        /* Set a NAK response by default. */
        app_stat->vdmResp.doCount = 1 ;
        app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;

        if ((context->dpmConfig.curPortType == CY_PD_PRT_TYPE_UFP) || (pd3_live))
        {
            /* VDM Commands (D_ID -- EXIT_MODE) should be NAKd if VDO Count in VDM
             * command is more than one. */
            if (vdm->len == 1)
            {
                switch(vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd)
                {
                    case CY_PDSTACK_VDM_CMD_DSC_IDENTITY:
                        count = gl_vdm_id_vdo_cnt[port];
                        if((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid ==  STD_SVID) && (count != 0))
                        {
                            app_stat->vdmResp.doCount = count;
                            dobj = gl_vdm_id_vdo_p[port];
                            for(i = 0 ; i < count; i++)
                            {
                                app_stat->vdmResp.respBuf[i] = dobj[i];
                            }

#if CY_PD_REV3_ENABLE
                            /* Mask Product Type (DFP) and Connector Type fields when VDM version is 1.0. */
                            if (app_stat->vdm_version == 0)
                            {
                                cy_pd_pd_do_t id_hdr = app_stat->vdmResp.respBuf[VDO_START_IDX];
                                uint8_t max_do_cnt = 4;

                                /* Make sure to clear fields that are reserved under PD 2.0. */
                                id_hdr.val &= 0xFC1FFFFF;

                                /* Make sure to not use invalid product types. */
                                if (id_hdr.std_id_hdr.prodType == CY_PDSTACK_PROD_TYPE_PSD)
                                {
                                    id_hdr.std_id_hdr.prodType = CY_PDSTACK_PROD_TYPE_UNDEF;
                                }

                                /* AMAs may be providing one extra VDO. */
                                if (id_hdr.std_id_hdr.prodType == CY_PDSTACK_PROD_TYPE_AMA)
                                {
                                    max_do_cnt++;
                                }

                                /* Ensure that the size of the response is limited to what is valid based
                                 * on the ID header. */
                                if (app_stat->vdmResp.doCount > max_do_cnt)
                                    app_stat->vdmResp.doCount = max_do_cnt;
                                app_stat->vdmResp.respBuf[VDO_START_IDX] = id_hdr;
                            }
#else /* !CY_PD_REV3_ENABLE */
                            app_stat->vdmResp.respBuf[VDO_START_IDX].std_id_hdr.rsvd1 = 0;
#endif /* CY_PD_REV3_ENABLE */

                            /* Set VDM Response ACKed */
                            app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;
                        }
                        break;

                    case CY_PDSTACK_VDM_CMD_DSC_SVIDS:
                        count = gl_vdm_svid_vdo_cnt[port];
                        if((vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid == STD_SVID) && (count != 0))
                        {
                            app_stat->vdmResp.doCount = count;
                            dobj = gl_vdm_svid_vdo_p[port];
                            for(i = 0 ; i < count; i++)
                            {
                                app_stat->vdmResp.respBuf[i] = dobj[i];
                            }
                            /* Set VDM Response ACKed */
                            app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;
                        }
                        break;

                    case CY_PDSTACK_VDM_CMD_DSC_MODES:
                        eval_rslt = get_modes_vdo_info (context, vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid,
                            &dobj, &count);
                        if (eval_rslt == true)
                        {
                            app_stat->vdmResp.doCount = count;
                            for (i = 0; i < count; i++)
                            {
                                app_stat->vdmResp.respBuf[i] = dobj[i];
                            }

                            /* Set VDM Response ACKed */
                            app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_ACK;
                        }
                        break;

                    case CY_PDSTACK_VDM_CMD_ENTER_MODE:
                        break;

                    case CY_PDSTACK_VDM_CMD_EXIT_MODE:
                        break;

                    case CY_PDSTACK_VDM_CMD_ATTENTION:
                        /* Ignore Attention VDM */
                        app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
                        break;

                    default:
                        break;
                }
            }
            if(pd3_live)
            {
                if(vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION)
                {
                    /* Ignore Attention VDM */
                    app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
                }
            }
        }
        else
        {
            if (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmd == CY_PDSTACK_VDM_CMD_ATTENTION)
            {
                /* Ignore Attention VDM */
                app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_REQ;
            }
            else
            {
                if (vdm->dat[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.svid == STD_SVID)
                {
                    /* Respond with a NAK to Standard VDMs received on PD 2.0 connection while in DFP role. */
                    app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.cmdType = CY_PDSTACK_CMD_TYPE_RESP_NAK;
                }
            }
        }

        /* Set the VDM version for the response. */
        app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stVer = app_stat->vdm_version;
        app_stat->vdmResp.respBuf[CY_PD_VDM_HEADER_IDX].std_vdm_hdr.stMinVer = app_stat->vdm_minor_version;
    }
    else
    {
        if (
                (!pd3_live)
           )
        {
            /* If UVDM not successed then ignore this UVDM */
            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_FROM_EC;
        }
        else
        {
            /* PD 3.0 Contract: Respond with Not_Supported. */
            app_stat->vdmResp.noResp = CY_PDSTACK_VDM_AMS_RESP_NOT_SUPP;
        }
    }

    vdm_resp_handler(context, &app_stat->vdmResp);
}

/* End of File */
