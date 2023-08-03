/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PMG1 MCU: USBPD DRP EPR Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"

#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#include "psource.h"
#include "swap.h"
#include "vdm.h"
#include "mtbcfg_ezpd.h"
#include "cycfg_pins.h"

cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t   gl_UsbPdPort0Ctx;

cy_stc_pdstack_context_t gl_PdStackPort0Ctx;
#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
        .dpmSnkWaitCapPeriod = 400,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_pdstack_dpm_params_t pdstack_port1_dpm_params =
{
        .dpmSnkWaitCapPeriod = 400,
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 0,
        .defCur = 90
};
#endif /* PMG1_PD_DUALPORT_ENABLE */
cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
        &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
        &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};


const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

/* Solution PD event handler */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    /* Nothing to do at present. */
    (void)ctx;
    (void)evt;
    (void)data;
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler(&(gl_TimerCtx));
}

static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

#if PMG1_PD_DUALPORT_ENABLE
static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);
}

static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
cy_stc_pd_dpm_config_t* get_dpm_port1_connect_stat()
{
    return &(gl_PdStackPort1Ctx.dpmConfig);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    app_event_handler,
#if (!CY_PD_SINK_ONLY)
    psrc_set_voltage,
    psrc_set_current,
    psrc_enable,
    psrc_disable,
#endif  /* (!CY_PD_SINK_ONLY) */
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
#if (!(CY_PD_SOURCE_ONLY))
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
#endif /* (!(CY_PD_SOURCE_ONLY)) */
#if (!CY_PD_SINK_ONLY)
    eval_rdo,
#endif  /* (!CY_PD_SINK_ONLY) */
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
#if CY_PD_REV3_ENABLE
#if ((!(CY_PD_SOURCE_ONLY)) && (!CY_PD_SINK_ONLY))
    eval_fr_swap,
#endif /* ((!(CY_PD_SOURCE_ONLY)) && (!CY_PD_SINK_ONLY))  */
#endif /* CY_PD_REV3_ENABLE */
    vbus_get_value,
#if (!CY_PD_SINK_ONLY)
    psrc_get_voltage,
#endif  /* (!CY_PD_SINK_ONLY) */
#if CY_PD_USB4_SUPPORT_ENABLE
   NULL,
#endif
#if ((CY_PD_EPR_ENABLE) && (!CY_PD_SINK_ONLY))
    eval_epr_mode,
    send_epr_cap,
#endif /* (CY_PD_EPR_ENABLE) && (!CY_PD_SINK_ONLY) */
#if (!CY_PD_SINK_ONLY)
    send_src_info
#endif /* (!CY_PD_SINK_ONLY) */
};

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

#if CY_PD_VMON_TYPEC_VBUS
void Cy_ExtVbusMonInit(cy_stc_usbpd_context_t *ptrUsbPdContext)
{
#if PMG1_PD_DUALPORT_ENABLE
    PPDSS_REGS_T pd = ptrUsbPdContext->base;
#endif /* PMG1_PD_DUALPORT_ENABLE */
    /* Connect VBUS monitor GPIO to AMUX_B. */
#if PMG1_PD_DUALPORT_ENABLE
    if (0u == ptrUsbPdContext->port)
#endif /* PMG1_PD_DUALPORT_ENABLE */
    {
        Cy_GPIO_SetHSIOM(VMON_TYPEC_VBUS_P0_PORT, VMON_TYPEC_VBUS_P0_PIN, HSIOM_SEL_AMUXA);
        /* Close left and right switches for AMUX splitter 0 & 1 to connect the VMON GPIO to PD-ADC. */
        HSIOM_AMUX_SPLIT_CTL(AMUX_SPLIT_CTL_0) = (CY_GPIO_AMUX_LR << HSIOM_AMUX_SPLIT_CTL_SWITCH_AA_SL_Pos);
        HSIOM_AMUX_SPLIT_CTL(AMUX_SPLIT_CTL_1) = (CY_GPIO_AMUX_LR << HSIOM_AMUX_SPLIT_CTL_SWITCH_AA_SL_Pos);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_SetHSIOM(VMON_TYPEC_VBUS_P1_PORT, VMON_TYPEC_VBUS_P1_PIN, HSIOM_SEL_AMUXB);
        /* Close left and right switches for AMUX splitter 0 & 1 to connect the VMON GPIO to PD-ADC. */
        HSIOM_AMUX_SPLIT_CTL(AMUX_SPLIT_CTL_0) |= (CY_GPIO_AMUX_LR << HSIOM_AMUX_SPLIT_CTL_SWITCH_BB_SL_Pos);
        HSIOM_AMUX_SPLIT_CTL(AMUX_SPLIT_CTL_1) |= (CY_GPIO_AMUX_LR << HSIOM_AMUX_SPLIT_CTL_SWITCH_BB_SL_Pos);

        /* Clear amux_nhv[3] to use amux_b instead of vbus_c connection. */
        pd->amux_nhv_ctrl &= ((uint32_t)~(1U << 3u));
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */

//#if CY_PD_EPR_36V_SUPP_EN
    /* As the external divider is 6%, for 36V EPR contracts,
     * voltage on the VMON pin will be 2.16V which is crossing the vref voltage.
     * Hence, change the ADC reference voltage to VDDD. */
    (void)Cy_USBPD_Adc_SelectVref(ptrUsbPdContext, APP_VBUS_POLL_ADC_ID, CY_USBPD_ADC_VREF_VDDD);
//#endif /* CY_PD_EPR_36V_SUPP_EN */
}
#endif /* CY_PD_VMON_TYPEC_VBUS */

int main(void)
{
    cy_rslt_t result;
    cy_stc_pdutils_timer_config_t timerConfig;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
	//Cy_GPIO_Clr (PFET_SRC_CTRL_P0_PORT, PFET_SRC_CTRL_P0_PIN);

    sln_pd_event_handler(&gl_PdStackPort0Ctx, APP_EVT_POWER_CYCLE, NULL);
    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    /* Enable global interrupts */
    __enable_irq();

    /* Enable NCP81239 controller */
    pd_ctrl_init ();

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port #1. */
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_CCG3)
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);

    /* Make sure SCP and RCP blocks have been disabled. */
    Cy_USBPD_Fault_Vbus_ScpDisable(&gl_UsbPdPort0Ctx);
    Cy_USBPD_Fault_Vbus_RcpDisable(&gl_UsbPdPort0Ctx);

#if PMG1_PD_DUALPORT_ENABLE
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_port1_connect_stat);
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif

#if CY_PD_VMON_TYPEC_VBUS
    Cy_ExtVbusMonInit(&gl_UsbPdPort0Ctx);
#endif /* CY_PD_VMON_TYPEC_VBUS */

    /* Workaround for pd_ctrl_init () bug. Set to 5 V. */
    APP_VBUS_SET_VOLT_P1(5000);
#if PMG1_PD_DUALPORT_ENABLE
    APP_VBUS_SET_VOLT_P2(5000);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

    /* Send NOT_SUPPORTED for DATA RESET message. */
    Cy_PdStack_Dpm_SetDataReset (&gl_PdStackPort0Ctx, false);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
                       &gl_UsbPdPort1Ctx,
                       &mtb_usbpd_port1_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort1Ctx),
                       &pdstack_port1_dpm_params,
                       &gl_TimerCtx);
    /* Send NOT_SUPPORTED for DATA RESET message. */
    Cy_PdStack_Dpm_SetDataReset (&gl_PdStackPort1Ctx, false);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Perform application level initialization. */
    app_init(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    app_init(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    fault_handler_init_vars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     * Since this application does not have any other function, the PMG1 device can be placed in "deep sleep"
     * mode for power saving whenever the PD stack and drivers are idle.
     */
#if !CY_PD_SINK_ONLY
    gl_PdStackPort0Ctx.dpmStat.srcCapStartDelay = DELAY_SRC_CAP_START_MS;
#endif /* !CY_PD_SINK_ONLY */

    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_PdStack_Dpm_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform any application level tasks. */
        app_task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        app_task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform tasks associated with instrumentation. */
        instrumentation_task();
        

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        system_sleep(&gl_PdStackPort0Ctx, 
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx
#else
                NULL
#endif /* PMG1_PD_DUALPORT_ENABLE */
                );
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
