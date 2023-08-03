/******************************************************************************
* File Name: ncp81239.c
*
* Description: This is source code for the NC81239 power controller.
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "ncp81239.h"

#if NCP_REG_ENABLE
/* I2C master data rate in Hz */
#define I2CM_DATA_RATE_HZ           (100000U)
#define I2CM_TIMER_PERIOD           (10U)

static cy_stc_scb_i2c_context_t  i2c_context;

/* I2C slave address assigned to NCP81239 controller for each PD port. */
static const uint8_t pd_ctrl_addr[NO_OF_TYPEC_PORTS] =
{
    NCP_PORT_1_SLAVE_ADDR
#if PMG1_PD_DUALPORT_ENABLE
        ,
    NCP_PORT_2_SLAVE_ADDR
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

uint8_t pd_ctrl_config[][2] =
{
#if CY_PD_FRS_RX_ENABLE
    {PD_CTRL_VPS_REG_ADDR, PD_CTRL_VPS_5V + NCP_REG_EXCESS_VOLTAGE},
#else /* !CY_PD_FRS_RX_ENABLE */
    {PD_CTRL_VPS_REG_ADDR, PD_CTRL_VPS_0V},
#endif /* CY_PD_FRS_RX_ENABLE */

    {PD_CTRL_SKEW_RATE_REG_ADDR, PD_CTRL_SKEW_RATE_4_9_MV_US}
};

/* Configure the I2C interface. */
void i2cm_init(void)
{
    uint32_t dataRate = 0U;
    Cy_SCB_I2C_Init (I2CM_HW, &I2CM_config, &i2c_context);

    /* Note. This function will have to be changed if the clock divider for the SCB is changed. */
    dataRate = Cy_SCB_I2C_SetDataRate(I2CM_HW, I2CM_DATA_RATE_HZ, Cy_SysClk_PeriphGetFrequency(CY_SYSCLK_DIV_8_BIT, 4U));
    if ((dataRate > I2CM_DATA_RATE_HZ) || (dataRate == 0U))
    {
        return;
    }

    Cy_SCB_I2C_Enable(I2CM_HW, &i2c_context);
}

bool I2C_Write (uint8_t addr, uint8_t *buffer, uint32_t count)
{
    cy_en_scb_i2c_status_t status;
    uint32_t timeout = I2CM_TIMER_PERIOD;

    /* Send Start condition, address and receive ACK/NACK response from slave */
    status = Cy_SCB_I2C_MasterSendStart(I2CM_HW, addr, CY_SCB_I2C_WRITE_XFER, timeout, &i2c_context);
    if (status == CY_SCB_I2C_SUCCESS)
    {
        uint32_t cnt = 0UL;
        /* Write data into the slave from the buffer */
        do
        {
            /* Write byte and receive ACK/NACK response */
            status = Cy_SCB_I2C_MasterWriteByte(I2CM_HW, buffer[cnt], timeout, &i2c_context);
            ++cnt;
        }
        while((status == CY_SCB_I2C_SUCCESS) && (cnt < count));
    }
    /* Check status of transaction */
    if ((status == CY_SCB_I2C_SUCCESS)           ||
            (status == CY_SCB_I2C_MASTER_MANUAL_NAK) ||
            (status == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK))
    {
        /* Send Stop condition on the bus */
        status = Cy_SCB_I2C_MasterSendStop(I2CM_HW, timeout, &i2c_context);
    }

    return (status == CY_SCB_I2C_SUCCESS) ? true : false;
}

bool pd_ctrl_init(void)
{
    uint8_t idx1, idx2, count;
    uint8_t wr_buf[2];
    uint8_t status = true;
    idx1 = TYPEC_PORT_0_IDX;
    /* Configure the I2C interface. */
    i2cm_init ();

    count = sizeof (pd_ctrl_config) / (2 * sizeof (uint8_t));
    for (idx1 = TYPEC_PORT_0_IDX; idx1 < NO_OF_TYPEC_PORTS; idx1++)
    {
#if NCP_GPIO_ENABLE
#if CY_PD_FRS_RX_ENABLE
        /* Leave the regulator enabled as FRS swap may be triggered at any time. */
        if (idx1 == TYPEC_PORT_0_IDX)
        {
            NCP81239_EN_P1();
        }
#if PMG1_PD_DUALPORT_ENABLE
        else
        {
            NCP81239_EN_P2();
        }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_PD_FRS_RX_ENABLE */
#endif /* NCP_GPIO_ENABLE */

        for (idx2 = 0; idx2 < count; idx2++)
        {
            /* Write data in index 1 to address in index 0. */
            wr_buf[0] = pd_ctrl_config[idx2][0];
            wr_buf[1] = pd_ctrl_config[idx2][1];

            status = I2C_Write(pd_ctrl_addr[idx1], wr_buf, 2);
            if (!status)
                return status;
        }
    }
    return status;
}

/* Setting power voltage on NCP81239 controller */
bool set_pd_ctrl_voltage(uint8_t port, uint16_t volt)
{
    uint8_t wr_buf[2];
    uint8_t status;

#if CY_PD_FRS_RX_ENABLE
    /* If fast role swap is enabled, we need to have a 5V supply ready at all times. */
    if (volt < CY_PD_VSAFE_5V)
    {
        volt = CY_PD_VSAFE_5V;
    }
#endif /* CY_PD_FRS_RX_ENABLE */

    /* Configure the regulator output voltage. */
    wr_buf[0] = PD_CTRL_VPS_REG_ADDR;
    wr_buf[1] = (volt / NCP_REG_VOLT_RESOLUTION) + NCP_REG_EXCESS_VOLTAGE;

    status = I2C_Write(pd_ctrl_addr[port], wr_buf, 2);
    return (status);
}
#endif /* !MPS_28V_REG_ENABLE */

/* [] END OF FILE */
