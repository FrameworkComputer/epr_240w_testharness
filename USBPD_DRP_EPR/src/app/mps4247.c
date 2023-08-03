/******************************************************************************
* File Name: mps4247.c
*
* Description: This is source code for the MPS EPR power regulator.
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

#if MPS_28V_REG_ENABLE
/* I2C master data rate in Hz */
#define I2CM_DATA_RATE_HZ           (100000U)
#define I2CM_TIMER_PERIOD           (10U)

#define MPS_DEFAULT_ADDR                 (0x67u)
#define MPS_PORT_0_SLAVE_ADDR            (0x67u)
#define MPS_PORT_1_SLAVE_ADDR            (0x68u)
#define MPS_PORT_1_ADDR_CONFIG           (0x08u)
#define MPS_MFR_CTRL4_BIT_7_5_DEFAULT    (0x20u)

#define MPS_VOLT_REG_ADDR                (0x21u)
#define MPS_MFR_CTRL4_REG_ADDR           (0xD4u)

#define MPS_EXCESS_VOLTAGE               (100u)

static cy_stc_scb_i2c_context_t  i2c_context;

/* I2C slave address assigned to NCP81239 controller for each PD port. */
static const uint8_t pd_ctrl_addr[NO_OF_TYPEC_PORTS] =
{
    MPS_PORT_0_SLAVE_ADDR
#if PMG1_PD_DUALPORT_ENABLE
        ,
    MPS_PORT_1_SLAVE_ADDR
#endif /* PMG1_PD_DUALPORT_ENABLE */
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

bool set_port1_mps_addr(uint8_t mps0Addr)
{
    bool status;
    uint8_t wr_buf[2];
    /* Disable port-0 MPS */
    Cy_GPIO_Clr(MPS_P0_EN_PORT, MPS_P0_EN_NUM);
    Cy_SysLib_Delay(100);
    
    wr_buf[0] = MPS_MFR_CTRL4_REG_ADDR;
    wr_buf[1] = (MPS_MFR_CTRL4_BIT_7_5_DEFAULT | mps0Addr);
    status = I2C_Write(MPS_DEFAULT_ADDR, wr_buf, 2);
    
    /* Enable port-0 MPS */
    Cy_GPIO_Set(MPS_P0_EN_PORT, MPS_P0_EN_NUM);

    return status;
}

bool pd_ctrl_init(void)
{
    bool status;

    /* Configure the I2C interface. */
    i2cm_init ();
    /* Change the port-1 MPS I2C address */
    status = set_port1_mps_addr(MPS_PORT_1_ADDR_CONFIG);
    
    return status;
}

/* Setting power voltage on MPS controller */
bool set_pd_ctrl_voltage(uint8_t port, uint16_t volt)
{
    uint8_t wr_buf[3];
    uint8_t status;
    volt = volt + MPS_EXCESS_VOLTAGE;

#if CY_PD_FRS_RX_ENABLE
    /* If fast role swap is enabled, we need to have a 5V supply read at all times. */
    if (volt < CY_PD_VSAFE_5V)
    {
        volt = CY_PD_VSAFE_5V;
    }
#endif /* CY_PD_FRS_RX_ENABLE */

    /* Configure the regulator output voltage. */
    wr_buf[0] = MPS_VOLT_REG_ADDR;
    wr_buf[1] = (volt * 1024) / 1500;
    wr_buf[2] = ((volt * 1024) / 1500) >> 8;

    status = I2C_Write(pd_ctrl_addr[port], wr_buf, 3);
    return (status);
}
#endif /* MPS_28V_REG_ENABLE */

/* [] END OF FILE */