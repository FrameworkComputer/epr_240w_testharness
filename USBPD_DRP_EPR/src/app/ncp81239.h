/******************************************************************************
* File Name: ncp81239.h
*
* Description: This file defines data structures and function prototypes
*              for the NC81239 power controller.
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

#ifndef _NCP81239_H_
#define _NCP81239_H_
    
#include <stdbool.h>
#include <stdint.h>

/* For NCP81239 Power Delivery Controller. */

/* PD controller I2C address */
#define PD_CTRL_SLAVE_ADDR              (0x74)

#define PD_CTRL_EN_ADDR             (0x00u)
#define PD_CTRL_EN_POL_POS                 (0u)
#define PD_CTRL_EN_POL                     (0u<<PD_CTRL_EN_POL_POS)        /*default*/
#define PD_CTRL_EN_PUP_POS                 (1u)
#define PD_CTRL_EN_PUP                     (0u<<PD_CTRL_EN_PUP_POS)        /*default*/
#define PD_CTRL_EN_MASK_POS             (2u)
#define PD_CTRL_EN_MASK                 (0u<<PD_CTRL_EN_MASK_POS)        /*default*/
#define PD_CTRL_EN_INT_POS                 (3u)
#define PD_CTRL_EN_INT                     (0u<<PD_CTRL_EN_INT_POS)        /*default*/
#define PD_CTRL_EN_REACTION_POS         (4u)
#define PD_CTRL_EN_REACTION             (0u<<PD_CTRL_EN_REACTION_POS)    /*default*/
    
#define PD_CTRL_VPS_REG_ADDR        (0x01u)
#define PD_CTRL_VPS_0V                  (0x00u)                            /*default*/
#define PD_CTRL_VPS_5V                  (0x32u)                         /* 5V */
#define PD_CTRL_VPS_20V                 (0xC8u)                         /* 20V */

#define PD_CTRL_SKEW_RATE_REG_ADDR  (0x02u)
#define PD_CTRL_SKEW_RATE_0_61_MV_US    (0x00u)                            /*default*/
#define PD_CTRL_SKEW_RATE_1_2_MV_US     (0x01u)
#define PD_CTRL_SKEW_RATE_2_4_MV_US     (0x02u)
#define PD_CTRL_SKEW_RATE_4_9_MV_US     (0x03u)

#define PD_CTRL_PWM_FREQ_ADDR         (0x03u)
#define PD_CTRL_PWM_FREQ_600KHZ         (0x00u)                            /*default*/
#define PD_CTRL_PWM_FREQ_150KHZ         (0x01u)
#define PD_CTRL_PWM_FREQ_300KHZ         (0x02u)
#define PD_CTRL_PWM_FREQ_450KHZ         (0x03u)
#define PD_CTRL_PWM_FREQ_750KHZ         (0x04u)
#define PD_CTRL_PWM_FREQ_900KHZ         (0x05u)
#define PD_CTRL_PWM_FREQ_1200KHZ         (0x06u)
#define PD_CTRL_PWM_FREQ_RESERVED         (0x07u)
#define PD_CTRL_PWM_FREQ_VPS_STEP_POS   (4u)
#define PD_CTRL_PWM_FREQ_VPS_STEP_10MV     (0<<PD_CTRL_PWM_FREQ_VPS_STEP_POS)    /*default*/
#define PD_CTRL_PWM_FREQ_VPS_STEP_5MV     (1<<PD_CTRL_PWM_FREQ_VPS_STEP_POS)

#define PD_CTRL_FET_ADDR             (0x04u)
#define PD_CTRL_FET_PFET_POS             (0u)
#define PD_CTRL_FET_PFET_DIS             (0u<<PD_CTRL_FET_PFET_POS)            /*default*/
#define PD_CTRL_FET_PFET_EN             (1u<<PD_CTRL_FET_PFET_POS)
#define PD_CTRL_FET_CFET_POS             (1u)
#define PD_CTRL_FET_CFET_DIS             (0u<<PD_CTRL_FET_CFET_POS)            /*default*/
#define PD_CTRL_FET_CFET_EN             (1u<<PD_CTRL_FET_CFET_POS)
#define PD_CTRL_FET_DB_POS                 (2u)
#define PD_CTRL_FET_DB_DIS                 (0u<<PD_CTRL_FET_DB_POS)            /*default*/
#define PD_CTRL_FET_DB_EN                 (1u<<PD_CTRL_FET_DB_POS)
#define PD_CTRL_FET_CS1_DCHRG_POS         (4u)
#define PD_CTRL_FET_CS1_DCHRG_DIS         (0u<<PD_CTRL_FET_CS1_DCHRG_POS)        /*default*/
#define PD_CTRL_FET_CS1_DCHRG_EN         (1u<<PD_CTRL_FET_CS1_DCHRG_POS)
#define PD_CTRL_FET_CS2_DCHRG_POS         (5u)
#define PD_CTRL_FET_CS2_DCHRG_DIS         (0u<<PD_CTRL_FET_CS2_DCHRG_POS)        /*default*/
#define PD_CTRL_FET_CS2_DCHRG_EN         (1u<<PD_CTRL_FET_CS2_DCHRG_POS)

#define PD_CTRL_OCP_CLIM_ADDR         (0x05u)
#define PD_CTRL_OCP_CLIMP_POS             (0u)
#define PD_CTRL_OCP_CLIMP_7_6A             (0u<<PD_CTRL_OCP_CLIMP_POS)            /*default*/
#define PD_CTRL_OCP_CLIMP_4_6A             (1u<<PD_CTRL_OCP_CLIMP_POS)
#define PD_CTRL_OCP_CLIMP_2_2A             (2u<<PD_CTRL_OCP_CLIMP_POS)
#define PD_CTRL_OCP_CLIMP_10A             (3u<<PD_CTRL_OCP_CLIMP_POS)
#define PD_CTRL_OCP_CLIMN_POS             (4u)
#define PD_CTRL_OCP_CLIMN_MINUS_8A         (0u<<PD_CTRL_OCP_CLIMN_POS)             /*default*/
#define PD_CTRL_OCP_CLIMN_MINUS_5A         (1u<<PD_CTRL_OCP_CLIMN_POS)
#define PD_CTRL_OCP_CLIMN_MINUS_3A         (2u<<PD_CTRL_OCP_CLIMN_POS)
#define PD_CTRL_OCP_CLIMN_0A             (3u<<PD_CTRL_OCP_CLIMN_POS)

#define PD_CTRL_CS_CLIM_ADDR         (0x06u)
#define PD_CTRL_CS1_CLIM_POS             (0u)
#define PD_CTRL_CS1_CLIM_0_5A             (0u<<PD_CTRL_CS1_CLIM_POS)             /*default*/
#define PD_CTRL_CS1_CLIM_1_5A             (1u<<PD_CTRL_CS1_CLIM_POS)
#define PD_CTRL_CS1_CLIM_3A             (2u<<PD_CTRL_CS1_CLIM_POS)
#define PD_CTRL_CS1_CLIM_5A             (3u<<PD_CTRL_CS1_CLIM_POS)
#define PD_CTRL_CS2_CLIM_POS             (2u)
#define PD_CTRL_CS2_CLIM_0_5A             (0u<<PD_CTRL_CS2_CLIM_POS)             /*default*/
#define PD_CTRL_CS2_CLIM_1_5A             (1u<<PD_CTRL_CS2_CLIM_POS)
#define PD_CTRL_CS2_CLIM_3A             (2u<<PD_CTRL_CS2_CLIM_POS)
#define PD_CTRL_CS2_CLIM_5A             (3u<<PD_CTRL_CS2_CLIM_POS)
#define PD_CTRL_CS_CLIM_EN_POS             (4u)
#define PD_CTRL_CS_CLIM_EN                 (0u<<PD_CTRL_CS_CLIM_EN_POS)         /*default */

#define PD_CTRL_GM_ADDR             (0x07u)
#define PD_CTRL_GM_AMPLO_POS             (0u)
#define PD_CTRL_GM_AMPLO_MASK             (0x07u)
#define PD_CTRL_GM_AMPLO_87US             (0u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_100US             (1u<<PD_CTRL_GM_AMPLO_POS)           /*default*/
#define PD_CTRL_GM_AMPLO_117US             (2u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_333US             (3u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_400US             (4u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_500US             (5u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_667US             (6u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_1000US         (7u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_DIS             (0<<3u)
#define PD_CTRL_GM_AMPLO_EN             (1<<3u)                             /*default*/
#define PD_CTRL_GM_AMPHI_POS             (0u)
#define PD_CTRL_GM_AMPHI_MASK             (0x07u)
#define PD_CTRL_GM_AMPHI_87US             (0u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_100US             (1u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_117US             (2u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_333US             (3u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_400US             (4u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_500US             (5u<<PD_CTRL_GM_AMPHI_POS)            /*default*/
#define PD_CTRL_GM_AMPHI_667US             (6u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_1000US         (7u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_DIS             (0<<7u)                                /*default*/
#define PD_CTRL_GM_AMPHI_EN             (1<<7u)

#define PD_CTRL_AMUX_ADDR           (0x08u)
#define PD_CTRL_AMUX_TRIG_POS           (0u)
#define PD_CTRL_AMUX_TRIG_SINGLE_FAULT  (0u<<PD_CTRL_AMUX_TRIG_POS)            /*default*/
#define PD_CTRL_AMUX_TRIG_SINGLE        (1u<<PD_CTRL_AMUX_TRIG_POS)
#define PD_CTRL_AMUX_TRIG_CONTINUOUS    (2u<<PD_CTRL_AMUX_TRIG_POS)
#define PD_CTRL_AMUX_TRIG_RESERVED      (3u<<PD_CTRL_AMUX_TRIG_POS)
#define PD_CTRL_AMUX_SEL_POS            (2u)
#define PD_CTRL_AMUX_SEL_V2             (0u<<PD_CTRL_AMUX_SEL_POS)             /*default*/
#define PD_CTRL_AMUX_SEL_V1             (1u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_SEL_CS2            (2u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_SEL_CS1            (3u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_SEL_ROTATING       (4u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_ADC_POS            (5u)
#define PD_CTRL_AMUX_ADC_EN             (0u<<PD_CTRL_AMUX_ADC_POS)             /*default*/
#define PD_CTRL_AMUX_ADC_DIS            (1u<<PD_CTRL_AMUX_ADC_POS)

#define PD_CTRL_INT_MASK_ADDR       (0x09u)
#define PD_CTRL_INT_MASK_CS_CLIND       (0b00000001u)
#define PD_CTRL_INT_MASK_OVP            (0b00000010u)
#define PD_CTRL_INT_MASK_OCP_P          (0b00000100u)
#define PD_CTRL_INT_MASK_PG_INT         (0b00001000u)
#define PD_CTRL_INT_MASK_TSD            (0b00010000u)
#define PD_CTRL_INT_MASK_UVP            (0b00100000u)
#define PD_CTRL_INT_MASK_VCHN           (0b01000000u)
#define PD_CTRL_INT_MASK_I2C_ACK        (0b10000000u)

#define PD_CTRL_INT2_MASK_ADDR      (0x0Au)
#define PD_CTRL_INT2_MASK_SHUTDOWN      (0b00000001u)

#define PD_CTRL_ADC_VOUT_ADDR       (0x10u)
#define PD_CTRL_ADC_VIN_ADDR        (0x11u)
#define PD_CTRL_ADC_CS2_ADDR        (0x12u)
#define PD_CTRL_ADC_CS1_ADDR        (0x13u)

#define PD_CTRL_INT_ADDR            (0x14u)
#define PD_CTRL_INT_CS_CLIND            (0b00000001u)
#define PD_CTRL_INT_OVP                 (0b00000010u)
#define PD_CTRL_INT_OCP_P               (0b00000100u)
#define PD_CTRL_INT_PG_INT              (0b00001000u)
#define PD_CTRL_INT_TSD                 (0b00010000u)
#define PD_CTRL_INT_UVP                 (0b00100000u)
#define PD_CTRL_INT_VCHN                (0b01000000u)
#define PD_CTRL_INT_I2C_ACK             (0b10000000u)

#define PD_CTRL_INT2_ADDR            (0x15u)
#define PD_CTRL_INT2_SHUTDOWN            (0b00000001u)

/* NCP Regulator I2C slave addresses for each PD port. */
#define NCP_PORT_1_SLAVE_ADDR                       (0x74u)
#define NCP_PORT_2_SLAVE_ADDR                       (0x75u)

/* Resolution of voltage changes (in mV) supported by the NCP regulator. */
#define NCP_REG_VOLT_RESOLUTION                     (100u)

/*
 * Excess voltage to be configured to make up for in-system drops. The unit used
 * is the resolution supported by the regulator itself.
 */
#define NCP_REG_EXCESS_VOLTAGE                      (1u)

/**
 * @brief Power Delivery Controller init
 * @return Returns true if controller is initialized, false otherwise
 */
bool pd_ctrl_init(void);

/**
 * @brief Selects Power Delivery voltage by NCP81239 instead of using VSELs
 * @param volt Voltage unit in PDO (mV)
 * @return Returns true if Data send to controller is successful, false otherwise
 */
bool set_pd_ctrl_voltage(uint8_t port, uint16_t volt);
    
#endif /* _NCP81239_H_ */

/* [] END OF FILE */
