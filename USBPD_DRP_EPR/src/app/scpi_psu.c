

#include "cy_pdl.h"
#include "cybsp.h"


cy_stc_scb_uart_context_t UART_context;


bool pd_ctrl_init(void)
{
    bool status = 1;

    Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    Cy_SCB_UART_Enable(UART_HW);

    Cy_SCB_UART_PutString(UART_HW, "*IDN?\r\n");
        Cy_SysLib_Delay(500);

    Cy_SCB_UART_PutString(UART_HW, "CURR 5.00\r\n");
    Cy_SysLib_Delay(500);

    Cy_SCB_UART_PutString(UART_HW, "OUTP ON\r\n");
    Cy_SysLib_Delay(500);

    
    return status;
}

/* Setting power voltage on MPS controller */
bool set_pd_ctrl_voltage(uint8_t port, uint16_t volt)
{
    uint8_t status = 1;
    volt = volt / 1000;
    char  out[32] = {'V', 'O', 'L', 'T', ' ', '0' + (volt / 10), '0' + (volt % 10), '.', '0', '0', '\r', '\n', 0, 0};

    Cy_SCB_UART_PutString(UART_HW, out);
    return (status);
}