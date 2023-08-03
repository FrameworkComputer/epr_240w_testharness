#ifndef _SCPI_PSU_H_
#define _SCPI_PSU_H_

#include <stdbool.h>
#include <stdint.h>

bool set_pd_ctrl_voltage(uint8_t port, uint16_t volt);
bool pd_ctrl_init(void);



#endif /* _SCPI_PSU_H_ */
