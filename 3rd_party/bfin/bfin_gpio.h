/*
 * bfin_gpio.h
 *
 *  Created on: Sep 3, 2018
 *      Author: Dean
 */

#ifndef _BFIN_GPIO_H_
#define _BFIN_GPIO_H_

#include "bf706_device.h"

static inline void bfin_gpio_set_output(Portgroup *port, unsigned int mask)
{
	port->FER_CLR.reg = mask;
	port->DIR_SET.reg = mask;
}

static inline void bfin_gpio_data_set(Portgroup *port, unsigned int mask)
{
	port->DATA_SET.reg = mask;
}

static inline void bfin_gpio_data_clr(Portgroup *port, unsigned int mask)
{
	port->DATA_CLR.reg = mask;
}

#endif /* 3RD_PARTY_BFIN_BFIN_GPIO_H_ */
