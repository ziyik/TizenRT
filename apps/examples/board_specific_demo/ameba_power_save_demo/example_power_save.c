/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
 /******************************************************************************

 * Copyright 2023 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *

  ******************************************************************************/


#include <stdio.h>
#include <sched.h>
#include <semaphore.h>


// #include "serial_api.h"
// #include "uart_ext.h"

// #define UART_IDX	0
// #define UART_BAUD	38400

// volatile char rc = 0;

// void uart_send_string(serial_t *sobj, char *pstr)
// {
// 	unsigned int i = 0;
// 	while (*(pstr + i) != 0) {
// 		serial_putc(sobj, *(pstr + i));
// 		i++;
// 	}
// }

// void uart_irq(uint32_t id, SerialIrq event)
// {
// 	serial_t *sobj = (void *)id;

// 	if (event == RxIrq) {
// 		while (serial_readable(sobj)) {
// 			rc = serial_getc(sobj);
// 			serial_putc(sobj, rc);
// 			printf("serial_putc\n");
// 		}
// 	}

// 	if (event == TxIrq && rc != 0) {
// 		rc = 0;
// 	}

// }

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int powersave_main(int argc, char *argv[])
#endif
{
// 	serial_t sobj;
// 	printf("uart_irq_demo\n");
// 	printf("uart_irq_demo\n");
// 	sobj.uart_idx = UART_IDX;

// 	serial_init(&sobj, UART_TX, UART_RX);
// 	serial_baud(&sobj, UART_BAUD);
// 	serial_format(&sobj, 8, ParityNone, 1);

// 	uart_send_string(&sobj, "UART IRQ API Demo...\r\n");
// 	uart_send_string(&sobj, "Hello World!!!\r\n");
// 	serial_irq_handler(&sobj, uart_irq, (uint32_t)&sobj);
// 	serial_irq_set(&sobj, RxIrq, 1);
// 	serial_irq_set(&sobj, TxIrq, 1);

// 	//while (1);
// printf("end uart_irq_demo\n");
// 	vTaskDelete(NULL);
// 	return 0;

}