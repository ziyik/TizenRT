/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
 ****************************************************************************/
/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#ifndef CONFIG_DISABLE_POLL
#include <poll.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <tinyara/fs/ioctl.h>
#define PM_DRVPATH 	"/dev/pm"
/****************************************************************************
* Pre-processor Definitions
****************************************************************************/
#ifndef CONFIG_UART_LOOPBACK_PORT
#define CONFIG_UART_LOOPBACK_PORT 2
#endif
#define UART_DEV_PATH 			"/dev/ttyS%d"
#define UART_POLL_TIMEOUT_MS 	10000
#define TEST_STR 			"1234567890abcdefghijklmnopqrstuvwxyz"
#define TEST_STR_LEN 	74
static int send_read = true;
static int uart_tx_loop(void)
{
		int fd = 0;
		char port[20] = {'\0'};
		sprintf(port, UART_DEV_PATH, CONFIG_UART_LOOPBACK_PORT);
		fd = open(port, O_RDWR | O_SYNC, 0666);
		if (fd < 0) {
				printf("ERROR: Failed to open Tx UART port : %s\n", port);
				return -1;
		}
		while(1) {
				usleep(100*1000);
				char test_data[TEST_STR_LEN] = TEST_STR;
				write(fd, (void *)test_data, TEST_STR_LEN);
				printf("SEND SIZE: 	%d\n", TEST_STR_LEN);
				printf("SEND UART: 	%s\n", TEST_STR);
		}
		close(fd);
		return 0;
}
static int uart_rx_loop(void)
{
		int fd = 0;
		char port[20] = {'\0'};
		ssize_t ret_size;
		int remain_size;
		sprintf(port, UART_DEV_PATH, CONFIG_UART_LOOPBACK_PORT);
		fd = open(port, O_RDWR | O_SYNC, 0666);
		if (fd < 0) {
				printf("ERROR: Failed to open Tx UART port : %s\n", port);
				return -1;
		}
		while (1) {
		char read_buf[TEST_STR_LEN];
		char *read_ptr = read_buf;
		int read_total_size = 0;
		remain_size = TEST_STR_LEN;
				while (0 < remain_size) {
						ret_size = read(fd, (void *)read_ptr, remain_size);
						remain_size -= ret_size;
						read_ptr += ret_size;
						read_total_size += ret_size;
				}
				printf("RECIVE SIZE: %d\n", read_total_size);
				printf("RECIVE UART: %s\n", read_buf);
/*
				if (strncmp(read_buf, TEST_STR, TEST_STR_LEN) == 0) {
						printf("UART LOOPBACK TEST: PASSED\n");
				} else {
						printf("UART LOOPBACK TEST: FAILED (It does not match)\n");
				}
*/
		}
		close(fd);
		return 0;
}
/****************************************************************************
* Public functions
****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char **argv)
#endif
{
		pthread_t tx_tid, rx_tid, pm_tid;
		pthread_addr_t retval;
		printf("######################### UART loopback test START #########################\n");
		if (pthread_create(&rx_tid, NULL, (pthread_startroutine_t)uart_rx_loop, NULL) < 0) {
				printf("Failed to create rx pthread for running logs\n");
				return -1;
		}
		/*if (pthread_create(&tx_tid, NULL, (pthread_startroutine_t)uart_tx_loop, NULL) < 0) {
				printf("Failed to create tx pthread for running logs\n");
				pthread_cancel(rx_tid);
				return -1;
		}*/
		int fd = open(PM_DRVPATH, O_WRONLY);
		if (fd < 0) {
				return 0;
		}
		if (ioctl(fd, PMIOC_RESUME, 0) < 0) {
				return 0;
		}
		close(fd);
		return 0;
}
