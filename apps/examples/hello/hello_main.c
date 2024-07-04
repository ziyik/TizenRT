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
#include <ble_manager/ble_manager.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/
static int g_scan_state = -1;
static ble_scan_callback_list scan_config = {
	ble_scan_state_changed_cb,
	NULL,
};

static void ble_scan_state_changed_cb(ble_scan_state_e scan_state)
{
	printf("'%s' is called[%d]\n", __FUNCTION__, scan_state);
	if (scan_state == BLE_SCAN_STOPPED) {
		g_scan_state = 0;
	} else if (scan_state == BLE_SCAN_STARTED) {
		g_scan_state = 1;
	}
	return;
}

uint8_t scan_count = 0;
static void ble_device_scanned_cb_for_test(ble_scanned_device *scanned_device)
{
	scan_count++;
	if (scan_count > 250) {
		printf("scan_count done[%d]\n", scan_count);
		scan_count = 0;
	}
}

static void *_test_BLE_thread(void *data)
{
	ble_result_e ret = BLE_MANAGER_FAIL;

	while(1) {
		/* BLE Init */
		ret = ble_manager_init(NULL);
		if (ret != BLE_MANAGER_SUCCESS) {
			if (ret != BLE_MANAGER_ALREADY_WORKING) {
				printf("init with null fail[%d]\n", ret);
				goto ble_rmc_done;
			}
			printf("init is already done\n");
		} else {
			printf("init with NULL done[%d]\n", ret);
		}

		/* Scan Start */
		printf("Scan Start without filter !\n");
		scan_config.device_scanned_cb = ble_device_scanned_cb_for_test;
		ret = ble_client_start_scan(NULL, &scan_config);

		if (ret != BLE_MANAGER_SUCCESS) {
			printf("scan start fail[%d]\n", ret);
			goto ble_rmc_done;
		}


		msleep(100);
		ret = ble_server_start_adv();	/* Adv Start */
		if (ret != BLE_MANAGER_SUCCESS) {
			printf("Fail to start adv [%d]\n", ret);
			goto ble_rmc_done;
		}
		printf("Start adv ... ok\n");



		msleep(100);
		ret = ble_server_stop_adv();	/* Adv Stop */
		if (ret != BLE_MANAGER_SUCCESS) {
			printf("Fail to stop adv [%d]\n", ret);
			goto ble_rmc_done;
		}
		printf("Stop adv ... ok\n");

		/* Scan Stop */
		printf("stop !\n");
		ret = ble_client_stop_scan();
		if (ret != BLE_MANAGER_SUCCESS) {
			printf("scan stop fail[%d]\n", ret);
			goto ble_rmc_done;
		}

		/* Deinit */
		ret = ble_manager_deinit();
		printf("deinit done[%d]\n", ret);
	}

ble_rmc_done:
	printf("done\n");
	return 0;
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");
	return 0;
}
