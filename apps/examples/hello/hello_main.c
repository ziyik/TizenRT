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
#include <pthread.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/
static int counter = 0;
static pthread_t ble_multi_adv1_task_id;
static pthread_t ble_multi_adv2_task_id;

uint8_t def_ext_adv_data_A[] = {       
	// Flags       
	0x02,       
	0x01,//RTK_BT_LE_GAP_ADTYPE_FLAGS,       
	0x01|0x04,//RTK_BT_LE_GAP_ADTYPE_FLAGS_LIMITED | RTK_BT_LE_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,       
	// Local name       
	0x12,       
	0x09,//RTK_BT_LE_GAP_ADTYPE_LOCAL_NAME_COMPLETE,       
	'0', 'E', 'K', '_', 'B', 'T', '_', 'P', 'E', 'R', 'I', 'P', 'H', 'E', 'R', 'A', 'L',       
};     
     
uint8_t def_ext_resp_data_A[] = {       
	// Flags       
	0x02,       
	0x01,//RTK_BT_LE_GAP_ADTYPE_FLAGS,       
	0x01|0x04,//RTK_BT_LE_GAP_ADTYPE_FLAGS_LIMITED | RTK_BT_LE_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,       
	// Local name       
	0x12,       
	0x09,//RTK_BT_LE_GAP_ADTYPE_LOCAL_NAME_COMPLETE,       
	'0', 'E', 'S', 'P', '1', '_', '_', 'P', 'E', 'R', 'I', 'P', 'H', 'E', 'R', 'A', 'L',       
};     

uint8_t def_ext_adv_data_B[] = {       
	// Flags       
	0x02,       
	0x01,//RTK_BT_LE_GAP_ADTYPE_FLAGS,       
	0x01|0x04,//RTK_BT_LE_GAP_ADTYPE_FLAGS_LIMITED | RTK_BT_LE_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,       
	// Local name       
	0x12,       
	0x09,//RTK_BT_LE_GAP_ADTYPE_LOCAL_NAME_COMPLETE,       
	'1', 'E', 'K', '_', 'B', 'T', '_', 'P', 'E', 'R', 'I', 'P', 'H', 'E', 'R', 'A', 'L',       
};   

uint8_t def_ext_resp_data_B[] = {       
	// Flags       
	0x02,       
	0x01,//RTK_BT_LE_GAP_ADTYPE_FLAGS,       
	0x01|0x04,//RTK_BT_LE_GAP_ADTYPE_FLAGS_LIMITED | RTK_BT_LE_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,       
	// Local name       
	0x12,       
	0x09,//RTK_BT_LE_GAP_ADTYPE_LOCAL_NAME_COMPLETE,       
	'1', 'E', 'S', 'P', '2', '_', '_', 'P', 'E', 'R', 'I', 'P', 'H', 'E', 'R', 'A', 'L',       
};   
static void ble_multi_adv1_task(void)
{
	uint8_t adv_handle = 0; 
	uint8_t ret = 0; 
	uint8_t adv_data_len = 20;   

	sleep(1);
	while(1) {
		
		ret = ble_server_stop_multi_adv(adv_handle);
		if (ret != 0) {  
			printf("Fail to stop 0[%d]\n", ret);  
		}  else {
			printf("stop 0... ok\n");  
		}

		// sleep(100);
		
		def_ext_adv_data_A[6]++;
		def_ext_resp_data_A[6]++;
		ret = ble_server_set_multi_adv_data(adv_handle, adv_data_len, def_ext_adv_data_A);
		if (ret != 0) {  
			printf("Fail to set adv data 0[%d]\n", ret);  
		}  else {
			printf("set adv data 0... ok\n");  
		}

		// sleep(100);

		ret = ble_server_set_multi_resp_data(adv_handle, adv_data_len, def_ext_resp_data_A);
		if (ret != 0) {  
			printf("Fail to set adv resp 0[%d]\n", ret);  
		}  else {
			printf("set adv resp 0... ok\n");  
		}

		// sleep(100);

		ret = ble_server_start_multi_adv(adv_handle); 
		if (ret != 0) {  
			printf("Fail to start adv 0[%d]\n", ret);  
		}  else {
			printf("start adv 0... ok\n");  
		}

		//sleep(100);

		printf("ble_multi_adv0_task\n");
		sleep(2);
	}
}

static void ble_multi_adv2_task(void)
{
	sleep(1);
	
	uint8_t adv_handle = 1; 
	uint8_t ret = 0; 
	uint8_t adv_data_len = 20;
	while(1) {
		
		ret = ble_server_stop_multi_adv(adv_handle);
		if (ret != 0) {  
			printf("Fail to stop 1[%d]\n", ret);  
		}  else {
			printf("stop 1... ok\n");  
		}

		// sleep(100);

		def_ext_adv_data_B[6]++;
		def_ext_resp_data_B[6]++;
		ret = ble_server_set_multi_adv_data(adv_handle, adv_data_len, def_ext_adv_data_B);
		if (ret != 0) {  
			printf("Fail to set adv data 1[%d]\n", ret);  
		}  else {
			printf("set adv data 1... ok\n");  
		}

		// sleep(100);

		ret = ble_server_set_multi_resp_data(adv_handle, adv_data_len, def_ext_resp_data_B);
		if (ret != 0) {  
			printf("Fail to set adv resp 1[%d]\n", ret);  
		}  else {
			printf("set adv resp 1... ok\n");  
		}

		// sleep(100);

		ret = ble_server_start_multi_adv(adv_handle); 
		if (ret != 0) {  
			printf("Fail to start adv 1[%d]\n", ret);  
		}  else {
			printf("start adv 1... ok\n");  
		}

		// sleep(100);
		printf("ble_multi_adv2_task\n");
		sleep(2);
	}
}

static int hello_task_create(pthread_t *task, char *task_name, void *task_function, void *args, int priority, unsigned int stack_size)
{
	pthread_t tid;
	pthread_attr_t attr;
	struct sched_param sparam;

	if (!task || !task_name || !task_function ||
		priority < 10 ||
		priority > 200 || stack_size <= 0)
		return -1;

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, stack_size);
	sparam.sched_priority = priority;
	pthread_attr_setschedparam(&attr, &sparam);

	if (pthread_create(&tid, &attr, (pthread_startroutine_t)task_function,
				(pthread_addr_t)args) == 0) {
		pthread_setname_np(tid, task_name);
		pthread_detach(tid);
		*task = tid;
		return 0;
	}

	return -1;	
}


static void adv1_therad(void)
{
	printf("adv1_therad\n");

	if(hello_task_create(&ble_multi_adv1_task_id, "ble_multi_adv1", ble_multi_adv1_task, NULL, 100, 1024*5) != 0) {
	 	printf("%d : %s", __LINE__, "Failed to run ble_multi_adv1_task");
		return;
	}	
}

static void adv2_therad(void)
{
	printf("adv2_therad\n");

	if(hello_task_create(&ble_multi_adv2_task_id, "ble_multi_adv2", ble_multi_adv2_task, NULL, 100, 1024*5) != 0) {
	 	printf("%d : %s", __LINE__, "Failed to run ble_multi_adv2_task");
		return;
	}	
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");

	counter++;
	if(counter >= 2) {
		adv1_therad();
		adv2_therad();
	}

	return 0;
}
