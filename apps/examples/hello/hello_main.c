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

/****************************************************************************
 * hello_main
 ****************************************************************************/

void hello_task(void *param)
{
	uint8_t pass_param[2];
	printf("Entering\n");

	memcpy(pass_param, param, sizeof(pass_param));
	printf("pass_param[0]: %x\n", pass_param[0]);
	printf("pass_param[1]: %x\n", pass_param[1]);
}

bool hello_task_create(void *pp_handle, const char *p_name, void (*p_routine)(void *),
                      void *p_param, uint16_t stack_size, uint16_t priority)
{
	int result;

dbg("%s create getpid: %d\n", p_name, getpid());

	pthread_t thread_info;
	pthread_attr_t attr;
	int res = 0;

	res = pthread_attr_init(&attr);
	if (res != 0) {
		dbg("Failed to pthread_attr_init\n");
		return -1;
	}
	stack_size = stack_size * sizeof(uint32_t);
	dbg("stack_size: %d\n", stack_size);

	res = pthread_attr_setstacksize(&attr, stack_size);
	if (res != 0) {
		dbg("Failed to pthread_attr_setstacksize\n");
		goto CREATE_FAIL;
	}

	priority = priority + 100;
	if (priority > 255)
		priority = 100;

	struct sched_param prio;
	prio.sched_priority = priority;
	dbg("priority: %d\n", priority);

	res = pthread_attr_setschedparam(&attr, &prio);
	if (res != 0) {
		dbg("Failed to pthread_attr_setschedparam\n");
		goto CREATE_FAIL;
	}

dbg("pthread_create\n");
	res = pthread_create(&thread_info, &attr, (pthread_startroutine_t)p_routine, p_param);
	if (res < 0) {
		dbg("Failed to pthread_create\n");
		goto CREATE_FAIL;
	}

	pthread_setname_np(thread_info, p_name);
dbg("pthread_setname_np\n");
	pthread_t *thread_info_new = malloc(sizeof(pthread_t));
	memcpy(thread_info_new, &thread_info, sizeof(pthread_t));
	pp_handle = (thread_info_new);

dbg("pthread_detach\n");
	//pthread_detach(*thread_info);
	return 0;

CREATE_FAIL:
	dbg("%s create fail\n", p_name);
	pthread_attr_destroy(&attr);
	return -1;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	void *task_poninter_local;
	uint8_t pass_param[2];

	printf("Hello, World!!\n");

	pass_param[0] = 0xFF;
	pass_param[1] = 0xEE;
	hello_task_create(task_poninter_local, "Temporary_Task", hello_task, pass_param, 1024, 5);
	printf("hello_task_create Done!!\n");

	return 0;
}
