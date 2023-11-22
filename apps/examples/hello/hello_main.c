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

#include <pm/pm.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");
	/* Call stay IOCTL to keep PM domain in current state because maybe we know we need to do some
	 * non-frequent work with time gaps and dont want the board to sleep in between
	 */
	pm_ioctl(PM_IOC_STAY);

	{
		//The work happens
	}

	/* The work is done, now relax the stay on the current state of the domain */
	pm_ioctl(PM_IOC_RELAX);

	{
		//App does some other non-blocking work
	}

	/* Now, maybe the app has completed the major part of its work and is suggesting that the board
	 * can go to sleep, given that there is nothing else going on.
	 * This does not mean that the board will immediately be put to sleep. If there is a stay applied
	 * by another application, then this SLEEP request will be DISCARDED(can be discussed if otherwise).
	 */
	pm_ioctl(PM_IOC_SLEEP);

	/* However, as discussed before, we may need the application to request/suggest the core to go to
	 * sleep for a specified amount of time. Alternatively, the application may want the core to be woken up at
	 * a specific timestamp when it expects to be able to do resume some work. In this case, the application
	 * should be able to request sleep with a timed interrupt.
	 */
	//pm_ioctl(PM_IOC_TIMEDSLEEP, <second argument for timed interval>, \
			<third argument for current time so that timer interrupt can be adjusted for elapsed time>)

	/* NOTE: At the beginning of this process, it seems that PM_IOC_RELAX and PM_IOC_SLEEP are similar in the
	 * way that both will result in the drop of PM state if feasible, and otherwise not make any state changes.
	 * PM_IOC_RELAX wil reduce the staycount by 1 i.e. enabling natural state transition thereafter, and 
	 * PM_IOC_SLEEP will try to invoke sleep immediately, but may be rejected and discarded.
	 *
	 * Hence, it is also noteworthy that: -
	 * pm_ioctl(PM_IOC_STAY);
	 * pm_ioctl(PM_IOC_RELAX); -> PM_IOC_RELAX has to be used after PM_IOC_STAY otherwise domain's state transition will be blocked
	 * pm_ioctl(PM_IOC_SLEEP);
	 */
	return 0;
}
