/****************************************************************************
 *
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
 ****************************************************************************/
/****************************************************************************
 * arch/arm/src/armv7-a/arm_initialize.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/arch.h>
#include <tinyara/board.h>
#include <arch/board/board.h>

#include "up_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
static inline void arm_color_intstack(void)
{
#ifdef CONFIG_SMP
	uint32_t *ptr = (uint32_t *)arm_intstack_alloc();
#else
	uint32_t *ptr = (uint32_t *)&g_intstackalloc;
#endif
	ssize_t size;

	for (size = ((CONFIG_ARCH_INTERRUPTSTACK & ~3) * CONFIG_SMP_NCPUS); size > 0; size -= sizeof(uint32_t)) {
		*ptr++ = INTSTACK_COLOR;
	}
}
#else
#define arm_color_intstack()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.  The architecture specific
 *   details of initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the clock, and
 *   registering device drivers are some of the things that are different
 *   for each processor and hardware platform.
 *
 *   up_initialize is called after the OS initialized but before the user
 *   initialization logic has been started and before the libraries have
 *   been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
#ifdef CONFIG_SYSTEM_REBOOT_REASON
	up_reboot_reason_init();
	lldbg("[Reboot Reason] : %d\n", up_reboot_reason_read());
#endif

	/* Colorize the interrupt stack */

	arm_color_intstack();

	/* Initialize the interrupt subsystem */

	up_irqinitialize();

#ifdef CONFIG_PM
	/* Initialize the power management subsystem.  This MCU-specific function
	 * must be called *very* early in the initialization sequence *before* any
	 * other device drivers are initialized (since they may attempt to register
	 * with the power management subsystem).
	 */

	arm_pminitialize();
#endif

#ifdef CONFIG_ARCH_DMA
	/* Initialize the DMA subsystem if the weak function arm_dma_initialize has
	 * been brought into the build
	 */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
	if (arm_dma_initialize)
#endif
	{
		arm_dma_initialize();
	}
#endif

#if !defined(CONFIG_SUPPRESS_INTERRUPTS) && !defined(CONFIG_SUPPRESS_TIMER_INTS) && \
	!defined(CONFIG_SYSTEMTICK_EXTCLK)
	up_timer_initialize();

#ifdef CONFIG_WATCHDOG_FOR_IRQ
#if ((CONFIG_WATCHDOG_FOR_IRQ_INTERVAL * 1000) <= CONFIG_USEC_PER_TICK)
#error "CONFIG_WATCHDOG_FOR_IRQ_INTERVAL should be greater than CONFIG_USEC_PER_TICK"
#endif
	up_wdog_init(CONFIG_WATCHDOG_FOR_IRQ_INTERVAL);
#endif
#endif

	/* Initialize pipe */

#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
	pipe_initialize();
#endif

	/* Register devices */

#if CONFIG_NFILE_DESCRIPTORS > 0

#if defined(CONFIG_DEV_NULL)
	devnull_register();			/* Standard /dev/null */
#endif

#ifdef CONFIG_VIDEO_NULL
	video_null_initialize("/dev/video0");	/* Standard /dev/video0 */
#endif

#if defined(CONFIG_BLUETOOTH) && defined(CONFIG_BLUETOOTH_NULL)
	btnull_register();			/* bluetooth bt_null */
#endif

#ifdef CONFIG_DEV_URANDOM
	devurandom_register();		/* /dev/urandom */
#endif

#if defined(CONFIG_DEV_ZERO)
	devzero_register();			/* Standard /dev/zero */
#endif

#if defined(CONFIG_VIRTKEY)
	virtkey_register();			/* virtual key driver */
#endif

#endif							/* CONFIG_NFILE_DESCRIPTORS */

	/* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
	up_serialinit();
#endif
#ifdef CONFIG_AMEBASMART_USBDEVICE
	/*if USB device enabled, we will unregister loguart and register /dev/console to usb*/
	register_usb();
#endif
	/* Initialize the console device driver (if it is other than the standard
	 * serial driver).
	 */

#if defined(CONFIG_DEV_LOWCONSOLE)
	lowconsole_init();
#elif defined(CONFIG_SYSLOG_CONSOLE)
	syslog_console_init();
#elif defined(CONFIG_RAMLOG_CONSOLE)
	ramlog_consoleinit();
#endif

	/* Initialize the network */

	arm_netinitialize();

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
	/* Initialize USB -- device and/or host */

	arm_usbinitialize();
#endif

	/* Initialize the L2 cache if present and selected */
#ifdef CONFIG_ARMV7A_HAVE_L2CC
	arm_l2ccinitialize();
#endif

	board_autoled_on(LED_IRQSENABLED);
}
