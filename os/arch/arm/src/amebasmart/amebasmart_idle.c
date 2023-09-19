/****************************************************************************
 * arch/arm/src/amebasmart/amebasmart_idle.c
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

#include <tinyara/config.h>

#include <tinyara/arch.h>
#include "arm_internal.h"

#ifdef CONFIG_PM
#include <tinyara/pm/pm.h>
//For reference to up_rtc_gettime() and up_rtc_time()
#include "amebasmart_rtc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

//TODO: -PENDING- implementation needed if enter_sleep_write function support in procfs is required
//		Refer up_pmsleep() in arch/arm/src/stm32l4/stm32l4_idle.c
#ifdef CONFIG_ARCH_SUPPORT_ENTER_SLEEP
int up_pmsleep(void)
{
	//TODO:
	return OK;
}
#endif

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_idlepm(void)
{
	static enum pm_state_e oldstate = PM_NORMAL;
	enum pm_state_e newstate;
	irqstate_t flags;
	int ret;

#ifdef CONFIG_RTC
	clock_t time_intval;
	struct tm tp1, tp2;
#endif

	/* Decide, which power saving level can be obtained */
	newstate = pm_checkstate(PM_IDLE_DOMAIN);

	/* Check for state changes */
	if (newstate != oldstate)
	{
		//TODO: Critical section code needed for SMP case?
		//Any additional implications of putting a core in critical section while trying to sleep?
		flags = irqsave();

		/* Perform board-specific, state-dependent logic here */
	printf("newstate= %d oldstate=%d\n", newstate, oldstate);

		/* Then force the global state change */

		ret = pm_changestate(PM_IDLE_DOMAIN, newstate);
		if (ret < 0) {
			/* The new state change failed, revert to the preceding state */
			(void)pm_changestate(PM_IDLE_DOMAIN, oldstate);
		} else {
			/* Save the new state */
			oldstate = newstate;
		}

		/* MCU-specific power management logic */
		switch (newstate) {
			case PM_NORMAL:
				break;

			case PM_IDLE:
				break;

			case PM_STANDBY:
#ifdef CONFIG_SCHED_TICKSUPPRESS
#ifdef CONFIG_RTC
				/* Note RTC time before sleep */
#ifdef CONFIG_RTC_HIRES
				//TODO: HIRES timer function also returns with a precision of seconds only?
				ret = up_rtc_gettime(&tp1);
#elif CONFIG_RTC_DRIVER
				//TODO: up_rtc_time() is a dummy function, so how to handle RTC time requests wihtout HIRES?
				ret = up_rtc_time();
#else
				//TODO: What if neither is available?
#endif
#endif	/* CONFIG_RTC */
				//TODO: -PENDING- Supress ticks function implementation, if desired, required from Realtek. Refer arch/arm/src/stm32l4/stm32l4_ticksuppress.c
				/* Disable tick interrupts */
				//supress_ticks();

				/* Set waketime interrupt for tickless idle  */
				set_waketime_interrupt();

				/* Enter sleep mode */
				//TODO: -PENDING- Put Amebasmart into sleep mode
				//stm32l4_pmsleep(false);

#ifdef CONFIG_RTC
				/* Read RTC time after wakeup */
#ifdef CONFIG_RTC_HIRES
				//TODO: HIRES timer function also returns with a precision of seconds only?
				ret = up_rtc_gettime(&tp2);
				time_intval = SEC2TICK(mktime(&tp2) - mktime(&tp1));
#elif CONFIG_RTC_DRIVER
				//TODO: up_rtc_time() is a dummy function, so how to handle RTC time requests wihtout HIRES?
				ret = up_rtc_time();
				time_intval = -1;
#else
				//TODO: What if neither is available?
				time_intval = -1;
#endif
				/* Update ticks */
				g_system_timer +=  time_intval;
				uwTick += TICK2MSEC(time_intval);
				/* Execute waketime interrupt */
				execute_waketime_interrupt(time_intval);
#endif
				//TODO: -PENDING- Implemented along with suppress_ticks()
				/* Enable tick interrupts */
				//enable_ticks();

				/* We dont want state change directly
				* it is the resposibility of the scheduled
				* event to inform the PM Core about the
				* pm activity based on its requirement */
				//oldstate = PM_IDLE; -> Re visit
#else
				//TODO: -PENDING- Amebasmart function to put to sleep, without supressing ticks
				//stm32l4_pmsleep(false);
#endif	/* CONFIG_SCHED_TICKSUPPRESS */
				break;

			case PM_SLEEP:
				//TODO: Revisit and check if something like this is required in Amebasmart
				// Refer to set_exti_button() function in arch/arm/src/stm32l4/stm32l4_idle.c
				/* Set EXTI interrupt */
				//set_exti_button();

				//TODO: -PENDING- Amebasmart function to enter STOP mode, refer arch/arm/src/stm32l4/stm32l4_pmstop.c
				//(void)stm32l4_pmstop2();

				/* Re configure clocks */
				//TODO: -PENDING- Amebasmart function to enable clock and re-configure.
				//stm32l4_clockenable();

				ret = pm_changestate(PM_IDLE_DOMAIN, PM_NORMAL);
				if (ret < 0) {
					oldstate = PM_NORMAL;
				}
				printf("Wakeup from STOP2!!\n");
				break;

			default:
				break;
		}

		//TODO: Handle critical section access logic for SMP case in 8730E?
		//Is leave_critical_section required? In accordance with irqsave()^
		irqrestore(flags);
	}
}
#else
#define up_idlepm()
#endif

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
	/* If the system is idle and there are no timer interrupts, then process
	 * "fake" timer interrupts. Hopefully, something will wake up.
	 */

	nxsched_process_timer();
#else

	/* Sleep until an interrupt occurs to save power */

	up_idlepm();
#endif
}

#ifdef CONFIG_PM
void arm_pminitialize(void)
{
	/* Then initialize the TinyAra power management subsystem proper */
	pm_initialize();
}
#endif