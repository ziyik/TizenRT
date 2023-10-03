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

// #include <tinyara/arch.h>
#include "../../include/armv7-a/irq.h"
#include "arm_internal.h"

#ifdef CONFIG_PM
#include <tinyara/pm/pm.h>
#include "amebasmart_config.h"
#include "arch_timer.h"
//For reference to up_rtc_gettime() and up_rtc_time()
// #include "amebasmart_rtc.h"
#include "ameba_soc.h"
#endif

#ifdef CONFIG_SMP
#include "gic.h"
#endif

static u32 system_can_yield = 1;
static bool system_np_wakelock = 1;
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
RTIM_TimeBaseInitTypeDef TIM_InitStruct_GT[8];
static void pg_timer_int_handler(u32 Data)
{

	RTIM_TimeBaseInitTypeDef *TIM_InitStruct = (RTIM_TimeBaseInitTypeDef *) Data;
	printf("pg Ap timer handler\n");
	printf("WAK_Event0 = %x 1=%x\n", HAL_READ32(PMC_BASE, WAK_STATUS0), HAL_READ32(PMC_BASE, WAK_STATUS1));

	RTIM_INTClear(TIMx[TIM_InitStruct->TIM_Idx]);

	printf("WAK_Event0 = %x 1=%x\n", HAL_READ32(PMC_BASE, WAK_STATUS0), HAL_READ32(PMC_BASE, WAK_STATUS1));
	RTIM_Cmd(TIMx[TIM_InitStruct->TIM_Idx], DISABLE);

}

static void set_timer_interrupt(u32 TimerIdx, u32 Timercnt) {
	printf("hs pg_sleep_Test aon_timer:%d Ms\n", Timercnt);
	// printf("\nClock(g_system_timer) before sleep: %d\n", TICK2MSEC(clock_systimer()));
	RTIM_TimeBaseInitTypeDef *pTIM_InitStruct_temp = &TIM_InitStruct_GT[TimerIdx];
	// RCC_PeriphClockCmd(APBPeriph_TIM0, APBPeriph_TIM0_CLOCK, ENABLE);
	RCC_PeriphClockCmd(APBPeriph_TIM1, APBPeriph_TIM1_CLOCK, ENABLE);
	// RCC_PeriphClockCmd(APBPeriph_TIM2, APBPeriph_TIM2_CLOCK, ENABLE);
	// RCC_PeriphClockCmd(APBPeriph_TIM3, APBPeriph_TIM3_CLOCK, ENABLE);
	// RCC_PeriphClockCmd(APBPeriph_TIM4, APBPeriph_TIM4_CLOCK, ENABLE);
	// RCC_PeriphClockCmd(APBPeriph_TIM5, APBPeriph_TIM5_CLOCK, ENABLE);
	// RCC_PeriphClockCmd(APBPeriph_TIM6, APBPeriph_TIM6_CLOCK, ENABLE);
	// RCC_PeriphClockCmd(APBPeriph_TIM7, APBPeriph_TIM7_CLOCK, ENABLE);

	RTIM_TimeBaseStructInit(pTIM_InitStruct_temp);

	pTIM_InitStruct_temp->TIM_Idx = TimerIdx;
	pTIM_InitStruct_temp->TIM_Prescaler = 0x00;
	pTIM_InitStruct_temp->TIM_Period = 32768 * Timercnt - 1;//0xFFFF>>11;

	pTIM_InitStruct_temp->TIM_UpdateEvent = ENABLE; /* UEV enable */
	pTIM_InitStruct_temp->TIM_UpdateSource = TIM_UpdateSource_Overflow;
	pTIM_InitStruct_temp->TIM_ARRProtection = ENABLE;

	RTIM_TimeBaseInit(TIMx[TimerIdx], pTIM_InitStruct_temp, TIMx_irq[TimerIdx], (IRQ_FUN) pg_timer_int_handler,
						(u32)pTIM_InitStruct_temp);
	RTIM_INTConfig(TIMx[TimerIdx], TIM_IT_Update, ENABLE);
	RTIM_Cmd(TIMx[TimerIdx], ENABLE);
	printf("hs Timer %x cnt %d\n", (WAKE_SRC_Timer0 << TimerIdx), Timercnt);
	SOCPS_SetAPWakeEvent_MSK0((WAKE_SRC_Timer0 << TimerIdx), ENABLE);
}

bool set_interrupt_count = 0;
#ifdef CONFIG_PM
static void up_idlepm(void)
{
	static enum pm_state_e oldstate = PM_NORMAL;
	uint32_t xModifiableIdleTime = 0;
	enum pm_state_e newstate;
	irqstate_t flags;
	int ret;

	/* Decide, which power saving level can be obtained */
	newstate = pm_checkstate(PM_IDLE_DOMAIN);

	/* Check for state changes */
	if (newstate != oldstate)
	{
		//TODO: Critical section code needed for SMP case?
		//Any additional implications of putting a core in critical section while trying to sleep?
		/* Perform board-specific, state-dependent logic here */
	  	printf("newstate= %d oldstate=%d\n", newstate, oldstate);
		if(oldstate == PM_STANDBY && newstate != PM_SLEEP) {
			// rtk_NP_powersave_disable();
			system_np_wakelock = 1;
		}
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
				printf("\n[%s] - %d, state = %d\n",__FUNCTION__,__LINE__, newstate);
			case PM_IDLE:
				printf("\n[%s] - %d, state = %d\n",__FUNCTION__,__LINE__, newstate);
				break;
			case PM_STANDBY:
				if(system_np_wakelock) {
					// rtk_NP_powersave_enable();
					system_np_wakelock = 0;
				}
				printf("\n[%s] - %d, state = %d\n",__FUNCTION__,__LINE__, newstate);
				break;
			case PM_SLEEP:
				/* need further check*/
				system_can_yield = 0;
				// set interrupt source
				if(!set_interrupt_count) {
					set_timer_interrupt(1, 5);
					set_interrupt_count = 1;
				}
				if (up_cpu_index() == 0) {
					/* mask sys tick interrupt*/
					arm_arch_timer_int_mask(1);
					flags = irqsave();
					if (tizenrt_ready_to_sleep()) {
// Consider for dual core condition
#if ( configNUM_CORES > 1 )
						/*PG flow */
						if (pmu_get_sleep_type() == SLEEP_PG) {
							/* CPU1 just come back from pg, so can't sleep here */
							if (pmu_get_secondary_cpu_state(1) == CPU1_WAKE_FROM_PG) {
								goto EXIT;
							}

							/* CPU1 is in task schedular, tell CPU1 to enter hotplug */
							if (pmu_get_secondary_cpu_state(1) == CPU1_RUNNING) {
								/* CPU1 may in WFI idle state. Wake it up to enter hotplug itself */
								up_irq_enable();
								arm_gic_raise_softirq(1, 0);
								arm_arch_timer_int_mask(0);
								DelayUs(100);
								goto EXIT;
							}
							/* CG flow */
						} else {
							if (!check_wfi_state(1)) {
								goto EXIT;
							}
						}
#endif
						// Interrupt source from BT/UART will wake cpu up, just leave expected idle time as 0
						// Enter sleep mode for AP
						configPRE_SLEEP_PROCESSING(xModifiableIdleTime);
						/* When wake from pg, arm timer has been reset, so a new compare value is necessary to
						trigger an timer interrupt */
						if (pmu_get_sleep_type() == SLEEP_PG) {
							arm_arch_timer_set_compare(arm_arch_timer_count() + 50000);
						}
						arm_arch_timer_int_mask(0);
						configPOST_SLEEP_PROCESSING(xModifiableIdleTime);
					}
					else {
						/* power saving when idle*/
						arm_arch_timer_int_mask(0);
						__asm(" DSB");
						__asm(" WFI");
						__asm(" ISB");
					}
#if ( configNUM_CORES > 1 )
EXIT:
#endif				
					/* Re-enable interrupts and sys tick*/
					up_irq_enable();
					// irqrestore(flags);
				}
				else if (up_cpu_index() == 1) {
					if (pmu_get_sleep_type() == SLEEP_PG) {
						if (tizenrt_ready_to_sleep()) {
							/* CPU1 will enter hotplug state. Raise a task yield to migrate its task */
							pmu_set_secondary_cpu_state(1, CPU1_HOTPLUG);
							// Check portYIELD();
							portYIELD();
						}
					}

					flags = irqsave();
					__asm("	DSB");
					__asm("	WFI");
					__asm("	ISB");
					up_irq_enable();
				}
				/* need further check*/
				system_can_yield = 1;
				// IPC AP->NP to acquire wakelock
				system_np_wakelock = 1;
				// rtk_NP_powersave_disable();
				// Switch status back to normal mode after wake up from interrupt
				ret = pm_changestate(PM_IDLE_DOMAIN, PM_NORMAL);
				// pm_stay(PM_IDLE_DOMAIN, PM_NORMAL);
				printf("Check the state 1 : %d\n", pm_checkstate(PM_IDLE_DOMAIN));
				printf("Check the state 2 : %d\n", pm_querystate(PM_IDLE_DOMAIN));
				if (ret < 0) {
					oldstate = PM_NORMAL;
				}
				printf("Wakeup from Sleep!!\n");
				break;

			default:
				break;
		}
		//TODO: Handle critical section access logic for SMP case in 8730E?
		//Is leave_critical_section required? In accordance with irqsave()^
#if ( configNUM_CORES > 1 )
		up_irq_enable();
#endif
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