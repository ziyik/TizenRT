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
 * arch/arm/src/armv7-a/arm_schedulesigaction.c
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

#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/arch.h>

#include "arm.h"
#include "sched/sched.h"
#include "up_internal.h"
#include "irq/irq.h"
#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'sigdeliver' callback is executed on the thread
 *   specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked
 *   nor should it cause any immediate execution of sigdeliver.
 *   Typically, a few cases need to be considered:
 *
 *   (1) This function may be called from an interrupt handler
 *       During interrupt processing, all xcptcontext structures
 *       should be valid for all tasks.  That structure should
 *       be modified to invoke sigdeliver() either on return
 *       from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT
 *       the currently executing task, then again just modify
 *       the saved xcptcontext structure for the recipient
 *       task so it will invoke sigdeliver when that task is
 *       later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the
 *       currently executing task -- just call the signal
 *       handler now.
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
	irqstate_t saved_state;

	svdbg("tcb=%p sigdeliver=%p\n", tcb, sigdeliver);

	saved_state = enter_critical_section();

	/* Refuse to handle nested signal actions */

	if (!tcb->xcp.sigdeliver) {
		/* First, handle some special cases when the signal is being delivered
		 * to task that is currently executing on this CPU.
		 */

		svdbg("rtcb=%p CURRENT_REGS=%p\n", this_task(), CURRENT_REGS);

		if (tcb == this_task()) {
			/* CASE 1:  We are not in an interrupt handler and a task is
			 * signaling itself for some reason.
			 */

			if (!CURRENT_REGS) {
				/* In this case just deliver the signal now.
				 * REVISIT:  Signal handler will run in a critical section!
				 */

				sigdeliver(tcb);
			}

			/* CASE 2:  We are in an interrupt handler AND the interrupted
			 * task is the same as the one that must receive the signal, then
			 * we will have to modify the return state as well as the state
			 * in the TCB.
			 *
			 * Hmmm... there looks like a latent bug here: The following logic
			 * would fail in the strange case where we are in an interrupt
			 * handler, the thread is signaling itself, but a context switch
			 * to another task has occurred so that CURRENT_REGS does not
			 * refer to the thread of this_task()!
			 */

			else {
				/* Save the return lr and cpsr and one scratch register
				 * These will be restored by the signal trampoline after
				 * the signals have been delivered.
				 */

				tcb->xcp.sigdeliver = sigdeliver;

				/* And make sure that the saved context in the TCB is the same
				 * as the interrupt return context.
				 */

				arm_savestate(tcb->xcp.saved_regs);

				/* Duplicate the register context.  These will be
				 * restored by the signal trampoline after the signal has been
				 * delivered.
				 */

				CURRENT_REGS = (void *)
										((uint32_t)CURRENT_REGS - (uint32_t)XCPTCONTEXT_SIZE);
				memcpy((uint32_t *)CURRENT_REGS, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);

				CURRENT_REGS[REG_SP] = (uint32_t)CURRENT_REGS + (uint32_t)XCPTCONTEXT_SIZE;

				/* Then set up to vector to the trampoline with interrupts
				 * disabled
				 */

				CURRENT_REGS[REG_PC] = (uint32_t)arm_sigdeliver;
				CURRENT_REGS[REG_CPSR] = (PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT);
#ifdef CONFIG_ARM_THUMB
				CURRENT_REGS[REG_CPSR] |= PSR_T_BIT;
#endif
			}
		}

		/* Otherwise, we are (1) signaling a task is not running from an
		 * interrupt handler or (2) we are not in an interrupt handler and the
		 * running task is signaling some other non-running task.
		 */

		else {
			/* Save the return lr and cpsr and one scratch register.  These
			 * will be restored by the signal trampoline after the signals
			 * have been delivered.
			 */

			tcb->xcp.sigdeliver = sigdeliver;

			/* Save the current register context location */

			tcb->xcp.saved_regs = tcb->xcp.regs;

			/* Duplicate the register context.  These will be
			 * restored by the signal trampoline after the signal has been
			 * delivered.
			 */

			tcb->xcp.regs = (void *)
									((uint32_t)tcb->xcp.regs - (uint32_t)XCPTCONTEXT_SIZE);
			memcpy(tcb->xcp.regs, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);

			tcb->xcp.regs[REG_SP] = (uint32_t)tcb->xcp.regs + (uint32_t)XCPTCONTEXT_SIZE;

			/* Then set up to vector to the trampoline with interrupts
			 * disabled
			 */

			tcb->xcp.regs[REG_PC] = (uint32_t)arm_sigdeliver;
			tcb->xcp.regs[REG_CPSR] = (PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT);
#ifdef CONFIG_ARM_THUMB
			tcb->xcp.regs[REG_CPSR] |= PSR_T_BIT;
#endif
		}
	}

	leave_critical_section(saved_state);

}
#endif							/* !CONFIG_SMP */

#ifdef CONFIG_SMP
void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
	int cpu;
	int me;
	irqstate_t saved_state;

	svdbg("tcb=%p sigdeliver=%p\n", tcb, sigdeliver);

	saved_state = enter_critical_section();

	/* Refuse to handle nested signal actions */

	if (!tcb->xcp.sigdeliver) {
		/* First, handle some special cases when the signal is being delivered
		 * to task that is currently executing on any CPU.
		 */

		svdbg("rtcb=%p CURRENT_REGS=%p\n", this_task(), CURRENT_REGS);

		if (tcb->task_state == TSTATE_TASK_RUNNING) {
			me = this_cpu();
			cpu = tcb->cpu;

			/* CASE 1:  We are not in an interrupt handler and a task is
			 * signaling itself for some reason.
			 */

			if (cpu == me && !CURRENT_REGS) {
				/* In this case just deliver the signal now.
				 * REVISIT:  Signal handler will run in a critical section!
				 */

				sigdeliver(tcb);
			}

			/* CASE 2:  The task that needs to receive the signal is running.
			 * This could happen if the task is running on another CPU OR if
			 * we are in an interrupt handler and the task is running on this
			 * CPU.  In the former case, we will have to PAUSE the other CPU
			 * first.  But in either case, we will have to modify the return
			 * state as well as the state in the TCB.
			 */

			else {
				/* If we signaling a task running on the other CPU, we have
				 * to PAUSE the other CPU.
				 */

				if (cpu != me) {
					/* Pause the CPU */

					up_cpu_pause(cpu);

					/* Now tcb on the other CPU can be accessed safely */

					/* Copy tcb->xcp.regs to tcp.xcp.saved. These will be
					 * restored by the signal trampoline after the signal has
					 * been delivered.
					 */

					tcb->xcp.sigdeliver = sigdeliver;

					/* Save the current register context location */

					tcb->xcp.saved_regs = tcb->xcp.regs;

					/* Duplicate the register context.  These will be
					 * restored by the signal trampoline after the signal has
					 * been delivered.
					 */

					tcb->xcp.regs = (void *)
									((uint32_t)tcb->xcp.regs - (uint32_t)XCPTCONTEXT_SIZE);
					memcpy(tcb->xcp.regs, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);

					tcb->xcp.regs[REG_SP] = (uint32_t)tcb->xcp.regs + (uint32_t)XCPTCONTEXT_SIZE;

					/* Then set up to vector to the trampoline with interrupts
					 * disabled
					 */

					tcb->xcp.regs[REG_PC] = (uint32_t)arm_sigdeliver;
					tcb->xcp.regs[REG_CPSR] = (PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT);
#ifdef CONFIG_ARM_THUMB
					tcb->xcp.regs[REG_CPSR] |= PSR_T_BIT;
#endif
				} else {
					/* tcb is running on the same CPU */

					/* Save the return PC, CPSR and either the BASEPRI or
					 * PRIMASK registers (and perhaps also the LR).  These will
					 * be restored by the signal trampoline after the signal
					 * has been delivered.
					 */

					tcb->xcp.sigdeliver = (void *)sigdeliver;

					/* And make sure that the saved context in the TCB is the
					 * same as the interrupt return context.
					 */

					arm_savestate(tcb->xcp.saved_regs);

					/* Duplicate the register context.  These will be
					 * restored by the signal trampoline after the signal has
					 * been delivered.
					 */

					CURRENT_REGS = (void *)
								   ((uint32_t)CURRENT_REGS - (uint32_t)XCPTCONTEXT_SIZE);
					memcpy((uint32_t *) CURRENT_REGS, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);

					CURRENT_REGS[REG_SP] = (uint32_t)CURRENT_REGS + (uint32_t)XCPTCONTEXT_SIZE;

					/* Then set up vector to the trampoline with interrupts
					 * disabled.  The kernel-space trampoline must run in
					 * privileged thread mode.
					 */

					CURRENT_REGS[REG_PC] = (uint32_t)arm_sigdeliver;
					CURRENT_REGS[REG_CPSR] = (PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT);
#ifdef CONFIG_ARM_THUMB
					CURRENT_REGS[REG_CPSR] |= PSR_T_BIT;
#endif
				}

				/* Increment the IRQ lock count so that when the task is
				 * restarted, it will hold the IRQ spinlock.
				 */

				DEBUGASSERT(tcb->irqcount < INT16_MAX);
				tcb->irqcount++;

				/* NOTE: If the task runs on another CPU(cpu), adjusting
				 * global IRQ controls will be done in the pause handler
				 * on the CPU(cpu) by taking a critical section.
				 * If the task is scheduled on this CPU(me), do nothing
				 * because this CPU already took a critical section
				 */

				/* RESUME the other CPU if it was PAUSED */

				if (cpu != me) {
					up_cpu_resume(cpu);
				}
			}
		}

		/* Otherwise, we are (1) signaling a task is not running from an
		 * interrupt handler or (2) we are not in an interrupt handler and the
		 * running task is signaling some other non-running task.
		 */

		else {
			/* Save the return lr and cpsr and one scratch register.  These
			 * will be restored by the signal trampoline after the signals
			 * have been delivered.
			 */

			tcb->xcp.sigdeliver = sigdeliver;

			/* Save the current register context location */

			tcb->xcp.saved_regs = tcb->xcp.regs;

			/* Duplicate the register context.  These will be
			 * restored by the signal trampoline after the signal has been
			 * delivered.
			 */

			tcb->xcp.regs = (void *)
							((uint32_t)tcb->xcp.regs - (uint32_t)XCPTCONTEXT_SIZE);
			memcpy(tcb->xcp.regs, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);

			tcb->xcp.regs[REG_SP] = (uint32_t)tcb->xcp.regs + (uint32_t)XCPTCONTEXT_SIZE;

			/* Increment the IRQ lock count so that when the task is restarted,
			 * it will hold the IRQ spinlock.
			 */

			DEBUGASSERT(tcb->irqcount < INT16_MAX);
			tcb->irqcount++;

			/* Then set up to vector to the trampoline with interrupts
			 * disabled
			 */

			tcb->xcp.regs[REG_PC] = (uint32_t)arm_sigdeliver;
			tcb->xcp.regs[REG_CPSR] = (PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT);
#ifdef CONFIG_ARM_THUMB
			tcb->xcp.regs[REG_CPSR] |= PSR_T_BIT;
#endif
		}
	}

	leave_critical_section(saved_state);

}
#endif							/* CONFIG_SMP */

#endif							/* !CONFIG_DISABLE_SIGNALS */
