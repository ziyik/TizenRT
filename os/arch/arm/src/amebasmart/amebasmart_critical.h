/****************************************************************************
 *
 * Copyright 2022 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_AMEBASMART_CRITICAL_H
#define __ARCH_ARM_SRC_AMEBASMART_CRITICAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>


/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	uint32_t
#define portBASE_TYPE	long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

typedef uint32_t TickType_t;
#define portMAX_DELAY				( TickType_t ) 0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
not need to be guarded with a critical section. */
#define portTICK_TYPE_IS_ATOMIC		1

#define portCRITICAL_NESTING_IN_TCB	1

/*-----------------------------------------------------------*/

/* Hardware specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT			64


#define portYIELD() __asm volatile ( "SWI 0" ::: "memory" );

/* The critical section macros only mask interrupts up to an application
determined priority level.  Sometimes it is necessary to turn interrupt off in
the CPU itself before modifying certain hardware registers. */
#define portCPU_IRQ_DISABLE()								\
	__asm volatile ( "CPSID i" ::: "memory" );				\
	__asm volatile ( "DSB" );								\
	__asm volatile ( "ISB" );

#define portCPU_IRQ_ENABLE()								\
	__asm volatile ( "CPSIE i" ::: "memory" );				\
	__asm volatile ( "DSB" );								\
	__asm volatile ( "ISB" );


/*-----------------------------------------------------------
 * Critical section control
 *----------------------------------------------------------*/
// extern void vTaskEnterCritical( void );
// extern void vTaskExitCritical( void );

// extern void xPortSpinLockTask( void );
// extern void xPortSpinUnLockTask( void );
// extern void vPortEnterCritical( void );
// extern void vPortExitCritical( void );
// extern void vPortInstallFreeRTOSVectorTable( void );

// uint32_t ulPortInterruptLock(void);
// void ulPortInterruptUnLock(uint32_t key);

BaseType_t xPortCpuIsInInterrupt(void);

// extern int pmu_secondary_cpu_state_is_running(uint32_t CoreID);
// extern int pmu_secondary_cpu_state_is_hotplug(uint32_t CoreID);

// Remember to come back and this part (enter/exit critical zone for single/dual core)
// For task lock, refer to spinlock.h
#if ( configNUM_CORES == 1 )
#define portGET_TASK_LOCK()
#define portRELEASE_TASK_LOCK()
#define portENTER_CRITICAL()		irqsave()
#define portEXIT_CRITICAL(x)		irqrestore(x)
#else
#define portGET_TASK_LOCK()			xPortSpinLockTask()
#define portRELEASE_TASK_LOCK()		xPortSpinUnLockTask()
#define portENTER_CRITICAL()		irqsave()
#define portEXIT_CRITICAL(x)		irqrestore(x)
#endif

#define portDISABLE_INTERRUPTS()	ulPortInterruptLock()
#define portRESTORE_INTERRUPTS(x)	ulPortInterruptUnLock(x)
#define portENABLE_INTERRUPTS()		portCPU_IRQ_ENABLE()
#define portSET_INTERRUPT_MASK_FROM_ISR()		ulPortInterruptLock()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	ulPortInterruptUnLock(x)

#define portCHECK_IF_IN_ISR()		xPortCpuIsInInterrupt()

/*-----------------------------------------------------------*/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif							/* __ASSEMBLY__ */
#endif							/* __ARCH_ARM_SRC_AMEBASMART_CRITICAL_H */
