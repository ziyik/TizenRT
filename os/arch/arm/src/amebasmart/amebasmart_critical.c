/****************************************************************************
 * arch/arm/src/amebasmart/amebasmart_critical.c
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
#include "amebasmart_critical.h"
#include "spinlock.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Masks all bits in the APSR other than the mode bits. */
#define portAPSR_MODE_BITS_MASK			( 0x1F )

/* The value of the mode bits in the APSR when the CPU is executing in user
mode. */
#define portAPSR_USER_MODE				( 0x10 )

/* The value of the mode bits in the APSR when the CPU is executing in user
mode. */
#define portAPSR_IRQ_MODE				( 0x12 )

/* Counts the interrupt nesting depth.  A context switch is only performed if
if the nesting depth is 0. */
volatile uint32_t ulPortInterruptNesting[ configNUM_CORES ] = { 0 };

/* Used for smp  */
spinlock_t task_lock;

/*-----------------------------------------------------------*/

void xPortSpinLockTask( void )
{
	spin_lock(&task_lock);
}
/*-----------------------------------------------------------*/

void xPortSpinUnLockTask( void )
{
	spin_unlock(&task_lock);
}
/*-----------------------------------------------------------*/

uint32_t ulPortInterruptLock(void)
{
uint32_t key;

	__asm volatile (	"mrs	%0, cpsr	\n": "=r" (key) :: "memory");
	portCPU_IRQ_DISABLE();

	return key;
}
/*-----------------------------------------------------------*/

void ulPortInterruptUnLock(uint32_t key)
{
	__asm volatile (	"msr	cpsr_c, %0	\n" :: "r" (key) : "memory");
}
/*-----------------------------------------------------------*/

#if configNUM_CORES > 1
BaseType_t xPortCpuIsInInterrupt(void)
{
	uint32_t cpsr;

	__asm volatile (	"mrs	%0, cpsr	\n" : "=r" (cpsr));

	return ((cpsr & portAPSR_MODE_BITS_MASK) == portAPSR_IRQ_MODE) || ulPortInterruptNesting[up_cpu_index()];
}
#endif