#include "arm.h"
#include "cp15.h"
#include "sctlr.h"

	.macro mov_imm _reg, _val
		.if ((\_val) & 0xffff0000) == 0
			mov	\_reg, #(\_val)
		.else
			movw	\_reg, #((\_val) & 0xffff)
			movt	\_reg, #((\_val) >> 16)
		.endif
	.endm

	.macro save_regs mode
		cps	\mode
		mrs	r2, spsr
		str	r2, [r0], #4
		str	sp, [r0], #4
		str	lr, [r0], #4
	.endm

	.macro restore_regs mode
		cps	\mode
		ldr	r2, [r0], #4
		ldr	sp, [r0], #4
		ldr	lr, [r0], #4
		msr	spsr_fsxc, r2
	.endm


.align 4
.global	cpu_suspend
.type cpu_suspend, %function
cpu_suspend:
	stmfd	sp!, {r4 - r11, lr}
	mov	r5, sp					@ current SP
	stmfd	sp!, {r0, r1}		@ save suspend func arg and pointer
	mov	r0, r5					@ SP
	bl	cpu_suspend_save
	ldmfd	sp!, {r0, pc}		@ call suspend fn


.align 4
.global	cpu_do_suspend
.type cpu_do_suspend, %function
cpu_do_suspend:
	stmfd	sp!, {r4 - r11, lr}
	cps #PSR_MODE_SVC
	mrc	CP15_FCSEIDR(r4)					@ save FCSE
	mrc	CP15_TPIDRURO(r5)					@ save TPIDR
	stmia	r0!, {r4 - r5}
	mrc	CP15_VBAR(r4)						@ save VBAR
	mrc	CP15_MAIR0(r6)						@ save MAIR0. LPAE is used so MAIR0 need to be saved.
	mrrc	p15, 0, r5, r7, c2				@ save TTBR0. TTBR1 is not used so not necessary to save it
	mrc	CP15_TTBCR(r11)						@ save TTBCR
	mrc	CP15_SCTLR(r8)						@ save SCTLR
	mrc	CP15_ACTLR(r9)						@ save ACTLR
	mrc	CP15_CPACR(r10)						@ save CPACR
	stmia	r0!, {r4 - r11}

	save_regs	#PSR_MODE_IRQ				@ save spsr, lr ,sp for irq, fiq, abt, und, and svc mode
	save_regs	#PSR_MODE_FIQ
	save_regs	#PSR_MODE_ABT
	save_regs	#PSR_MODE_UND
	save_regs	#PSR_MODE_SVC
	cps	#PSR_MODE_SYS

	ldmfd	sp!, {r4 - r11, pc}


.align 4
.global	cpu_resume
.type cpu_resume, %function
cpu_resume:
	ldr	r1, = ctx							@ get ctx addr directly because mmu is 1:1 mapping
	add	r0, r1, #0x4
	mov	ip, #0
	mcr CP15_ICIALLU(ip)					@ invalidate I cache
	mcr	CP15_CONTEXTIDR(ip)					@ set context id to 0
	ldmia	r0!, {r4 - r5}
	mcr	CP15_FCSEIDR(r4)					@ restore FCSE
	mcr	CP15_TPIDRURO(r5)					@ restore TPIDR
	ldmia	r0!, {r4 - r11}

	mcr	CP15_VBAR(r4)						@ restore VBAR
	mcr CP15_TLBIALL(ip, c7)				@ invalidate TLBs
	mcr	CP15_MAIR0(r6)						@ restore MAIR0
	mcrr	p15, 1, r1, ip, c2				@ set TTBR1 to 0 because it is not used
	mcrr	p15, 0, r5, r7, c2				@ restore TTBR0
	mcr	CP15_TTBCR(r11)						@ restore TTBCR
	mcr	CP15_ACTLR(r9)						@ restore ACTLR
	mcr	CP15_CPACR(r10)						@ restore CPACR

	isb
	dsb

	/* Need to find other method to write back SCTLR reg, mainly bit 0, 2, 12, or else will cause memory alignment issue at later stage */
	@ mcr CP15_SCTLR(r8)						@ restore SCTLR. cache and mmu will be enabled. it is safe because of 1:1 mapping

	restore_regs	#PSR_MODE_IRQ			@ restore spsr, lr, sp for irq, fiq, abt, und, svc mode.
	restore_regs	#PSR_MODE_FIQ
	restore_regs	#PSR_MODE_ABT
	restore_regs	#PSR_MODE_UND
	restore_regs	#PSR_MODE_SVC

	isb
	dsb

	cps	#PSR_MODE_SYS						@ tizenrt task is running in sys mode.
	ldr	r1, = ctx
	ldr	sp, [r1]

	isb
	mov r0, #0
	ldmfd	sp!, {r4 - r11, pc}				@ come back to where to sleep.