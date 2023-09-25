#ifndef CONFIG_PLATFORM_TIZENRT_OS
#include "FreeRTOS.h"
#include <task.h>
#include "freertos_pmu.h"
#endif
// #include "platform_autoconf.h"
// #include "ameba_tizenrt_pmu.h"
#include "sheipa.h"
#include "ameba_soc.h"
#include "sys_io.h"
#include "gic.h"


#define ROUND_UP(divider, divisor) (divider%divisor) ? ((divider/divisor)+1) : (divider/divisor)


typedef struct {
	u32 intr_enable[ROUND_UP(GIC_MAX_NUM_INTR, 32)];
	u32 intr_active[ROUND_UP(GIC_MAX_NUM_INTR, 32)];
	u32 intr_config[ROUND_UP(GIC_MAX_NUM_INTR, 16)];
	u32 intr_target[ROUND_UP(GIC_MAX_NUM_INTR, 4)];
	u32 intr_pri[ROUND_UP(GIC_MAX_NUM_INTR, 4)];
	u32 primask;
} saved_gic_regs;


typedef struct {
	/* sp for sys mode. sys_lr has been saved by stmfd*/
	u32 sp;
	/*	saved regsters:
		FCSE
		TPDIR
		VBAR
		TTBR0(low 32bit)
		MAIR0
		TTBR0(high 32bit)
		TTBCR
		SCTLR
		ACTLR
		CPACR
		IRQ_SPSR
		IRQ_SP
		IRQ_LR
		FIQ_SPSR
		FIQ_SP
		FIQ_LR
		ABT_SPSR
		ABT_SP
		ABT_LR
		UND_SPSR
		UND_SP
		UND_LR
		SVC_SPSR
		SVC_SP
		SVC_LR
	*/
	u32 suspend_regs[30];
} ns_pm_ctx;


ns_pm_ctx ctx;
saved_gic_regs gic_regs;
extern SLEEP_ParamDef sleep_param;
extern void cpu_suspend(unsigned long arg, int (*fn)(unsigned long));
extern void cpu_resume(void);
extern void cpu_do_suspend(u32 *regs);


void cpu_suspend_save(u32 sp)
{
	ctx.sp = sp;
	cpu_do_suspend(ctx.suspend_regs);

	DCache_CleanInvalidate(0xffffffff, 0xffffffff);
}


int SOCPS_PG_Enter(unsigned long arg)
{
	UNUSED(arg);

	psci_cpu_suspend(0x01010022, (unsigned long) cpu_resume);

	while (1) {
		DelayMs(1000);
		DBG_8195A("Error! Should never reach here!\n");
	}
}


void SOCPS_Save_GIC(void)
{
	int i;

	gic_regs.primask = sys_read32(GIC_ICCPMR);

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 32)); i++) {
		gic_regs.intr_enable[i] = sys_read32(GIC_ICDISER(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 32)); i++) {
		gic_regs.intr_active[i] = sys_read32(GIC_ICDSAR(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 16)); i++) {
		gic_regs.intr_config[i] = sys_read32(GIC_ICDICFR(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 4)); i++) {
		gic_regs.intr_target[i] = sys_read32(GIC_ICDIPTR(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 4)); i++) {
		gic_regs.intr_pri[i] = sys_read32(GIC_ICDIPR(i * 4));
	}
}


void SOCPS_Restore_GIC(void)
{
	int i;
	u32 val;

	sys_write32(gic_regs.primask, GIC_ICCPMR);

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 32)); i++) {
		sys_write32(gic_regs.intr_enable[i], GIC_ICDISER(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 32)); i++) {
		sys_write32(gic_regs.intr_active[i], GIC_ICDSAR(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 16)); i++) {
		sys_write32(gic_regs.intr_config[i], GIC_ICDICFR(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 4)); i++) {
		sys_write32(gic_regs.intr_target[i], GIC_ICDIPTR(i * 4));
	}

	for (i = 0; i < (ROUND_UP(GIC_MAX_NUM_INTR, 4)); i++) {
		sys_write32(gic_regs.intr_pri[i], GIC_ICDIPR(i * 4));
	}

	val = sys_read32(GIC_ICCICR);
	val &= ~(GIC_ICCICRS_FIQBYPDISGRP0 | GIC_ICCICRS_IRQBYPDISGRP0 | GIC_ICCICRS_FIQBYPDISGRP1 | GIC_ICCICRS_IRQBYPDISGRP1);
	val &= ~(GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1);
	val |= GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1;
	val |= GIC_ICCICRS_ACKTCTL;

	sys_write32(val, GIC_ICCICR);
	sys_write32((GIC_ICCICRS_ENABLEGRP0 | GIC_ICCICRS_ENABLEGRP1), GIC_ICDDCR);

}


void SOCPS_SleepPG(void)
{
	u32 nDeviceIdOffset = 0;
	/* exec sleep hook functions */
	nDeviceIdOffset = pmu_exec_sleep_hook_funs();
	if (nDeviceIdOffset != PMU_MAX) {
		pmu_exec_wakeup_hook_funs(nDeviceIdOffset);

		if (tickless_debug) {
			DBG_8195A("DBG: CA32 Sleep PG blocked because Dev %x  busy\n", nDeviceIdOffset);
		}
		return;
	}
	printf("pg-s\n");

	//save gic registers
	SOCPS_Save_GIC();

	HAL_WRITE8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW,
			   HAL_READ8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW) & (~ LSYS_BIT_AP_RUNNING));

	cpu_suspend(0, SOCPS_PG_Enter);

	//restore gic registers
	SOCPS_Restore_GIC();

	printf("pg-w\n");
	/* exec sleep hook functions */
	pmu_exec_wakeup_hook_funs(PMU_MAX);

#if ( configNUM_CORES > 1 )
	/* Now init core1 */
	smp_init();
	pmu_set_secondary_cpu_state(1, CPU1_WAKE_FROM_PG);
#endif
	//pmu_acquire_wakelock(PMU_OS);
}

void SOCPS_SleepCG(void)
{
	u32 nDeviceIdOffset = 0;

	/* exec sleep hook functions */
	nDeviceIdOffset = pmu_exec_sleep_hook_funs();
	if (nDeviceIdOffset != PMU_MAX) {
		pmu_exec_wakeup_hook_funs(nDeviceIdOffset);

		if (tickless_debug) {
			DBG_8195A("DBG: CA32 Sleep CG blocked because Dev %x  busy\n", nDeviceIdOffset);
		}
		return;
	}

	sleep_param.sleep_type = SLEEP_CG;
	printf("cg-s\n");
	HAL_WRITE8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW,
			   HAL_READ8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW) & (~ LSYS_BIT_AP_RUNNING));

	/* for AP outstanding, make sure AP is idle before disable clk */
	__asm volatile("isb");
	ipc_send_message(IPC_AP_TO_LP, IPC_A2L_TICKLESS_INDICATION, (PIPC_MSG_STRUCT)&sleep_param);
	HAL_WRITE32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_CKE, HAL_READ32(SYSTEM_CTRL_BASE_HP, REG_HSYS_HP_CKE) & (~HSYS_BIT_CKE_AP));
	DelayUs(2);
	//asm volatile("wfi");

	//while ((HAL_READ8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW) & (LSYS_BIT_AP_RUNNING)) == 0);
	printf("cg-w\n");

	/* exec sleep hook functions */
	pmu_exec_wakeup_hook_funs(PMU_MAX);
	//pmu_acquire_wakelock(PMU_OS);
}


