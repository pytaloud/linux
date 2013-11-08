/*
 * arch/arm/mach-exynos/edcs.c - exynos dual cluster power management support
 *
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 * Author: Tarek Dakhran <t.dakhran @ samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * EDCS(exynos dual cluster support) for Exynos5410 SoC.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/mcpm.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/cp15.h>

#include <linux/arm-cci.h>
#include <mach/regs-pmu.h>

#define EDCS_CPUS_PER_CLUSTER	4
#define EDCS_CLUSTERS		2

/* Exynos5410 power management registers */
#define EDCS_CORE_CONFIGURATION(_nr)	(S5P_ARM_CORE0_CONFIGURATION	\
						+ ((_nr) * 0x80))
#define EDCS_CORE_STATUS(_nr)		(EDCS_CORE_CONFIGURATION(_nr) + 0x4)
#define EDCS_CORE_OPTION(_nr)		(EDCS_CORE_CONFIGURATION(_nr) + 0x8)

#define REG_CPU_STATE_ADDR0		(S5P_VA_SYSRAM_NS + 0x28)
#define REG_CPU_STATE_ADDR(_nr)		(REG_CPU_STATE_ADDR0 +	\
						 (_nr) * EDCS_CPUS_PER_CLUSTER)

#define SECONDARY_RESET		(1 << 1)
#define REG_ENTRY_ADDR		(S5P_VA_SYSRAM_NS + 0x1c)

static arch_spinlock_t edcs_lock = __ARCH_SPIN_LOCK_UNLOCKED;

static int edcs_use_count[EDCS_CPUS_PER_CLUSTER][EDCS_CLUSTERS];
static int core_count[EDCS_CLUSTERS];

static void exynos_core_power_control(unsigned int cpu, unsigned int cluster,
				bool enable)
{
	unsigned int offset = cluster * MAX_CPUS_PER_CLUSTER + cpu;
	int value = enable ? S5P_CORE_LOCAL_PWR_EN : 0;

	if ((readl_relaxed(EDCS_CORE_STATUS(offset)) & 0x3) != value) {
		wmb();
		writel_relaxed(value, EDCS_CORE_CONFIGURATION(offset));
	}
}

static void exynos_core_power_up(unsigned int cpu, unsigned int cluster)
{
	exynos_core_power_control(cpu, cluster, true);
}

static void exynos_core_power_down(unsigned int cpu, unsigned int cluster)
{
	exynos_core_power_control(cpu, cluster, false);
}

void set_boot_flag(unsigned int cpu, unsigned int mode)
{
	writel_relaxed(mode, REG_CPU_STATE_ADDR(cpu));
}

static int exynos_power_up(unsigned int cpu, unsigned int cluster)
{
	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cpu >= EDCS_CPUS_PER_CLUSTER || cluster >= EDCS_CLUSTERS);

	local_irq_disable();
	arch_spin_lock(&edcs_lock);

	edcs_use_count[cpu][cluster]++;
	if (edcs_use_count[cpu][cluster] == 1) {
		++core_count[cluster];
		set_boot_flag(cpu, SECONDARY_RESET);
		exynos_core_power_up(cpu, cluster);
	} else if (edcs_use_count[cpu][cluster] != 2) {
		/*
		 * The only possible values are:
		 * 0 = CPU down
		 * 1 = CPU (still) up
		 * 2 = CPU requested to be up before it had a chance
		 *     to actually make itself down.
		 * Any other value is a bug.
		 */
		BUG();
	}

	arch_spin_unlock(&edcs_lock);
	local_irq_enable();

	return 0;
}
static void exynos_power_down(void)
{
	unsigned int mpidr, cpu, cluster;
	bool last_man = false, skip_wfi = false;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: CORE%d on CLUSTER %d\n", __func__, cpu, cluster);
	BUG_ON(cpu >= EDCS_CPUS_PER_CLUSTER  || cluster >= EDCS_CLUSTERS);

	__mcpm_cpu_going_down(cpu, cluster);

	arch_spin_lock(&edcs_lock);
	BUG_ON(__mcpm_cluster_state(cluster) != CLUSTER_UP);
	edcs_use_count[cpu][cluster]--;
	if (edcs_use_count[cpu][cluster] == 0) {
		--core_count[cluster];
		if (core_count[cluster] == 0)
			last_man = true;
	} else if (edcs_use_count[cpu][cluster] == 1) {
		/*
		 * A power_up request went ahead of us.
		 * Even if we do not want to shut this CPU down,
		 * the caller expects a certain state as if the WFI
		 * was aborted.  So let's continue with cache cleaning.
		 */
		skip_wfi = true;
	} else
		BUG();

	if (!skip_wfi)
		gic_cpu_if_down();

	if (last_man && __mcpm_outbound_enter_critical(cpu, cluster)) {
		arch_spin_unlock(&edcs_lock);

		if (read_cpuid_part_number() == ARM_CPU_PART_CORTEX_A15) {
			/*
			 * On the Cortex-A15 we need to disable
			 * L2 prefetching before flushing the cache.
			 */
			asm volatile(
			"mcr	p15, 1, %0, c15, c0, 3\n\t"
			"isb\n\t"
			"dsb"
			: : "r" (0x400));
		}

		/*
		 * We need to disable and flush the whole (L1 and L2) cache.
		 * Let's do it in the safest possible way i.e. with
		 * no memory access within the following sequence
		 * including the stack.
		 *
		 * Note: fp is preserved to the stack explicitly prior doing
		 * this since adding it to the clobber list is incompatible
		 * with having CONFIG_FRAME_POINTER=y.
		 */
		asm volatile(
		"str	fp, [sp, #-4]!\n\t"
		"mrc	p15, 0, r0, c1, c0, 0	 @  get CR\n\t"
		"bic	r0, r0, #"__stringify(CR_C)"\n\t"
		"mcr	p15, 0, r0, c1, c0, 0	 @  set CR\n\t"
		"isb\n\t"
		"bl	v7_flush_dcache_all\n\t"
		"clrex\n\t"
		"mrc	p15, 0, r0, c1, c0, 1	 @  get AUXCR\n\t"
		"bic	r0, r0, #(1 << 6)	 @  disable local coherency\n\t"
		"mcr	p15, 0, r0, c1, c0, 1	 @  set AUXCR\n\t"
		"isb\n\t"
		"dsb\n\t"
		"ldr	fp, [sp], #4"
		: : : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
			"r9", "r10", "lr", "memory");

		cci_disable_port_by_cpu(mpidr);

		__mcpm_outbound_leave_critical(cluster, CLUSTER_DOWN);

	} else {
		arch_spin_unlock(&edcs_lock);
		/*
			* We need to disable and flush only the L1 cache.
			* Let's do it in the safest possible way as above.
		*/
		asm volatile(
		"str	fp, [sp, #-4]!\n\t"
		"mrc	p15, 0, r0, c1, c0, 0	 @  get CR\n\t"
		"bic	r0, r0, #"__stringify(CR_C)"\n\t"
		"mcr	p15, 0, r0, c1, c0, 0	 @  set CR\n\t"
		"isb\n\t"
		"bl	v7_flush_dcache_louis\n\t"
		"clrex\n\t"
		"mrc	p15, 0, r0, c1, c0, 1	 @  get AUXCR\n\t"
		"bic	r0, r0, #(1 << 6)	 @  disable local coherency\n\t"
		"mcr	p15, 0, r0, c1, c0, 1	 @  set AUXCR\n\t"
		"isb\n\t"
		"dsb\n\t"
		"ldr	fp, [sp], #4"
		: : : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
		      "r9", "r10", "lr", "memory");

	}
	__mcpm_cpu_down(cpu, cluster);

	if (!skip_wfi) {
		exynos_core_power_down(cpu, cluster);
		wfi();
	}
}

static const struct mcpm_platform_ops exynos_power_ops = {
	.power_up	= exynos_power_up,
	.power_down	= exynos_power_down,
};

static void __init edcs_data_init(void)
{
	unsigned int mpidr, cpu, cluster;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cpu >= EDCS_CPUS_PER_CLUSTER  || cluster >= EDCS_CLUSTERS);
	edcs_use_count[cpu][cluster] = 1;
	++core_count[cluster];
}

/*
 * Enable cluster-level coherency, in preparation for turning on the MMU.
 */
static void __naked edcs_power_up_setup(unsigned int affinity_level)
{
	asm volatile ("\n"
	"b	cci_enable_port_for_self");
}

static int __init edcs_init(void)
{
	int ret;
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "samsung,exynos5410");
	if (!node)
		return -ENODEV;

	if (!cci_probed())
		return -ENODEV;

	/*
	 * Future entries into the kernel can now go
	 * through the cluster entry vectors.
	 */
	writel_relaxed(virt_to_phys(mcpm_entry_point), REG_ENTRY_ADDR);

	edcs_data_init();
	mcpm_smp_set_ops();

	ret = mcpm_platform_register(&exynos_power_ops);
	if (!ret) {
		mcpm_sync_init(edcs_power_up_setup);
		pr_info("EDCS power management initialized\n");
	}
	return ret;
}

early_initcall(edcs_init);
