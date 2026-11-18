/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2015 Cavium, Inc
 * Copyright(c) 2020 Arm Limited
 */

#ifndef _RTE_ATOMIC_ARM64_H_
#define _RTE_ATOMIC_ARM64_H_

#include "generic/rte_atomic.h"
#include <rte_branch_prediction.h>
#include <rte_debug.h>

#ifdef __cplusplus
extern "C" {
#endif

#define rte_mb() asm volatile("dmb osh" : : : "memory")

#define rte_wmb() asm volatile("dmb oshst" : : : "memory")

#define rte_rmb() asm volatile("dmb oshld" : : : "memory")

#define rte_io_mb() rte_mb()

#define rte_io_wmb() rte_wmb()

#define rte_io_rmb() rte_rmb()

static __rte_always_inline void
rte_atomic_thread_fence(rte_memory_order memorder)
{
	__rte_atomic_thread_fence(memorder);
}

/*------------------------ 128 bit atomic operations -------------------------*/

/*
 * Select one CAS backend at build time. An LSE target (for example,
 * -march=armv8.1-a or an LSE-capable DPDK platform) uses direct CASP.
 * A generic Armv8-A build uses libgcc's runtime dispatch with GCC 14+
 * and the inline LL/SC fallback with other toolchains.
 */
#if defined(__ARM_FEATURE_ATOMICS) || defined(RTE_ARM_FEATURE_ATOMICS)
#if defined(RTE_CC_CLANG)
#define __RTE_ARM64_CAS128_LSE_BUILTIN
/*
 * -march=armv8.1-a already enables LSE in Clang. If only the DPDK
 * platform flag guarantees LSE, tell Clang about it for this function.
 */
#ifndef __ARM_FEATURE_ATOMICS
#define __RTE_ARM64_CAS128_TARGET __rte_target("lse")
#endif
#else
#define __RTE_ARM64_CAS128_LSE_ASM
/* GCC can lower the 16-byte builtin to libatomic even on an LSE target. */
#define __ATOMIC128_CAS_OP(cas_op_name, op_string)                          \
static __rte_always_inline void                                             \
cas_op_name(rte_int128_t *dst, rte_int128_t *old, rte_int128_t updated)     \
{                                                                           \
	asm volatile(                                                       \
		op_string " %[old], %H[old], %[upd], %H[upd], [%[dst]]"     \
		: [old] "+r"(old->int128)                                   \
		: [upd] "r"(updated.int128), [dst] "r"(dst)                 \
		: "memory");                                                \
}

__ATOMIC128_CAS_OP(__cas_128_relaxed, "casp")
__ATOMIC128_CAS_OP(__cas_128_acquire, "caspa")
__ATOMIC128_CAS_OP(__cas_128_release, "caspl")
__ATOMIC128_CAS_OP(__cas_128_acq_rel, "caspal")

#undef __ATOMIC128_CAS_OP
#endif

#elif defined(RTE_CC_GCC) && GCC_VERSION >= 140000
#define __RTE_ARM64_CAS128_OUTLINE
/*
 * libgcc chooses CASP or LL/SC from the running CPU's capabilities.
 * Use GCC 14 or later: older helpers can return a torn 128-bit value
 * on an LL/SC mismatch, e.g. a stack head and counter from different updates.
 */
__extension__ extern __int128 __aarch64_cas16_relax(__int128, __int128, __int128 *);
__extension__ extern __int128 __aarch64_cas16_acq(__int128, __int128, __int128 *);
__extension__ extern __int128 __aarch64_cas16_rel(__int128, __int128, __int128 *);
__extension__ extern __int128 __aarch64_cas16_acq_rel(__int128, __int128, __int128 *);
#endif

#ifdef __RTE_ARM64_CAS128_TARGET
__RTE_ARM64_CAS128_TARGET
#endif
static inline int
rte_atomic128_cmp_exchange(rte_int128_t *dst, rte_int128_t *exp,
		const rte_int128_t *src, unsigned int weak, int success,
		int failure)
{
	/* Always do strong CAS */
	RTE_SET_USED(weak);
	/* Find invalid memory order */
	RTE_ASSERT(success == rte_memory_order_relaxed ||
		success == rte_memory_order_acquire ||
		success == rte_memory_order_release ||
		success == rte_memory_order_acq_rel ||
		success == rte_memory_order_seq_cst);

#ifdef __RTE_ARM64_CAS128_LSE_BUILTIN
	return __atomic_compare_exchange_n(&dst->int128, &exp->int128, src->int128, false,
			success, failure);
#else
	/* The assembly and outlined paths select instructions by success order. */
	RTE_SET_USED(failure);

	rte_int128_t expected = *exp;
	rte_int128_t desired = *src;
	rte_int128_t old;

#if defined(__RTE_ARM64_CAS128_LSE_ASM)
	if (success == rte_memory_order_relaxed)
		__cas_128_relaxed(dst, exp, desired);
	else if (success == rte_memory_order_acquire)
		__cas_128_acquire(dst, exp, desired);
	else if (success == rte_memory_order_release)
		__cas_128_release(dst, exp, desired);
	else
		__cas_128_acq_rel(dst, exp, desired);
	old = *exp;
#elif defined(__RTE_ARM64_CAS128_OUTLINE)
	if (success == rte_memory_order_relaxed)
		old.int128 = __aarch64_cas16_relax(expected.int128, desired.int128, &dst->int128);
	else if (success == rte_memory_order_acquire)
		old.int128 = __aarch64_cas16_acq(expected.int128, desired.int128, &dst->int128);
	else if (success == rte_memory_order_release)
		old.int128 = __aarch64_cas16_rel(expected.int128, desired.int128, &dst->int128);
	else
		old.int128 = __aarch64_cas16_acq_rel(expected.int128, desired.int128, &dst->int128);
	*exp = old;
#else
#define __HAS_ACQ(mo) ((mo) != rte_memory_order_relaxed && (mo) != rte_memory_order_release)
#define __HAS_RLS(mo) ((mo) == rte_memory_order_release || (mo) == rte_memory_order_acq_rel || \
		(mo) == rte_memory_order_seq_cst)

	int ldx_mo = __HAS_ACQ(success) ? rte_memory_order_acquire : rte_memory_order_relaxed;
	int stx_mo = __HAS_RLS(success) ? rte_memory_order_release : rte_memory_order_relaxed;

#undef __HAS_ACQ
#undef __HAS_RLS

	uint32_t ret = 1;

	/*
	 * A load-exclusive pair alone can return a torn value. On a mismatch,
	 * store the loaded pair back unchanged and retry if the store fails.
	 */
	do {

#define __LOAD_128(op_string, src, dst) { \
	asm volatile(                     \
		op_string " %0, %1, %2"   \
		: "=&r" (dst.val[0]),     \
		  "=&r" (dst.val[1])      \
		: "Q" (src->val[0])       \
		: "memory"); }

		if (ldx_mo == rte_memory_order_relaxed)
			__LOAD_128("ldxp", dst, old)
		else
			__LOAD_128("ldaxp", dst, old)

#undef __LOAD_128

#define __STORE_128(op_string, dst, src, ret) { \
	asm volatile(                           \
		op_string " %w0, %1, %2, %3"    \
		: "=&r" (ret)                   \
		: "r" (src.val[0]),             \
		  "r" (src.val[1]),             \
		  "Q" (dst->val[0])             \
		: "memory"); }

		if (likely(old.int128 == expected.int128)) {
			if (stx_mo == rte_memory_order_relaxed)
				__STORE_128("stxp", dst, desired, ret)
			else
				__STORE_128("stlxp", dst, desired, ret)
		} else {
			if (stx_mo == rte_memory_order_relaxed)
				__STORE_128("stxp", dst, old, ret)
			else
				__STORE_128("stlxp", dst, old, ret)
		}

#undef __STORE_128

	} while (unlikely(ret));

	/* Unconditionally updating the value of exp removes an 'if' statement.
	 * The value of exp should already be in register if not in the cache.
	 */
	*exp = old;
#endif

	return (old.int128 == expected.int128);
#endif
}

#undef __RTE_ARM64_CAS128_LSE_BUILTIN
#undef __RTE_ARM64_CAS128_LSE_ASM
#undef __RTE_ARM64_CAS128_OUTLINE
#undef __RTE_ARM64_CAS128_TARGET

#ifdef __cplusplus
}
#endif

#endif /* _RTE_ATOMIC_ARM64_H_ */
