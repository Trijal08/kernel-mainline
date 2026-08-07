/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Force-included (via -include) mainline build compatibility shims for the
 * bcm4390 dhd driver. See the dhd-bcm4390-integration plan.
 */
#ifndef _MAINLINE_BCMDHD_COMPAT_H_
#define _MAINLINE_BCMDHD_COMPAT_H_

/*
 * NOTE(mainline): strncpy() was removed from the mainline kernel upstream
 * (both the <linux/string.h> prototype and the lib/string.c symbol are gone).
 * dhd still calls strncpy() across ~8 source files. Provide a standard
 * implementation and redirect calls to it via a macro so we don't collide with
 * the compiler's builtin recognition of the name "strncpy".
 */
static inline char *
__dhd_compat_strncpy(char *dest, const char *src, unsigned long count)
{
	char *ret = dest;

	while (count) {
		if ((*dest = *src) != '\0')
			src++;
		dest++;
		count--;
	}
	return ret;
}
#define strncpy(d, s, n) __dhd_compat_strncpy((d), (s), (n))

#endif /* _MAINLINE_BCMDHD_COMPAT_H_ */
