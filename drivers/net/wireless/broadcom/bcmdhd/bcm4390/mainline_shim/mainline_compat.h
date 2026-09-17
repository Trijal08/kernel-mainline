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

/*
 * NOTE(mainline): strlcpy() was removed upstream too. Its remaining callers sit
 * in DHD_DEBUG-only code (dhd_linux_exportfs.c, dhd_debug.c, dhd_cfg80211.c), so
 * they only start failing to build once DHD_DEBUG is defined. Same macro trick
 * as strncpy above, and open-coded for the same reason this header is
 * force-included: <linux/string.h> has not necessarily been seen yet, so
 * strlen()/memcpy() cannot be called here.
 *
 * Keeps the BSD return value (the length the result would have had), which is
 * what distinguishes it from strscpy() and what the callers' truncation checks
 * expect.
 *
 * Do NOT add a strlcat() shim next to this one: unlike strlcpy(), strlcat() is
 * still declared in <linux/string.h>, and a function-like macro of that name
 * rewrites its own prototype there into a syntax error.
 */
static inline unsigned long
__dhd_compat_strlcpy(char *dest, const char *src, unsigned long size)
{
	unsigned long srclen = 0;

	while (src[srclen] != '\0')
		srclen++;

	if (size) {
		unsigned long len = (srclen >= size) ? size - 1 : srclen;
		unsigned long i;

		for (i = 0; i < len; i++)
			dest[i] = src[i];
		dest[len] = '\0';
	}
	return srclen;
}
#define strlcpy(d, s, n) __dhd_compat_strlcpy((d), (s), (n))

#endif /* _MAINLINE_BCMDHD_COMPAT_H_ */
