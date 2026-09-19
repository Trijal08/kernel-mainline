#!/usr/bin/env python3
"""Compare or rebuild the zuma CMU suspend register lists against the vendor.

Source of truth: the Zuma PMU-calibration header from the Pixel 9 kernel tree,
    google-modules/soc/gs/drivers/soc/google/cal-if/zuma/flexpmu_cal_local_zuma.h
Each power domain has a struct pmucal_seq <pd>_save[] of
PMUCAL_SEQ_DESC(access_type, "SFR_NAME", base_pa, offset, mask, value, ...), and
only the PMUCAL_SAVE_RESTORE rows are carried across a domain power cycle. 24
domains have a save list; the other CMUs sit in domains that never power down,
and a clock-gated register keeps its value, so the vendor saves nothing for them.

This is the check that replaces rebooting to find a bad offset: an entry in a
save list is read by exynos_arm64_cmu_suspend() while its CMU is on the way
down, and an offset that does not exist there raises an SError and resets the
SoC with no post-mortem. The vendor rows are the hardware's own offsets.

  --check                every CMU in clk-zuma.c agrees with the vendor
  --diff   CMU_HSI0      one CMU: ours vs vendor, wrong offsets, missing defines
  --emit   CMU_HSI0      C source: the #defines we lack and the generated list

Usage: vendor-clk-saves.py <flexpmu_cal_local_zuma.h> <clk-zuma.c> [cmd [CMU_*]]
"""
import re
import sys

# our section -> (vendor pd name or None, cmu physical base)
CMUS = {
    "CMU_HSI0": ("hsi0", 0x11000000),
    "CMU_HSI1": ("hsi1", 0x12000000),
    "CMU_DPU": ("dpub", 0x19400000),
    "CMU_HSI2": (None, 0x13000000),
    "CMU_TOP": (None, 0x26040000),
    "CMU_APM": (None, 0x15400000),
    "CMU_MISC": (None, 0x10010000),
    "CMU_PERIC0": (None, 0x10800000),
    "CMU_PERIC1": (None, 0x10c00000),
}

SAVE_ROW = re.compile(r'PMUCAL_SEQ_DESC\(\s*PMUCAL_SAVE_RESTORE\s*,\s*"([^"]+)"'
                      r'\s*,\s*0x([0-9a-f]+)\s*,\s*0x([0-9a-f]+)')


def vendor_saves(header, pd, base):
    """[(sfr_name, offset)] that the firmware carries across <pd>'s power cycle."""
    text = open(header).read()
    m = re.search(r"struct pmucal_seq %s_save\[\] = \{(.*?)\n\};" % pd, text, re.S)
    if not m:
        sys.exit("no %s_save[] in %s" % (pd, header))
    return [(n, int(o, 16)) for n, b, o in SAVE_ROW.findall(m.group(1))
            if int(b, 16) == base]


def our_section(src, cmu):
    """(defines, [(offset, name)]) for one CMU_... section of clk-zuma.c."""
    text = open(src).read()
    i = text.index("/* ---- %s " % cmu)
    j = text.find("/* ---- CMU_", i + 10)
    sec = text[i:j if j > 0 else len(text)]
    defines = {}
    for m in re.finditer(r"^#define\s+(\w+)\s+0x([0-9a-f]+)\s*$", sec, re.M | re.I):
        defines.setdefault(m.group(1), int(m.group(2), 16))
    cur = []
    m = re.search(r"static const unsigned long \w+_clk_regs\[\][^=]*=\s*\{(.*?)\n\};",
                  sec, re.S)
    if m:
        cur = [(defines[w], w) for w in re.findall(r"\b[A-Z][A-Z0-9_]{5,}\b",
                                                   m.group(1)) if w in defines]
    return defines, cur


def diff(cmu, saves, defines, cur, out=print):
    mine = {o for o, _ in cur}
    theirs = {o for _, o in saves}
    out("%s: we save %d, vendor saves %d" % (cmu, len(mine), len(theirs)))
    for o, n in cur:
        if o not in theirs:
            out("  ours, not vendor: %#06x %s" % (o, n))
    for n, o in saves:
        if o not in mine:
            have = next((d for d, v in defines.items() if v == o), None)
            out("  vendor, not ours: %#06x %-52s %s"
                % (o, n, "= our " + have if have else "NO DEFINE"))
    for n, o in saves:
        if n in defines and defines[n] != o:
            out("  WRONG OFFSET: %s ours %#x vendor %#x" % (n, defines[n], o))
    return mine != theirs


def check(header, src):
    bad = 0
    for cmu, (pd, base) in sorted(CMUS.items()):
        saves = [] if pd is None else vendor_saves(header, pd, base)
        defines, cur = our_section(src, cmu)
        if not cur and not saves:
            print("%-11s no save list, none needed" % cmu)
            continue
        if diff(cmu, saves, defines, cur):
            bad += 1
    print("%d CMU(s) out of agreement with the vendor" % bad)
    return bad


def emit(cmu, saves, defines):
    pd = CMUS[cmu][0]
    new = [(n, o) for n, o in saves if n not in defines or defines[n] != o]
    for n, o in sorted(new, key=lambda x: x[1]):
        print("#define %-64s 0x%04x" % (n, o))
    if new:
        print()
    print("static const unsigned long %s_clk_regs[] __initconst = {" % pd)
    for n, _ in saves:
        print("\t%s," % n)
    print("};")


def main():
    header, src = sys.argv[1], sys.argv[2]
    cmd = sys.argv[3] if len(sys.argv) > 3 else "--check"
    if cmd == "--check":
        return 1 if check(header, src) else 0
    cmu = sys.argv[4]
    pd, base = CMUS[cmu]
    if pd is None:
        sys.exit("%s has no power domain and so no save list" % cmu)
    saves = vendor_saves(header, pd, base)
    defines, cur = our_section(src, cmu)
    if cmd == "--emit":
        emit(cmu, saves, defines)
    else:
        diff(cmu, saves, defines, cur)
    return 0


if __name__ == "__main__":
    sys.exit(main())
