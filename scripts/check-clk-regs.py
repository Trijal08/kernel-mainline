#!/usr/bin/env python3
"""Audit the zuma CMU suspend register lists against the driver's own clock model.

Every entry in a *_clk_regs[] list is readl() by exynos_arm64_cmu_suspend() with
that CMU's domain on its way down. The lists were transcribed from a vendor
register dump that is demonstrably wrong -- CMU_HSI0 alone has seven gate
offsets claimed by two different names -- and a bogus offset is an SError that
resets the SoC with no post-mortem, so each one has so far cost a reboot.

The model is the trustworthy side: a mux, divider or PLL register that a MUX()
DIV() or PLL() entry references is read and often written at probe, so its
offset is proven good on live hardware. A gate register is weaker -- with
auto_clock_gate its enable and disable are nops and only the gate-debug alias at
offset + 0x4000 is ever read -- so gates are counted as modelled but are not
proof. Anything in the list that not even the model names is a bare guess.

Reports, does not enforce: not every CMU follows the rule yet, and in
auto_clock_gate mode nobody writes the gate registers, so "model offset missing
from the list" is a note rather than a finding.

  offsets the model uses twice          -> mis-transcription
  save entries no model clock touches   -> unverified, SError suspects
  model offsets missing from the save   -> lost across a power cycle

Usage: check-clk-regs.py [clk-zuma.c ...]
"""
import re
import sys
from collections import OrderedDict

SECTION = re.compile(r"^/\* ---- (CMU_\w+)", re.M)
DEFINE = re.compile(r"^#define\s+(\w+)\s+(\(?\s*0x[0-9a-fA-F]+\s*\)?)\s*$", re.M)
LIST = re.compile(r"^static const unsigned long (\w+)\[\]", re.M)
MODEL = re.compile(r"^static const struct samsung_\w+clock (\w+)_(?:mux|div|gate|pll|fixed)", re.M)


def sections(text):
    """Yield (cmu-name, body) split on the '---- CMU_X' banner comments."""
    name, buf = None, []
    for line in text.splitlines():
        m = SECTION.match(line)
        if m:
            if name:
                yield name, "\n".join(buf)
            name, buf = m.group(1), []
        if name:
            buf.append(line)
    if name:
        yield name, "\n".join(buf)


def names_in(body, pattern):
    """Register names used by every array whose header matches pattern."""
    out = []
    for m in pattern.finditer(body):
        start = body.index("{", m.end())
        end = body.find("};", start)
        out += re.findall(r"\b[A-Z][A-Z0-9_]{5,}\b", body[start:end])
    return out


def audit(path):
    # line continuations carry register names onto their own line
    text = re.sub(r"\\\n\s*", " ", open(path).read())
    total = 0
    for cmu, body in sections(text):
        regs = {}
        for m in DEFINE.finditer(body):
            regs.setdefault(m.group(1), int(m.group(2).strip("() "), 0))
        save = OrderedDict()
        for m in LIST.finditer(body):
            head = body.index("{", m.end())
            entries = re.findall(r"\b\w+\b",
                                 body[head:body.find("};", head)])
            for e in entries:
                if e in regs:
                    save.setdefault(regs[e], []).append(e)
        if not save:
            continue
        model = {regs[n] for n in names_in(body, MODEL) if n in regs}
        option = set()
        for m in re.finditer(r"\.option_offset\s*=\s*(\w+)", body):
            if m.group(1) in regs:
                option.add(regs[m.group(1)])

        print(f"{cmu}: save {len(save)}, model {len(model)}")
        for off, ns in save.items():
            if len(ns) > 1:
                print(f"  {hex(off)} claimed by {', '.join(sorted(set(ns)))}")
                total += 1
        unverified = [hex(o) for o in sorted(save)
                      if o not in model and o not in option]
        lost = [hex(o) for o in sorted(model - set(save))]
        if unverified:
            print(f"  unverified (not in model): {len(unverified)}: "
                  + " ".join(unverified))
            total += len(unverified)
        if lost:
            print(f"  model offsets not saved: {len(lost)}: " + " ".join(lost))
    print(f"{path}: {total} findings")
    return total


if __name__ == "__main__":
    sys.exit(1 if any(audit(p) for p in
                      (sys.argv[1:] or ["drivers/clk/samsung/clk-zuma.c"]))
             else 0)
