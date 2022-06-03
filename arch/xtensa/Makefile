#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 2001 - 2005  Tensilica Inc.
# Copyright (C) 2014 Cadence Design Systems Inc.
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.

# Core configuration.
# (Use VAR=<xtensa_config> to use another default compiler.)

variant-y := $(CONFIG_XTENSA_VARIANT_NAME)

VARIANT = $(variant-y)

ifneq ($(VARIANT),)
  ifdef cross_compiling
    ifndef CROSS_COMPILE
      CROSS_COMPILE = xtensa_$(VARIANT)-
    endif
  endif
endif

# The Espressif "xtensa-esp-elf" toolchain is built for several core
# configurations at once and uses none of them unless XTENSA_GNU_CONFIG points
# at one. Without it the assembler rejects core-specific registers and opcodes
# (ATOMCTL, RER/WER, the PIE and HWLP TIE, ...), and the Kconfig compiler
# probes describe the wrong core: the fallback configuration is big endian, so
# CONFIG_CPU_BIG_ENDIAN ends up disagreeing with the code the compiler emits.
# Toolchains built for a single core configuration have it compiled in and need
# nothing here.
#
# *config targets run before CONFIG_XTENSA_VARIANT_NAME is known, so the
# variant then has to come from the configuration that is about to be used.

xtensa-variant := $(VARIANT)

ifeq ($(xtensa-variant),)
  xtensa-config := $(firstword $(wildcard \
	$(addprefix $(srctree)/arch/xtensa/configs/,$(filter %_defconfig,$(MAKECMDGOALS))) \
	$(objtree)/.config))
  ifneq ($(xtensa-config),)
    xtensa-variant := $(shell sed -n \
	's/^CONFIG_XTENSA_VARIANT_CUSTOM_NAME="\(.*\)"$$/\1/p' $(xtensa-config))
  endif
endif

ifneq ($(xtensa-variant),)
  xtensa-libgcc-dir := $(dir $(shell $(CC) -print-libgcc-file-name))
  xtensa-gnu-config := $(firstword $(wildcard $(xtensa-libgcc-dir)../../../xtensa_$(xtensa-variant).so))
  ifneq ($(xtensa-gnu-config),)
    export XTENSA_GNU_CONFIG := $(xtensa-gnu-config)
  endif
endif

# Platform configuration

platform-$(CONFIG_XTENSA_PLATFORM_XT2000)	:= xt2000
platform-$(CONFIG_XTENSA_PLATFORM_ISS)		:= iss
platform-$(CONFIG_XTENSA_PLATFORM_XTFPGA)	:= xtfpga

# temporarily until string.h is fixed
KBUILD_CFLAGS += -ffreestanding -D__linux__
KBUILD_CFLAGS += -pipe -mlongcalls -mtext-section-literals
KBUILD_CFLAGS += $(call cc-option,-mforce-no-pic,)
KBUILD_CFLAGS += $(call cc-option,-mno-serialize-volatile,)
KBUILD_CFLAGS += $(call cc-option,-mno-fdpic,)
ifneq ($(CONFIG_KERNEL_ABI_CALL0),)
KBUILD_CFLAGS += -mabi=call0
KBUILD_AFLAGS += -mabi=call0
endif

KBUILD_AFLAGS += -mlongcalls -mtext-section-literals
KBUILD_AFLAGS += $(call cc-option,-mno-fdpic,)

KBUILD_LDFLAGS += -m elf32xtensa

ifneq ($(CONFIG_LD_NO_RELAX),)
KBUILD_LDFLAGS += --no-relax
endif

CHECKFLAGS += -D$(if $(CONFIG_CPU_BIG_ENDIAN),__XTENSA_EB__,__XTENSA_EL__)

vardirs := $(patsubst %,arch/xtensa/variants/%/,$(variant-y))
plfdirs := $(patsubst %,arch/xtensa/platforms/%/,$(platform-y))

KBUILD_CPPFLAGS += $(patsubst %,-I$(srctree)/%include,$(vardirs) $(plfdirs))

KBUILD_DEFCONFIG := iss_defconfig

libs-y		+= arch/xtensa/lib/

boot		:= arch/xtensa/boot

all Image zImage uImage xipImage: vmlinux
	$(Q)$(MAKE) $(build)=$(boot) $@

archheaders:
	$(Q)$(MAKE) $(build)=arch/xtensa/kernel/syscalls all

define archhelp
  @echo '* Image       - Kernel ELF image with reset vector'
  @echo '* zImage      - Compressed kernel image (arch/xtensa/boot/images/zImage.*)'
  @echo '* uImage      - U-Boot wrapped image'
  @echo '  xipImage    - XIP image'
endef
