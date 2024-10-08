###########################################################################
#
# Copyright 2016-2017 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
###########################################################################

-include $(TOPDIR)/Make.defs

ASRCS =

CSRCS += pm_changestate.c pm_checkstate.c pm_initialize.c pm_idle.c
CSRCS += pm_register.c pm_domain_register.c pm_unregister.c pm_procfs.c
CSRCS += pm_suspend.c pm_resume.c pm_timedsuspend.c
CSRCS += pm_suspendcount.c pm_wakehandler.c

ifeq ($(CONFIG_PM_TIMEDWAKEUP),y)
CSRCS += pm_sleep.c
endif

ifeq ($(CONFIG_PM_METRICS),y)
CSRCS += pm_metrics.c
endif

ifeq ($(CONFIG_PM_DVFS),y)
CSRCS += pm_dvfs.c
endif


DEPPATH = --dep-path .
VPATH = .

COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libpm$(LIBEXT)


#ifeq ($(CONFIG_WINDOWS_NATIVE),y)
#  CFLAGS += -I$(ARCH_SRCDIR)\board
#else
#  ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
#ifeq ($(WINTOOL),y)
#  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/board}"
#else
#  CFLAGS += -I$(ARCH_SRCDIR)/board
#endif
#endif


all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
