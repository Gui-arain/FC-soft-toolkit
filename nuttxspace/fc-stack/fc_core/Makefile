include $(APPDIR)/Make.defs

PROGNAME  = fc_core
PRIORITY  = SCHED_PRIORITY_MAX
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_FC_CORE)

MAINSRC   = fc_core_main.cpp      # .cpp, not .c
CXXSRCS   = attitude.cpp \        # CXXSRCS, not CSRCS
             motors.cpp \
             scheduler.cpp

include $(APPDIR)/Application.mk