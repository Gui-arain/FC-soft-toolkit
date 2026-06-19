CXXEXT = .cpp

include $(APPDIR)/Make.defs

PROGNAME  = estimator
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_FC_ESTIMATOR)

CXXFLAGS += -I$(TOPDIR)/../drivers/include

MAINSRC   = estimator_main.cpp

include $(APPDIR)/Application.mk
