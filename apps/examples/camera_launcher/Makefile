include $(APPDIR)/Make.defs

PROGNAME  = launcher
PRIORITY  = 100
STACKSIZE = 1536
MODULE    = $(CONFIG_EXAMPLES_CAMERA_LAUNCHER)

MAINSRC = launcher.c
CFLAGS += -I$(TOPDIR)/../shareRAM/ipc

include $(APPDIR)/Application.mk
