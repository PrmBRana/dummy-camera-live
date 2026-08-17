include $(APPDIR)/Make.defs

PROGNAME  = ipcc_test
PRIORITY  = 100
STACKSIZE = 2048

CFLAGS += -I$(TOPDIR)/../shareRAM/ipc

MAINSRC = ipcc_test.c
CSRCS   = $(TOPDIR)/../shareRAM/ipc/Sring_buffer.c

include $(APPDIR)/Application.mk