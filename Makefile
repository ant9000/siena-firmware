APPLICATION = siena-firmware
BOARD ?= lora3a-h10
RIOTBASE ?= $(CURDIR)/../RIOT
LORA3ABASE ?= $(CURDIR)/../lora3a-boards-ant9000
EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg
DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0
BACKUP_MODE ?= 1
ADDRESS ?= 2
AES ?= 0


USEMODULE += od
USEMODULE += od_string
USEMODULE += hdc3020
USEMODULE += fram
USEMODULE += saml21_cpu_debug
USEMODULE += saml21_backup_mode
USEMODULE += printf_float
USEMODULE += saul_default
USEMODULE += periph_adc
USEMODULE += periph_spi_reconfigure
USEMODULE += at24c16a
USEMODULE += sx1276
USEMODULE += ztimer_msec
USEMODULE += ztimer_usec
USEMODULE += periph_rtc_mem
USEPKG += soniclib

#CFLAGS += -DENABLE_ACME1
CFLAGS += -DENABLE_ACME2=MODE_I2C

ifeq ($(BACKUP_MODE), 1)
  CFLAGS += -DBACKUP_MODE
endif

ifneq (,$(ADDRESS))
  CFLAGS += -DEMB_ADDRESS=$(ADDRESS)
endif


include $(RIOTBASE)/Makefile.include
