# Default Makefile, for host native GNRC-based networking

# name of your application
APPLICATION = gcoap_example

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

BOARD_INSUFFICIENT_MEMORY := chronos msb-430 msb-430h nucleo32-f031 nucleo32-f042 \
                             nucleo32-l031 nucleo-f030 nucleo-f334 nucleo-l053 \
                             stm32f0discovery telosb wsn430-v1_3b wsn430-v1_4 z1

## Uncomment to redefine port, for example use 61616 for RFC 6282 UDP compression.
#GCOAP_PORT = 5683
#CFLAGS += -DGCOAP_PORT=$(GCOAP_PORT)

## Uncomment to redefine request token length, max 8.
#GCOAP_TOKENLEN = 2
#CFLAGS += -DGCOAP_TOKENLEN=$(GCOAP_TOKENLEN)

# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif

# Specify the mandatory networking modules
USEMODULE += gnrc_ipv6_default
USEMODULE += gcoap
# Additional networking modules that can be dropped if not needed
USEMODULE += gnrc_icmpv6_echo

# Required by gcoap example
USEMODULE += od
USEMODULE += fmt
# Add also the shell, some shell commands
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

CFLAGS += -DGNRC_PKTBUF_SIZE=512
CFLAGS += -DGNRC_IPV6_NETIF_ADDR_NUMOF=5
CFLAGS += -DGNRC_IPV6_NC_SIZE=1
#CFLAGS += -DGNRC_IPV6_NIB_NUMOF=1

CFLAGS += -DTHREAD_STACKSIZE_DEFAULT=512
CFLAGS += -DTHREAD_STACKSIZE_IDLE=129
CFLAGS += -DTHREAD_EXTRA_STACKSIZE_PRINTF=128

CFLAGS += -DAT86RF2XX_MAC_STACKSIZE=512
CFLAGS += -DGNRC_SIXLOWPAN_STACK_SIZE=512
CFLAGS += -DGNRC_IPV6_STACK_SIZE=512
CFLAGS += -DGNRC_RPL_STACK_SIZE=512


CFLAGS += -DMODULE_GNRC_NETIF_SHELL_ENABLE
CFLAGS += -DMODULE_PS_SHELL_ENABLE
CFLAGS += -DMODULE_GNRC_ICMPV6_ECHO_SHELL_ENABLE

#verbose assertion message
CFLAGS += -DDEBUG_ASSERT_VERBOSE

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
