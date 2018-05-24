COMPONENT=NeighborDiscoveryC

PFLAGS += -I/opt/tinyos-2.1.2/wustl/upma/lib/macs/RbMac
PFLAGS += -I/opt/tinyos-2.1.2/tos/lib/printf

PFLAGS += -DLOW_POWER_PROBING
#PFLAGS += -DQS_ACK
PFLAGS += -DRbMac
PFLAGS += -DDEBUG_GCC
UPMA_MAC = RbMac

include $(UPMA_DIR)/Makefile.include
include $(MAKERULES)
