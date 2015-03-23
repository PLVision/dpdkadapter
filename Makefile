# source files.
SRC = dpdk_adapter.cpp dpdk_api.cpp dpdk_log.cpp
 
OBJ = $(SRC:.cpp=.o)
 
OUT = ./libdpdkadapter.so
 
# include directories
INCLUDES = -I. -I../dpdk-1.7.0/x86_64-native-linuxapp-gcc/include

# include files
INCLUDE_FILES = -include ../dpdk-1.7.0/x86_64-native-linuxapp-gcc/include/rte_config.h

# C++ compiler flags (-g -O2 -Wall)
CCFLAGS = -g -D__STDC_LIMIT_MACROS

# RTE related definitions
CCFLAGS += -DRTE_MAX_LCORE=64 \
			-DRTE_PKTMBUF_HEADROOM=128 \
			-DRTE_MAX_ETHPORTS=32

CCFLAGS += -DRTE_MACHINE_CPUFLAG_SSE \
                  -DRTE_MACHINE_CPUFLAG_SSE2 \
                  -DRTE_MACHINE_CPUFLAG_SSE3 \
                  -DRTE_MACHINE_CPUFLAG_SSSE3 \
                  -DRTE_COMPILE_TIME_CPUFLAGS=RTE_CPUFLAG_SSE,RTE_CPUFLAG_SSE2,RTE_CPUFLAG_SSE3,RTE_CPUFLAG_SSSE3

# make code position-independent
CCFLAGS += -fPIC

# compiler
CCC = g++
 
# library paths
LIBS = -L../dpdk-1.7.0/x86_64-native-linuxapp-gcc/lib -lintel_dpdk -ldl -pthread
 
# compile flags
LDFLAGS = -g -shared
 
.SUFFIXES: .cpp
 
default: $(OUT)
 
.cpp.o:
	$(CCC) $(INCLUDES) $(INCLUDE_FILES) $(CCFLAGS) -c $< -o $@
 
$(OUT): $(OBJ)
	$(CCC) $(LDFLAGS) $^ -o $@ $(LIBS)
 
clean:
	rm -f $(OBJ) $(OUT) Makefile.bak
