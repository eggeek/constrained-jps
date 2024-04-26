WARTHOG_SRC = $(wildcard src/*/*.cpp)
EXTRA_SRC = $(wildcard extra/*.cpp)

WARTHOG_EXE = $(wildcard programs/*.cpp)
WARTHOG_TEST = $(wildcard test/*.cpp)
WARTHOG_OBJ = $(WARTHOG_SRC:%.cpp=obj/%.o)
EXTRA_OBJ = $(EXTRA_SRC:%.cpp=obj/%.o)
CXX = clang++

D_WARTHOG_INCLUDES = -I../../src/ -I../../src/domains -I../../src/util			\
	-I../../src/search -I../../src/experimental -I../../src/heuristics				\
	-I../../src/jps -I../../src/contraction -I../../src/label -I../../src/memory	\
	-I../../src/mapf -I../../src/sys -I../../src/sipp -I../../src/cpd				\
	-I../../third_party -I../../extra

D_INCLUDES = $(D_WARTHOG_INCLUDES) -I/usr/include -I/usr/local/include
D_LIBS = -L./lib -L/usr/local/lib

warthog = ./lib/libwarthog.a
extra = ./lib/libwarthog-extra.a

CFLAGS = -std=c++11 -pedantic -pthread -Wall -Wno-conversion \
	-Wno-unused-result -Wno-unused-but-set-variable
# PROFILE_CFLAGS = $(DEV_CFLAGS) -pg -DNDEBUG

FLAVOURS = fast dev debug
PROGRAMS = $(WARTHOG_EXE:programs/%.cpp=bin/%)
PROGRAMS += $(WARTHOG_TEST:.cpp=)

ifeq ("$(findstring Darwin, "$(shell uname -s)")", "Darwin")
  CC = g++
  CFLAGS += -DOS_MAC
else
  CC = g++
  ifeq ("$(findstring Linux, "$(shell uname -s)")", "Linux")
    D_LIBS += -lrt
  endif
endif

.EXPORT_ALL_VARIABLES:

.PHONY: all help default clean clobber $(FLAVOURS)

.SILENT: all clean clobber

default:
	+$(MAKE) fast main

##@ Options

# These need to be explicitly split
all:														## Build all flavours
	for dir in $(FLAVOURS); do \
		$(MAKE) $$dir; \
	done

help:														## Print this message
	@echo "Compile warthog in different flavours, the executables will be built"
	@echo "in their respective directory."
	@echo ""
	@echo "Usage: make [cmd | flavour [target]]"
	@awk 'BEGIN {FS = ":.*##";} /^[a-zA-Z_-]+:.*?##/ { printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2 } /^##@/ { printf "\n\033[1m%s\033[0m\n", substr($$0, 5) } ' $(MAKEFILE_LIST)
	@echo "$(PROGRAMS)"

##@ Cleaning

clean:													## Clean build artifacts, can be run as a flavour
ifeq ($(firstword $(MAKECMDGOALS)), clean) # Only run if invoked directly
	for dir in $(FLAVOURS); do \
		if [ -d "build/$$dir" ]; then $(MAKE) -C "build/$$dir" clean; fi \
	done
endif

clobber:												## Remove build directory
	if [ -d build ]; then $(RM) -rf build; fi

##@ Flavours
# List of *actions* to run for the chose flavour
ACTIONS = $(wordlist 2, $(words $(MAKECMDGOALS)), $(MAKECMDGOALS))

# We need to call make explicitly on the subdirs as they are the ones deciding
# what needs to run.
fast: CFLAGS += -O3 -DNDEBUG -Wno-unused-variable
fast: build/fast/Makefile		## Compile with opti flags
	+$(MAKE) -C $(<D) $(ACTIONS)

fastcnt: CFLAGS += -O3 -Darwin -Wno-unused-variable $(FAST_CFLAGS) -DCNT $(D_INCLUDES)
fastcnt: build/fast/Makefile		## Compile with opti flags
	+$(MAKE) -C $(<D) $(ACTIONS)

dev: CFLAGS += -ggdb -O0 -gdwarf-4 -DSINGLE_THREADED -DQUICK_LOG=1 -DCNT
dev: build/dev/Makefile			## Compile with debug symbols
	+$(MAKE) -C $(<D) $(ACTIONS)

debug: CFLAGS += -ggdb -O3 -gdwarf-4 -DQUICK_LOG=3 -fno-omit-frame-pointer
debug: build/debug/Makefile		## Opti flags with debug symbols
	+$(MAKE) -C $(<D) $(ACTIONS)

# Generate the Makefile of the appropriate flavour
%/Makefile: make.file
	@$(shell mkdir -p $(@D))
	@$(shell cp $< $@)

# Gobble extra targets
%:
	@true

# Used for help
##@ Targets
