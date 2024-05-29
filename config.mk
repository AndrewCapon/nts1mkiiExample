##############################################################################
# Configuration for Makefile
#

PROJECT := test_osc
PROJECT_TYPE := osc

##############################################################################
# Sources
#

# C sources 
UCSRC = header.c
ifeq ($(USE_USART), 1)
	UCSRC +=  stm32h7xx_ll_gpio.c stm32h7xx_ll_rcc.c stm32h7xx_ll_usart.c system_stm32h7xx.c
endif

# C++ sources 
UCXXSRC = unit.cc

# List ASM source files here
UASMSRC = 

UASMXSRC = 

##############################################################################
# Include Paths
#

UINCDIR  = 

##############################################################################
# Library Paths
#

ULIBDIR = 

##############################################################################
# Libraries
#

ULIBS  = -lm

##############################################################################
# Macros
#

UDEFS = 

