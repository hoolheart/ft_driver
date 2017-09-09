# All of the sources participating in the build are defined here
C_UPPER_SRCS := 
CXX_SRCS := 
C++_SRCS := 
OBJ_SRCS := 
CC_SRCS := 
ASM_SRCS := 
CPP_SRCS := 
C_SRCS := 
O_SRCS := 
S_UPPER_SRCS := 
CC_DEPS := 
C++_DEPS := 
EXECUTABLES := 
C_UPPER_DEPS := 
CXX_DEPS := 
OBJS := 
CPP_DEPS := 
C_DEPS := 

# Every subdirectory with source files must be described here
SUBDIRS := \
. \

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
./test_lib.cpp 

OBJS += \
./test_lib.o 

CPP_DEPS += \
./test_lib.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

USER_OBJS :=

LIBS := -lft_card_lib

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# All Target
all: test_lib

# Tool invocations
test_lib: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -L. -pthread -o "test_lib" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(C_UPPER_DEPS)$(CXX_DEPS)$(OBJS)$(CPP_DEPS)$(C_DEPS) test_lib
	-@echo ' '

.PHONY: all clean dependents

