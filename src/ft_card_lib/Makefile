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
LIBRARIES := 
CC_DEPS := 
C++_DEPS := 
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
./FTCardDriver_class.cpp \
./FTCardLib.cpp 

OBJS += \
./FTCardDriver_class.o \
./FTCardLib.o 

CPP_DEPS += \
./FTCardDriver_class.d \
./FTCardLib.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ./%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -fPIC -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

USER_OBJS :=

LIBS :=

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
all: libft_card_lib.so

# Tool invocations
libft_card_lib.so: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -shared -o "libft_card_lib.so" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(LIBRARIES)$(CC_DEPS)$(C++_DEPS)$(C_UPPER_DEPS)$(CXX_DEPS)$(OBJS)$(CPP_DEPS)$(C_DEPS) libft_card_lib.so
	-@echo ' '

.PHONY: all clean dependents

