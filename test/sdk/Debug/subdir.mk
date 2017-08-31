################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../FTCardDriver_class.cpp \
../FTCardLib.cpp \
../test_lib.cpp 

OBJS += \
./FTCardDriver_class.o \
./FTCardLib.o \
./test_lib.o 

CPP_DEPS += \
./FTCardDriver_class.d \
./FTCardLib.d \
./test_lib.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


