################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/AMath.cpp \
../src/APVRec.cpp \
../src/ATimeSpace.cpp \
../src/pvrec.cpp 

OBJS += \
./src/AMath.o \
./src/APVRec.o \
./src/ATimeSpace.o \
./src/pvrec.o 

CPP_DEPS += \
./src/AMath.d \
./src/APVRec.d \
./src/ATimeSpace.d \
./src/pvrec.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


