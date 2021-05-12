################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-arm_20.2.2.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./ADCdemo1.obj" \
"./ADCdemo2.obj" \
"./Main.obj" \
"./OLEDTest.obj" \
"./butsTest4.obj" \
"./buttons4.obj" \
"./circBufT.obj" \
"./pwmGen.obj" \
"./pwmGen2.obj" \
"./pwmGen3.obj" \
"./pwmGen4.obj" \
"./tm4c123gh6pm_startup_ccs.obj" \
"./uartDemo.obj" \
"./ustdlib.obj" \
"./Modules/Altitude.obj" \
"./Modules/Display.obj" \
"./Modules/PWM.obj" \
"./Modules/PWMcontrol.obj" \
"./Modules/Yaw.obj" \
"./OrbitOLED/OrbitOLEDInterface.obj" \
"./OrbitOLED/lib_OrbitOled/ChrFont0.obj" \
"./OrbitOLED/lib_OrbitOled/FillPat.obj" \
"./OrbitOLED/lib_OrbitOled/OrbitOled.obj" \
"./OrbitOLED/lib_OrbitOled/OrbitOledChar.obj" \
"./OrbitOLED/lib_OrbitOled/OrbitOledGrph.obj" \
"./OrbitOLED/lib_OrbitOled/delay.obj" \
"../tm4c123gh6pm.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Modules/subdir_vars.mk
-include OrbitOLED/subdir_vars.mk
-include OrbitOLED/lib_OrbitOled/subdir_vars.mk
-include subdir_rules.mk
-include Modules/subdir_rules.mk
-include OrbitOLED/subdir_rules.mk
-include OrbitOLED/lib_OrbitOled/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Helicopter.out \

EXE_OUTPUTS__QUOTED += \
"Helicopter.out" \

BIN_OUTPUTS += \
Helicopter.hex \

BIN_OUTPUTS__QUOTED += \
"Helicopter.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "Helicopter.out"

# Tool invocations
Helicopter.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-arm_20.2.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --define=ccs="ccs" --define=PART_TM4C123GH6PM --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"Helicopter.map" --heap_size=0 --stack_size=512 -i"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-arm_20.2.2.LTS/lib" -i"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-arm_20.2.2.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="Helicopter_linkInfo.xml" --rom_model -o "Helicopter.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

Helicopter.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccs1020/ccs/tools/compiler/ti-cgt-arm_20.2.2.LTS/bin/armhex" --diag_wrap=off -o "Helicopter.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ADCdemo1.obj" "ADCdemo2.obj" "Main.obj" "OLEDTest.obj" "butsTest4.obj" "buttons4.obj" "circBufT.obj" "pwmGen.obj" "pwmGen2.obj" "pwmGen3.obj" "pwmGen4.obj" "tm4c123gh6pm_startup_ccs.obj" "uartDemo.obj" "ustdlib.obj" "Modules\Altitude.obj" "Modules\Display.obj" "Modules\PWM.obj" "Modules\PWMcontrol.obj" "Modules\Yaw.obj" "OrbitOLED\OrbitOLEDInterface.obj" "OrbitOLED\lib_OrbitOled\ChrFont0.obj" "OrbitOLED\lib_OrbitOled\FillPat.obj" "OrbitOLED\lib_OrbitOled\OrbitOled.obj" "OrbitOLED\lib_OrbitOled\OrbitOledChar.obj" "OrbitOLED\lib_OrbitOled\OrbitOledGrph.obj" "OrbitOLED\lib_OrbitOled\delay.obj" 
	-$(RM) "ADCdemo1.d" "ADCdemo2.d" "Main.d" "OLEDTest.d" "butsTest4.d" "buttons4.d" "circBufT.d" "pwmGen.d" "pwmGen2.d" "pwmGen3.d" "pwmGen4.d" "tm4c123gh6pm_startup_ccs.d" "uartDemo.d" "ustdlib.d" "Modules\Altitude.d" "Modules\Display.d" "Modules\PWM.d" "Modules\PWMcontrol.d" "Modules\Yaw.d" "OrbitOLED\OrbitOLEDInterface.d" "OrbitOLED\lib_OrbitOled\ChrFont0.d" "OrbitOLED\lib_OrbitOled\FillPat.d" "OrbitOLED\lib_OrbitOled\OrbitOled.d" "OrbitOLED\lib_OrbitOled\OrbitOledChar.d" "OrbitOLED\lib_OrbitOled\OrbitOledGrph.d" "OrbitOLED\lib_OrbitOled\delay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

