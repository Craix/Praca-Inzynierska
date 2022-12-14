/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v11.0
processor: LPC55S69
package_id: LPC55S69JBD100
mcu_data: ksdk2_0
processor_version: 11.0.1
board: LPCXpresso55S69
functionalGroups:
- name: BOARD_InitPeripherals_cm33_core0
  UUID: 61d0725d-b300-49cb-9c66-b5edfbf8ffc1
  called_from_default_init: true
  selectedCore: cm33_core0
- name: BOARD_InitPeripherals_cm33_core1
  UUID: e2041cd4-ebb6-45a5-807f-e0c2dc047d48
  selectedCore: cm33_core1
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals_cm33_core0 functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals_cm33_core0'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table:
      - 0: []
      - 1: []
      - 2: []
      - 3: []
      - 4: []
      - 5: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * MPU6050_I2C initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'MPU6050_I2C'
- type: 'flexcomm_i2c'
- mode: 'I2C_Polling'
- custom_name_enabled: 'true'
- type_id: 'flexcomm_i2c_c8597948f61bd571ab263ea4330b9dd6'
- functional_group: 'BOARD_InitPeripherals_cm33_core0'
- peripheral: 'FLEXCOMM4'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'FXCOMFunctionClock'
    - clockSourceFreq: 'BOARD_BootClockFROHF96M'
    - i2c_master_config:
      - enableMaster: 'true'
      - baudRate_Bps: '100000'
      - enableTimeout: 'true'
      - timeout_Ms: '70'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t MPU6050_I2C_config = {
  .enableMaster = true,
  .baudRate_Bps = 100000UL,
  .enableTimeout = true,
  .timeout_Ms = 70U
};

static void MPU6050_I2C_init(void) {
  /* Initialization function */
  I2C_MasterInit(MPU6050_I2C_PERIPHERAL, &MPU6050_I2C_config, MPU6050_I2C_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * LCD_SPI initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LCD_SPI'
- type: 'flexcomm_spi'
- mode: 'SPI_Polling'
- custom_name_enabled: 'true'
- type_id: 'flexcomm_spi_481dadba00035f986f31ed9ac95af181'
- functional_group: 'BOARD_InitPeripherals_cm33_core0'
- peripheral: 'FLEXCOMM8'
- config_sets:
  - fsl_spi:
    - spi_mode: 'kSPI_Master'
    - clockSource: 'FXCOMFunctionClock'
    - clockSourceFreq: 'BOARD_BootClockFROHF96M'
    - spi_master_config:
      - enableLoopback: 'false'
      - enableMaster: 'true'
      - polarity: 'kSPI_ClockPolarityActiveHigh'
      - phase: 'kSPI_ClockPhaseFirstEdge'
      - direction: 'kSPI_MsbFirst'
      - baudRate_Bps: '30000000'
      - dataWidth: 'kSPI_Data8Bits'
      - sselNum: 'kSPI_Ssel1'
      - sselPol_set: ''
      - txWatermark: 'kSPI_TxFifo0'
      - rxWatermark: 'kSPI_RxFifo1'
      - delayConfig:
        - preDelay: '0'
        - postDelay: '0'
        - frameDelay: '0'
        - transferDelay: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const spi_master_config_t LCD_SPI_config = {
  .enableLoopback = false,
  .enableMaster = true,
  .polarity = kSPI_ClockPolarityActiveHigh,
  .phase = kSPI_ClockPhaseFirstEdge,
  .direction = kSPI_MsbFirst,
  .baudRate_Bps = 30000000UL,
  .dataWidth = kSPI_Data8Bits,
  .sselNum = kSPI_Ssel1,
  .sselPol = kSPI_SpolActiveAllLow,
  .txWatermark = kSPI_TxFifo0,
  .rxWatermark = kSPI_RxFifo1,
  .delayConfig = {
    .preDelay = 0U,
    .postDelay = 0U,
    .frameDelay = 0U,
    .transferDelay = 0U
  }
};

static void LCD_SPI_init(void) {
  /* Initialization function */
  SPI_MasterInit(LCD_SPI_PERIPHERAL, &LCD_SPI_config, LCD_SPI_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * PWM_TIMER initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PWM_TIMER'
- type: 'ctimer'
- mode: 'PWM'
- custom_name_enabled: 'true'
- type_id: 'ctimer_72ecb1f82fe6700da71dde4e8bc60e39'
- functional_group: 'BOARD_InitPeripherals_cm33_core0'
- peripheral: 'CTIMER2'
- config_sets:
  - fsl_ctimer:
    - ctimerConfig:
      - mode: 'kCTIMER_TimerMode'
      - clockSource: 'FunctionClock'
      - clockSourceFreq: 'BOARD_BootClockFROHF96M'
      - timerPrescaler: '960'
    - EnableTimerInInit: 'true'
    - pwmConfig:
      - pwmPeriodChannel: 'kCTIMER_Match_3'
      - pwmPeriodValueStr: '100'
      - enableInterrupt: 'true'
      - pwmChannels:
        - 0:
          - pwmChannelPrefixId: 'PWM_1'
          - pwmChannel: 'kCTIMER_Match_1'
          - pwmDutyValueStr: '0'
          - enableInterrupt: 'false'
        - 1:
          - pwmChannelPrefixId: 'PWM_2'
          - pwmChannel: 'kCTIMER_Match_2'
          - pwmDutyValueStr: '0'
          - enableInterrupt: 'false'
    - interruptCallbackConfig:
      - interrupt:
        - IRQn: 'CTIMER2_IRQn'
        - enable_priority: 'true'
        - priority: '2'
      - callback: 'kCTIMER_SingleCallback'
      - singleCallback: 'CB_PWM'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ctimer_config_t PWM_TIMER_config = {
  .mode = kCTIMER_TimerMode,
  .input = kCTIMER_Capture_0,
  .prescale = 959
};
/* Single callback functions definition */
ctimer_callback_t PWM_TIMER_callback[] = {CB_PWM};

static void PWM_TIMER_init(void) {
  /* CTIMER2 peripheral initialization */
  CTIMER_Init(PWM_TIMER_PERIPHERAL, &PWM_TIMER_config);
  /* Interrupt vector CTIMER2_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(PWM_TIMER_TIMER_IRQN, PWM_TIMER_TIMER_IRQ_PRIORITY);
  /* PWM channel 1 of CTIMER2 peripheral initialization */
  CTIMER_SetupPwmPeriod(PWM_TIMER_PERIPHERAL, PWM_TIMER_PWM_PERIOD_CH, PWM_TIMER_PWM_1_CHANNEL, PWM_TIMER_PWM_PERIOD, PWM_TIMER_PWM_1_DUTY, false);
  /* PWM channel 2 of CTIMER2 peripheral initialization */
  CTIMER_SetupPwmPeriod(PWM_TIMER_PERIPHERAL, PWM_TIMER_PWM_PERIOD_CH, PWM_TIMER_PWM_2_CHANNEL, PWM_TIMER_PWM_PERIOD, PWM_TIMER_PWM_2_DUTY, false);
  /* Enable interrupt of PWM channel 3 that determinates the PWM period */
  CTIMER_EnableInterrupts(PWM_TIMER_PERIPHERAL, kCTIMER_Match3InterruptEnable);
  CTIMER_RegisterCallBack(PWM_TIMER_PERIPHERAL, PWM_TIMER_callback, kCTIMER_SingleCallback);
  /* Enable interrupt CTIMER2_IRQn request in the NVIC. */
  EnableIRQ(PWM_TIMER_TIMER_IRQN);
  /* Start the timer */
  CTIMER_StartTimer(PWM_TIMER_PERIPHERAL);
}

/***********************************************************************************************************************
 * ADC_BATTERY initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC_BATTERY'
- type: 'lpadc'
- mode: 'LPADC'
- custom_name_enabled: 'true'
- type_id: 'lpadc_ddcc12878b96237847ab78b571214e1c'
- functional_group: 'BOARD_InitPeripherals_cm33_core0'
- peripheral: 'ADC0'
- config_sets:
  - fsl_lpadc:
    - lpadcConfig:
      - clockSource: 'AsynchronousFunctionClock'
      - clockSourceFreq: 'BOARD_BootClockFROHF96M'
      - enableInDozeMode: 'true'
      - conversionAverageMode: 'kLPADC_ConversionAverage1'
      - offsetCalibration: 'no'
      - autoCalibrate: 'true'
      - enableAnalogPreliminary: 'true'
      - powerUpDelay: '0x80'
      - referenceVoltageSource: 'kLPADC_ReferenceVoltageAlt2'
      - powerLevelMode: 'kLPADC_PowerLevelAlt4'
      - triggerPriorityPolicy: 'kLPADC_TriggerPriorityPreemptImmediately'
      - enableConvPause: 'false'
      - convPauseDelay: '0'
      - FIFO0Watermark: '0'
      - FIFO1Watermark: '0'
      - FIFO0WatermarkDMA: 'false'
      - FIFO1WatermarkDMA: 'false'
    - lpadcConvCommandConfig:
      - 0:
        - user_commandId: ''
        - commandId: '1'
        - chainedNextCommandNumber: '0'
        - sampleChannelMode: 'kLPADC_SampleChannelSingleEndSideB'
        - channelNumber: 'CH.8'
        - enableAutoChannelIncrement: 'false'
        - loopCount: '0'
        - hardwareAverageMode: 'kLPADC_HardwareAverageCount128'
        - sampleTimeMode: 'kLPADC_SampleTimeADCK131'
        - hardwareCompareMode: 'kLPADC_HardwareCompareDisabled'
        - hardwareCompareValueHigh: '0'
        - hardwareCompareValueLow: '0'
        - conversionResoultuionMode: 'kLPADC_ConversionResolutionHigh'
        - enableWaitTrigger: 'false'
    - lpadcConvTriggerConfig:
      - 0:
        - user_triggerId: 'adc_battery'
        - triggerId: '5'
        - targetCommandId: '1'
        - delayPower: '0'
        - priority: 'false'
        - channelAFIFOSelect: '0'
        - channelBFIFOSelect: '0'
        - enableHardwareTrigger: 'true'
    - IRQ_cfg:
      - interrupt_type: 'kLPADC_FIFO0WatermarkInterruptEnable'
      - enable_irq: 'true'
      - adc_interrupt:
        - IRQn: 'ADC0_IRQn'
        - enable_interrrupt: 'enabled'
        - enable_priority: 'true'
        - priority: '3'
        - enable_custom_name: 'true'
        - handler_custom_name: 'CB_ADC'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const lpadc_config_t ADC_BATTERY_config = {
  .enableInDozeMode = true,
  .conversionAverageMode = kLPADC_ConversionAverage1,
  .enableAnalogPreliminary = true,
  .powerUpDelay = 0x80UL,
  .referenceVoltageSource = kLPADC_ReferenceVoltageAlt2,
  .powerLevelMode = kLPADC_PowerLevelAlt4,
  .triggerPriorityPolicy = kLPADC_TriggerPriorityPreemptImmediately,
  .enableConvPause = false,
  .convPauseDelay = 0UL,
  .FIFO0Watermark = 0UL,
  .FIFO1Watermark = 0UL
};
lpadc_conv_command_config_t ADC_BATTERY_commandsConfig[1] = {
  {
    .sampleChannelMode = kLPADC_SampleChannelSingleEndSideB,
    .channelNumber = 0U,
    .chainedNextCommandNumber = 0,
    .enableAutoChannelIncrement = false,
    .loopCount = 0UL,
    .hardwareAverageMode = kLPADC_HardwareAverageCount128,
    .sampleTimeMode = kLPADC_SampleTimeADCK131,
    .hardwareCompareMode = kLPADC_HardwareCompareDisabled,
    .hardwareCompareValueHigh = 0UL,
    .hardwareCompareValueLow = 0UL,
    .conversionResolutionMode = kLPADC_ConversionResolutionHigh,
    .enableWaitTrigger = false
  }
};
lpadc_conv_trigger_config_t ADC_BATTERY_triggersConfig[1] = {
  {
    .targetCommandId = 1,
    .delayPower = 0UL,
    .channelAFIFOSelect = 0,
    .channelBFIFOSelect = 0,
    .priority = 1,
    .enableHardwareTrigger = true
  }
};

static void ADC_BATTERY_init(void) {
  /* Initialize LPADC converter */
  LPADC_Init(ADC_BATTERY_PERIPHERAL, &ADC_BATTERY_config);
  /* Perform auto calibration */
  LPADC_DoAutoCalibration(ADC_BATTERY_PERIPHERAL);
  /* Configure conversion command 1. */
  LPADC_SetConvCommandConfig(ADC_BATTERY_PERIPHERAL, 1, &ADC_BATTERY_commandsConfig[0]);
  /* Configure trigger 5. */
  LPADC_SetConvTriggerConfig(ADC_BATTERY_PERIPHERAL, ADC_BATTERY_ADC_BATTERY, &ADC_BATTERY_triggersConfig[0]);
  /* Interrupt vector ADC0_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(ADC_BATTERY_IRQN, ADC_BATTERY_IRQ_PRIORITY);
  /* Enable interrupts from LPADC */
  LPADC_EnableInterrupts(ADC_BATTERY_PERIPHERAL, (kLPADC_FIFO0WatermarkInterruptEnable));
  /* Enable interrupt ADC0_IRQn request in the NVIC. */
  EnableIRQ(ADC_BATTERY_IRQN);
}

/***********************************************************************************************************************
 * ADC_TIMER initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC_TIMER'
- type: 'ctimer'
- mode: 'Capture_Match'
- custom_name_enabled: 'true'
- type_id: 'ctimer_72ecb1f82fe6700da71dde4e8bc60e39'
- functional_group: 'BOARD_InitPeripherals_cm33_core0'
- peripheral: 'CTIMER0'
- config_sets:
  - fsl_ctimer:
    - ctimerConfig:
      - mode: 'kCTIMER_TimerMode'
      - clockSource: 'FunctionClock'
      - clockSourceFreq: 'BOARD_BootClockFROHF96M'
      - timerPrescaler: '1'
    - EnableTimerInInit: 'true'
    - matchChannels:
      - 0:
        - matchChannelPrefixId: 'Match_3'
        - matchChannel: 'kCTIMER_Match_3'
        - matchValueStr: '96000000'
        - enableCounterReset: 'true'
        - enableCounterStop: 'false'
        - outControl: 'kCTIMER_Output_Toggle'
        - outPinInitValue: 'low'
        - enableInterrupt: 'false'
    - captureChannels: []
    - interruptCallbackConfig:
      - interrupt:
        - IRQn: 'CTIMER0_IRQn'
        - enable_priority: 'false'
        - priority: '0'
      - callback: 'kCTIMER_NoCallback'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ctimer_config_t ADC_TIMER_config = {
  .mode = kCTIMER_TimerMode,
  .input = kCTIMER_Capture_0,
  .prescale = 0
};
const ctimer_match_config_t ADC_TIMER_Match_3_config = {
  .matchValue = 95999999,
  .enableCounterReset = true,
  .enableCounterStop = false,
  .outControl = kCTIMER_Output_Toggle,
  .outPinInitState = false,
  .enableInterrupt = false
};

static void ADC_TIMER_init(void) {
  /* CTIMER0 peripheral initialization */
  CTIMER_Init(ADC_TIMER_PERIPHERAL, &ADC_TIMER_config);
  /* Match channel 3 of CTIMER0 peripheral initialization */
  CTIMER_SetupMatch(ADC_TIMER_PERIPHERAL, ADC_TIMER_MATCH_3_CHANNEL, &ADC_TIMER_Match_3_config);
  /* Start the timer */
  CTIMER_StartTimer(ADC_TIMER_PERIPHERAL);
}

/***********************************************************************************************************************
 * PINT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PINT'
- type: 'pint'
- mode: 'interrupt_mode'
- custom_name_enabled: 'false'
- type_id: 'pint_cf4a806bb2a6c1ffced58ae2ed7b43af'
- functional_group: 'BOARD_InitPeripherals_cm33_core0'
- peripheral: 'PINT'
- config_sets:
  - general:
    - interrupt_array:
      - 0:
        - interrupt_id: 'MPU6050'
        - interrupt_selection: 'PINT.0'
        - interrupt_type: 'kPINT_PinIntEnableFallEdge'
        - callback_function: 'CB_MPU6050'
        - enable_callback: 'true'
        - interrupt:
          - IRQn: 'PIN_INT0_IRQn'
          - enable_priority: 'true'
          - priority: '0'
      - 1:
        - interrupt_id: 'ENCA_CLK'
        - interrupt_selection: 'PINT.1'
        - interrupt_type: 'kPINT_PinIntEnableFallEdge'
        - callback_function: 'CB_ENCA'
        - enable_callback: 'true'
        - interrupt:
          - IRQn: 'PIN_INT1_IRQn'
          - enable_priority: 'true'
          - priority: '1'
      - 2:
        - interrupt_id: 'ENCB_CLK'
        - interrupt_selection: 'PINT.2'
        - interrupt_type: 'kPINT_PinIntEnableFallEdge'
        - callback_function: 'CB_ENCB'
        - enable_callback: 'true'
        - interrupt:
          - IRQn: 'PIN_INT2_IRQn'
          - enable_priority: 'true'
          - priority: '1'
      - 3:
        - interrupt_id: 'BUTTON'
        - interrupt_selection: 'PINT.3'
        - interrupt_type: 'kPINT_PinIntEnableFallEdge'
        - callback_function: 'CB_BUTTON'
        - enable_callback: 'true'
        - interrupt:
          - IRQn: 'PIN_INT3_IRQn'
          - enable_priority: 'true'
          - priority: '4'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void PINT_init(void) {
  /* PINT initiation  */
  PINT_Init(PINT_PERIPHERAL);
  /* Interrupt vector PIN_INT0_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(PINT_PINT_0_IRQN, PINT_PINT_0_IRQ_PRIORITY);
  /* Interrupt vector PIN_INT1_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(PINT_PINT_1_IRQN, PINT_PINT_1_IRQ_PRIORITY);
  /* Interrupt vector PIN_INT2_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(PINT_PINT_2_IRQN, PINT_PINT_2_IRQ_PRIORITY);
  /* Interrupt vector PIN_INT3_IRQn priority settings in the NVIC. */
  NVIC_SetPriority(PINT_PINT_3_IRQN, PINT_PINT_3_IRQ_PRIORITY);
  /* PINT PINT.0 configuration */
  PINT_PinInterruptConfig(PINT_PERIPHERAL, PINT_MPU6050, kPINT_PinIntEnableFallEdge, CB_MPU6050);
  /* PINT PINT.1 configuration */
  PINT_PinInterruptConfig(PINT_PERIPHERAL, PINT_ENCA_CLK, kPINT_PinIntEnableFallEdge, CB_ENCA);
  /* PINT PINT.2 configuration */
  PINT_PinInterruptConfig(PINT_PERIPHERAL, PINT_ENCB_CLK, kPINT_PinIntEnableFallEdge, CB_ENCB);
  /* PINT PINT.3 configuration */
  PINT_PinInterruptConfig(PINT_PERIPHERAL, PINT_BUTTON, kPINT_PinIntEnableFallEdge, CB_BUTTON);
  /* Enable PINT PINT.0 callback */
  PINT_EnableCallbackByIndex(PINT_PERIPHERAL, kPINT_PinInt0);
  /* Enable PINT PINT.1 callback */
  PINT_EnableCallbackByIndex(PINT_PERIPHERAL, kPINT_PinInt1);
  /* Enable PINT PINT.2 callback */
  PINT_EnableCallbackByIndex(PINT_PERIPHERAL, kPINT_PinInt2);
  /* Enable PINT PINT.3 callback */
  PINT_EnableCallbackByIndex(PINT_PERIPHERAL, kPINT_PinInt3);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals_cm33_core0(void)
{
  /* Initialize components */
  MPU6050_I2C_init();
  LCD_SPI_init();
  PWM_TIMER_init();
  ADC_BATTERY_init();
  ADC_TIMER_init();
  PINT_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals_cm33_core0();
}
