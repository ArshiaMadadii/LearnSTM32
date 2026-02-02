#include "main.h"
#include "i2c.h"

#include "MAX30102/MAX30102.h"
#include "MAX30102/algorithm.h"

#define I2C_TIMEOUT 100

I2C_HandleTypeDef *i2c_max30102;

volatile uint32_t IrBuffer[MAX30102_BUFFER_LENGTH];      // IR LED sensor data
volatile uint32_t RedBuffer[MAX30102_BUFFER_LENGTH];     // Red LED sensor data
volatile uint32_t BufferHead;
volatile uint32_t BufferTail;
volatile uint32_t CollectedSamples;
volatile uint8_t  IsFingerOnScreen;

int32_t Sp02Value;
int8_t  Sp02IsValid;
int32_t HeartRate;
int8_t  IsHrValid;

typedef enum
{
  MAX30102_STATE_BEGIN,
  MAX30102_STATE_CALIBRATE,
  MAX30102_STATE_CALCULATE_HR,
  MAX30102_STATE_COLLECT_NEXT_PORTION
} MAX30102_STATE;

MAX30102_STATE StateMachine;

// ----------------------------------------------------
// Low-level I2C register access
// ----------------------------------------------------
MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data)
{
  if (HAL_I2C_Mem_Write(i2c_max30102, MAX30102_ADDRESS, uch_addr, 1,
                        &uch_data, 1, I2C_TIMEOUT) == HAL_OK)
    return MAX30102_OK;
  return MAX30102_ERROR;
}

MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data)
{
  if (HAL_I2C_Mem_Read(i2c_max30102, MAX30102_ADDRESS, uch_addr, 1,
                       puch_data, 1, I2C_TIMEOUT) == HAL_OK)
    return MAX30102_OK;
  return MAX30102_ERROR;
}

MAX30102_STATUS Max30102_WriteRegisterBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t tmp;
  if (MAX30102_OK != Max30102_ReadReg(Register, &tmp))
    return MAX30102_ERROR;

  tmp &= ~(1U << Bit);
  tmp |= ((Value & 0x01U) << Bit);

  if (MAX30102_OK != Max30102_WriteReg(Register, tmp))
    return MAX30102_ERROR;

  return MAX30102_OK;
}

// ----------------------------------------------------
// FIFO read (RAW sample)
// ----------------------------------------------------
MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led,
                                  volatile uint32_t *pun_ir_led)
{
  uint32_t un_temp;
  *pun_red_led = 0;
  *pun_ir_led  = 0;

  uint8_t ach_i2c_data[6];

  if (HAL_I2C_Mem_Read(i2c_max30102, MAX30102_ADDRESS, REG_FIFO_DATA, 1,
                       ach_i2c_data, 6, I2C_TIMEOUT) != HAL_OK)
  {
    return MAX30102_ERROR;
  }

  // RED (24-bit)
  un_temp = (uint32_t)ach_i2c_data[0];
  un_temp <<= 16;
  *pun_red_led += un_temp;
  un_temp = (uint32_t)ach_i2c_data[1];
  un_temp <<= 8;
  *pun_red_led += un_temp;
  un_temp = (uint32_t)ach_i2c_data[2];
  *pun_red_led += un_temp;

  // IR (24-bit)
  un_temp = (uint32_t)ach_i2c_data[3];
  un_temp <<= 16;
  *pun_ir_led += un_temp;
  un_temp = (uint32_t)ach_i2c_data[4];
  un_temp <<= 8;
  *pun_ir_led += un_temp;
  un_temp = (uint32_t)ach_i2c_data[5];
  *pun_ir_led += un_temp;

  // Mask to 18-bit
  *pun_red_led &= 0x03FFFF;
  *pun_ir_led  &= 0x03FFFF;

  return MAX30102_OK;
}

// ----------------------------------------------------
// Interrupt enable/flags
// ----------------------------------------------------
MAX30102_STATUS Max30102_SetIntAlmostFullEnabled(uint8_t Enable)
{
  return Max30102_WriteRegisterBit(REG_INTR_ENABLE_1, INT_A_FULL_BIT, Enable);
}

MAX30102_STATUS Max30102_SetIntFifoDataReadyEnabled(uint8_t Enable)
{
  return Max30102_WriteRegisterBit(REG_INTR_ENABLE_1, INT_PPG_RDY_BIT, Enable);
}

MAX30102_STATUS Max30102_SetIntAmbientLightCancelationOvfEnabled(uint8_t Enable)
{
  return Max30102_WriteRegisterBit(REG_INTR_ENABLE_2, INT_ALC_OVF_BIT, Enable);
}

#ifdef MAX30102_USE_INTERNAL_TEMPERATURE
MAX30102_STATUS Max30102_SetIntInternalTemperatureReadyEnabled(uint8_t Enable)
{
  return Max30102_WriteRegisterBit(REG_INTR_ENABLE_2, INT_DIE_TEMP_RDY_BIT, Enable);
}
#endif

MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status)
{
  uint8_t tmp;
  *Status = 0;

  if (MAX30102_OK != Max30102_ReadReg(REG_INTR_STATUS_1, &tmp))
    return MAX30102_ERROR;

  *Status |= (tmp & 0xE1); // keep only used bits

#ifdef MAX30102_USE_INTERNAL_TEMPERATURE
  if (MAX30102_OK != Max30102_ReadReg(REG_INTR_STATUS_2, &tmp))
    return MAX30102_ERROR;
  *Status |= (tmp & 0x02);
#endif

  return MAX30102_OK;
}

void Max30102_InterruptCallback(void)
{
  uint8_t Status;

  while (MAX30102_OK != Max30102_ReadInterruptStatus(&Status)) {}

  // Almost Full
  if (Status & (1U << INT_A_FULL_BIT))
  {
    for (uint8_t i = 0; i < MAX30102_FIFO_ALMOST_FULL_SAMPLES; i++)
    {
      while (MAX30102_OK != Max30102_ReadFifo((RedBuffer + BufferHead),
                                              (IrBuffer  + BufferHead))) {}

      if (IsFingerOnScreen)
      {
        if (IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR)
          IsFingerOnScreen = 0;
      }
      else
      {
        if (IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR)
          IsFingerOnScreen = 1;
      }

      BufferHead = (BufferHead + 1U) % MAX30102_BUFFER_LENGTH;
      CollectedSamples++;
    }
  }

  // PPG Ready
  if (Status & (1U << INT_PPG_RDY_BIT))
  {
    while (MAX30102_OK != Max30102_ReadFifo((RedBuffer + BufferHead),
                                            (IrBuffer  + BufferHead))) {}

    if (IsFingerOnScreen)
    {
      if (IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR)
        IsFingerOnScreen = 0;
    }
    else
    {
      if (IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR)
        IsFingerOnScreen = 1;
    }

    BufferHead = (BufferHead + 1U) % MAX30102_BUFFER_LENGTH;
    CollectedSamples++;
  }

  // ALC_OVF / PWR_RDY / TEMP_RDY are ignored here
}

// ----------------------------------------------------
// FIFO Configuration
// ----------------------------------------------------
MAX30102_STATUS Max30102_FifoWritePointer(uint8_t Address)
{
  if (MAX30102_OK != Max30102_WriteReg(REG_FIFO_WR_PTR, (Address & 0x1F)))
    return MAX30102_ERROR;
  return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoOverflowCounter(uint8_t Address)
{
  if (MAX30102_OK != Max30102_WriteReg(REG_OVF_COUNTER, (Address & 0x1F)))
    return MAX30102_ERROR;
  return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoReadPointer(uint8_t Address)
{
  if (MAX30102_OK != Max30102_WriteReg(REG_FIFO_RD_PTR, (Address & 0x1F)))
    return MAX30102_ERROR;
  return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value)
{
  uint8_t tmp;
  if (MAX30102_OK != Max30102_ReadReg(REG_FIFO_CONFIG, &tmp))
    return MAX30102_ERROR;

  // FIX: bits [7:5]
  tmp &= ~(0x07U << 5);
  tmp |= ((Value & 0x07U) << 5);

  if (MAX30102_OK != Max30102_WriteReg(REG_FIFO_CONFIG, tmp))
    return MAX30102_ERROR;

  return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable)
{
  return Max30102_WriteRegisterBit(REG_FIFO_CONFIG,
                                   FIFO_CONF_FIFO_ROLLOVER_EN_BIT,
                                   (Enable & 0x01U));
}

MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value)
{
  if (Value < 17) Value = 17;
  if (Value > 32) Value = 32;

  Value = (uint8_t)(32 - Value);

  uint8_t tmp;
  if (MAX30102_OK != Max30102_ReadReg(REG_FIFO_CONFIG, &tmp))
    return MAX30102_ERROR;

  tmp &= ~(0x0FU);
  tmp |= (Value & 0x0FU);

  if (MAX30102_OK != Max30102_WriteReg(REG_FIFO_CONFIG, tmp))
    return MAX30102_ERROR;

  return MAX30102_OK;
}

// ----------------------------------------------------
// Mode Configuration
// ----------------------------------------------------
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable)
{
  return Max30102_WriteRegisterBit(REG_MODE_CONFIG, MODE_SHDN_BIT, (Enable & 0x01U));
}

MAX30102_STATUS Max30102_Reset(void)
{
  uint8_t tmp = 0xFF;

  if (MAX30102_OK != Max30102_WriteReg(REG_MODE_CONFIG, 0x40))
    return MAX30102_ERROR;

  do
  {
    if (MAX30102_OK != Max30102_ReadReg(REG_MODE_CONFIG, &tmp))
      return MAX30102_ERROR;
  } while (tmp & (1U << 6));

  return MAX30102_OK;
}

MAX30102_STATUS Max30102_SetMode(uint8_t Mode)
{
  uint8_t tmp;
  if (MAX30102_OK != Max30102_ReadReg(REG_MODE_CONFIG, &tmp))
    return MAX30102_ERROR;

  tmp &= ~(0x07U);
  tmp |= (Mode & 0x07U);

  if (MAX30102_OK != Max30102_WriteReg(REG_MODE_CONFIG, tmp))
    return MAX30102_ERROR;

  return MAX30102_OK;
}

// ----------------------------------------------------
// SpO2 Configuration
// ----------------------------------------------------
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value)
{
  uint8_t tmp;
  if (MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
    return MAX30102_ERROR;

  // FIX: bits [6:5]
  tmp &= ~(0x03U << 5);
  tmp |= ((Value & 0x03U) << 5);

  if (MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
    return MAX30102_ERROR;

  return MAX30102_OK;
}

MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value)
{
  uint8_t tmp;
  if (MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
    return MAX30102_ERROR;

  // FIX: bits [4:2]
  tmp &= ~(0x07U << 2);
  tmp |= ((Value & 0x07U) << 2);

  if (MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
    return MAX30102_ERROR;

  return MAX30102_OK;
}

MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value)
{
  uint8_t tmp;
  if (MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
    return MAX30102_ERROR;

  // bits [1:0]
  tmp &= ~(0x03U);
  tmp |= (Value & 0x03U);

  if (MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
    return MAX30102_ERROR;

  return MAX30102_OK;
}

// ----------------------------------------------------
// LED Pulse Amplitude (current)
// LED current = Value * 0.2mA
// ----------------------------------------------------
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value)
{
  if (MAX30102_OK != Max30102_WriteReg(REG_LED1_PA, Value))
    return MAX30102_ERROR;
  return MAX30102_OK;
}

MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value)
{
  if (MAX30102_OK != Max30102_WriteReg(REG_LED2_PA, Value))
    return MAX30102_ERROR;
  return MAX30102_OK;
}

// ----------------------------------------------------
// Usage API
// ----------------------------------------------------
uint8_t Max30102_IsFingerOnSensor(void)
{
  return IsFingerOnScreen;
}

int32_t Max30102_GetHeartRate(void)
{
  return (IsHrValid ? HeartRate : 0);
}

int32_t Max30102_GetSpO2Value(void)
{
  return (Sp02IsValid ? Sp02Value : 0);
}

void Max30102_Task(void)
{
  switch (StateMachine)
  {
    case MAX30102_STATE_BEGIN:
      HeartRate = 0;
      Sp02Value = 0;
      if (IsFingerOnScreen)
      {
        CollectedSamples = 0;
        BufferTail = BufferHead;

        // Keep high while finger is on (as you had)
        Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_HIGH);
        Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_HIGH);

        StateMachine = MAX30102_STATE_CALIBRATE;
      }
      break;

    case MAX30102_STATE_CALIBRATE:
      if (IsFingerOnScreen)
      {
        if (CollectedSamples > (MAX30102_BUFFER_LENGTH - MAX30102_SAMPLES_PER_SECOND))
          StateMachine = MAX30102_STATE_CALCULATE_HR;
      }
      else
      {
        Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
        Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
        StateMachine = MAX30102_STATE_BEGIN;
      }
      break;

    case MAX30102_STATE_CALCULATE_HR:
      if (IsFingerOnScreen)
      {
        maxim_heart_rate_and_oxygen_saturation(
          IrBuffer,
          RedBuffer,
          (MAX30102_BUFFER_LENGTH - MAX30102_SAMPLES_PER_SECOND),
          (uint16_t)BufferTail,
          &Sp02Value,
          &Sp02IsValid,
          &HeartRate,
          &IsHrValid
        );

        BufferTail = (BufferTail + MAX30102_SAMPLES_PER_SECOND) % MAX30102_BUFFER_LENGTH;
        CollectedSamples = 0;
        StateMachine = MAX30102_STATE_COLLECT_NEXT_PORTION;
      }
      else
      {
        Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
        Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
        StateMachine = MAX30102_STATE_BEGIN;
      }
      break;

    case MAX30102_STATE_COLLECT_NEXT_PORTION:
      if (IsFingerOnScreen)
      {
        if (CollectedSamples > MAX30102_SAMPLES_PER_SECOND)
          StateMachine = MAX30102_STATE_CALCULATE_HR;
      }
      else
      {
        Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
        Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
        StateMachine = MAX30102_STATE_BEGIN;
      }
      break;

    default:
      StateMachine = MAX30102_STATE_BEGIN;
      break;
  }
}

// ----------------------------------------------------
// Initialization
// ----------------------------------------------------
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c)
{
  uint8_t uch_dummy;

  i2c_max30102 = i2c;

  // clear globals
  BufferHead = 0;
  BufferTail = 0;
  CollectedSamples = 0;
  IsFingerOnScreen = 0;
  Sp02Value = 0;
  Sp02IsValid = 0;
  HeartRate = 0;
  IsHrValid = 0;

  if (MAX30102_OK != Max30102_Reset())
    return MAX30102_ERROR;

  if (MAX30102_OK != Max30102_ReadReg(0, &uch_dummy))
    return MAX30102_ERROR;

  if (MAX30102_OK != Max30102_FifoWritePointer(0x00))
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_FifoOverflowCounter(0x00))
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_FifoReadPointer(0x00))
    return MAX30102_ERROR;

  if (MAX30102_OK != Max30102_FifoSampleAveraging(FIFO_SMP_AVE_1))
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_FifoRolloverEnable(0))
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_FifoAlmostFullValue(MAX30102_FIFO_ALMOST_FULL_SAMPLES))
    return MAX30102_ERROR;

  if (MAX30102_OK != Max30102_SetMode(MODE_SPO2_MODE))
    return MAX30102_ERROR;

  if (MAX30102_OK != Max30102_SpO2AdcRange(SPO2_ADC_RGE_4096))
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_SpO2SampleRate(SPO2_SAMPLE_RATE))
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_SpO2LedPulseWidth(SPO2_PULSE_WIDTH_411))
    return MAX30102_ERROR;

  // IMPORTANT: start with a non-tiny current so finger detect can work
  // (You can tune these later.)
  if (MAX30102_OK != Max30102_Led1PulseAmplitude(0x24)) // RED
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_Led2PulseAmplitude(0x24)) // IR
    return MAX30102_ERROR;

  if (MAX30102_OK != Max30102_SetIntAlmostFullEnabled(1))
    return MAX30102_ERROR;
  if (MAX30102_OK != Max30102_SetIntFifoDataReadyEnabled(1))
    return MAX30102_ERROR;

  StateMachine = MAX30102_STATE_BEGIN;
  return MAX30102_OK;
}
