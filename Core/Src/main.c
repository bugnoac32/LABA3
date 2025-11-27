#include "main.h"
#include <string.h>
#include <stdio.h>

#define MPU6500_ADDR_LOW    (0x68u << 1)
#define MPU6500_ADDR_HIGH   (0x69u << 1)

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_WHO_AM_I        0x75
#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_XOUT_H     0x43

#define GYRO_SENS_500DPS    (65.5f)

I2C_HandleTypeDef hi2c2;

static uint16_t i2c_timeout = 100u;
static uint8_t  mpu_addr    = 0u;

volatile uint8_t  who_am_i   = 0u;

volatile int16_t  gyro_x     = 0;
volatile int16_t  gyro_y     = 0;
volatile int16_t  gyro_z     = 0;

volatile float    gyro_dps_x = 0.0f;
volatile float    gyro_dps_y = 0.0f;
volatile float    gyro_dps_z = 0.0f;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);

static HAL_StatusTypeDef mpu6500_write(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef mpu6500_read(uint8_t reg, uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef mpu6500_detect(void);
static HAL_StatusTypeDef mpu6500_init_variant9(void);
static void               mpu6500_update_gyro(void);
static void               mpu6500_log_gyro_swo(void);

static inline int  ITM_SendChar_(int ch);
static inline void swo_print(const char *s);

static HAL_StatusTypeDef mpu6500_write(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c2,
                           mpu_addr,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &val,
                           1u,
                           i2c_timeout);
}

static HAL_StatusTypeDef mpu6500_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2,
                          mpu_addr,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          len,
                          i2c_timeout);
}

static HAL_StatusTypeDef mpu6500_detect(void)
{
  uint8_t id = 0u;

  mpu_addr = MPU6500_ADDR_LOW;
  if (HAL_I2C_IsDeviceReady(&hi2c2, mpu_addr, 2u, i2c_timeout) == HAL_OK)
  {
    if (mpu6500_read(REG_WHO_AM_I, &id, 1u) == HAL_OK)
    {
      who_am_i = id;
      return HAL_OK;
    }
  }

  mpu_addr = MPU6500_ADDR_HIGH;
  if (HAL_I2C_IsDeviceReady(&hi2c2, mpu_addr, 2u, i2c_timeout) == HAL_OK)
  {
    if (mpu6500_read(REG_WHO_AM_I, &id, 1u) == HAL_OK)
    {
      who_am_i = id;
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}

static HAL_StatusTypeDef mpu6500_init_variant9(void)
{
  HAL_StatusTypeDef st;
  uint8_t cfg;

  st = mpu6500_write(REG_PWR_MGMT_1, 0x01u);
  if (st != HAL_OK) return st;
  HAL_Delay(10u);

  st = mpu6500_write(REG_CONFIG, 0x00u);
  if (st != HAL_OK) return st;

  st = mpu6500_read(REG_GYRO_CONFIG, &cfg, 1u);
  if (st != HAL_OK) return st;

  cfg &= (uint8_t)~0x18u;
  cfg |= (uint8_t)(1u << 3);

  cfg &= (uint8_t)~0x03u;
  cfg |= (uint8_t)0x03u;

  st = mpu6500_write(REG_GYRO_CONFIG, cfg);
  if (st != HAL_OK) return st;

  st = mpu6500_write(REG_SMPLRT_DIV, 18u);
  if (st != HAL_OK) return st;

  return HAL_OK;
}

static void mpu6500_update_gyro(void)
{
  uint8_t data[6];

  if (mpu6500_read(REG_GYRO_XOUT_H, data, sizeof(data)) == HAL_OK)
  {
    gyro_x = (int16_t)((data[0] << 8) | data[1]);
    gyro_y = (int16_t)((data[2] << 8) | data[3]);
    gyro_z = (int16_t)((data[4] << 8) | data[5]);

    gyro_dps_x = (float)gyro_x / GYRO_SENS_500DPS;
    gyro_dps_y = (float)gyro_y / GYRO_SENS_500DPS;
    gyro_dps_z = (float)gyro_z / GYRO_SENS_500DPS;
  }
}

static void mpu6500_log_gyro_swo(void)
{
  static uint32_t t_prev = 0u;
  uint32_t now = HAL_GetTick();

  if (now - t_prev > 200u)
  {
    t_prev = now;
    char buf[120];
    int n = snprintf(buf, sizeof(buf),
                     "gx=%6d gy=%6d gz=%6d  dps=[%7.2f %7.2f %7.2f]\n",
                     gyro_x, gyro_y, gyro_z,
                     (double)gyro_dps_x,
                     (double)gyro_dps_y,
                     (double)gyro_dps_z);
    if (n > 0) swo_print(buf);
  }
}

static inline int ITM_SendChar_(int ch)
{
  volatile uint32_t *ITM_STIM0 = (uint32_t*)0xE0000000;
  volatile uint32_t *ITM_TCR   = (uint32_t*)0xE0000E80;
  volatile uint32_t *ITM_ENA   = (uint32_t*)0xE0000E00;

  if ((*ITM_TCR & 1u) == 0u) return -1;
  if ((*ITM_ENA & 1u) == 0u) return -1;

  while ((*ITM_STIM0 & 1u) == 0u)
  {
    __NOP();
  }

  *(volatile char*)ITM_STIM0 = (char)ch;
  return ch;
}

static inline void swo_print(const char *s)
{
  while (*s != '\0')
  {
    ITM_SendChar_(*s++);
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();

  if (mpu6500_detect() != HAL_OK)
  {
    swo_print("MPU6500 not found!\n");
    Error_Handler();
  }
  else
  {
    char msg[48];
    (void)snprintf(msg, sizeof(msg),
                   "WHO_AM_I=0x%02X\n",
                   (unsigned int)who_am_i);
    swo_print(msg);
  }

  if (mpu6500_init_variant9() != HAL_OK)
  {
    swo_print("MPU6500 init failed!\n");
    Error_Handler();
  }
  else
  {
    swo_print("MPU6500 init OK (±500 dps, ~421 Hz)\n");
  }

  while (1)
  {
    mpu6500_update_gyro();
    mpu6500_log_gyro_swo();
    HAL_Delay(10u);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                     RCC_CLOCKTYPE_SYSCLK|
                                     RCC_CLOCKTYPE_PCLK1 |
                                     RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance             = I2C2;
  hi2c2.Init.ClockSpeed      = 100000u;
  hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1     = 0u;
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2     = 0u;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
