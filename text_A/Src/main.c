/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PAW3395DM-T6QU 光学导航传感器例程（STM32F103RCT6 + HAL）
  *
  * 功能：SPI1 读传感器位移 → 通过 USART1 以 15 字节二进制帧发给上位机
  *       上位机：Host/paw3395_monitor.py
  *
  * 接线：SCLK=PA5  MISO=PA6  MOSI=PA7  NCS=PA4  NRESET=PA3  MOTION=PA2(可选)
  *       USART1_TX=PA9  USART1_RX=PA10
  *
  * 寄存器与时序依据 PAW3395DM-T6QU Datasheet v1.3（08 Apr 2024）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 初始化寄存器表元素：{寄存器地址, 写入值} */
typedef struct
{
  uint8_t reg;
  uint8_t val;
} PAW_RegVal_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ======================= PAW3395 例程可修改配置 ======================= */
/* --- 引脚（按实际接线改） --- */
#define PAW_NCS_PORT        GPIOA
#define PAW_NCS_PIN         GPIO_PIN_4
#define PAW_NRESET_PORT     GPIOA
#define PAW_NRESET_PIN      GPIO_PIN_3

/* --- 传感器寄存器地址（Bank0；先写 0x7F 可切换扩展页） --- */
#define PAW_REG_PRODUCT_ID      0x00
#define PAW_REG_MOTION          0x02
#define PAW_REG_DELTA_X_L       0x03
#define PAW_REG_DELTA_X_H       0x04
#define PAW_REG_DELTA_Y_L       0x05
#define PAW_REG_DELTA_Y_H       0x06
#define PAW_REG_MOTION_BURST    0x16
#define PAW_REG_POWER_UP_RESET  0x3A
#define PAW_REG_SET_RESOLUTION  0x47
#define PAW_REG_RES_X_LOW       0x48
#define PAW_REG_RES_X_HIGH      0x49
#define PAW_REG_RES_Y_LOW       0x4A
#define PAW_REG_RES_Y_HIGH      0x4B
#define PAW_REG_BANK_SEL        0x7F
#define PAW_PRODUCT_ID_VALUE    0x51    /* 读 0x00 应得 0x51，判断通信是否正常 */

/* --- SPI 时钟分频 ---
 * 72MHz / 16 = 4.5MHz（手册上限 10MHz）。
 * 调试阶段若一直读不到 PRODUCT_ID = 0x51，先降速试：
 *   SPI_BAUDRATEPRESCALER_64  → 1.125MHz
 *   SPI_BAUDRATEPRESCALER_128 → 562kHz
 *   SPI_BAUDRATEPRESCALER_256 → 281kHz
 */
#define PAW_SPI_PRESCALER       SPI_BAUDRATEPRESCALER_16

/* --- SPI 时序参数（单位 us / ms，注释为手册要求的最小值） --- */
#define PAW_SPI_TIMEOUT_MS      5       /* SPI 阻塞超时 */
#define PAW_T_NCS_SCLK_US       1       /* NCS 拉低→第一个 SCLK   ≥120ns */
#define PAW_T_SRAD_US           5       /* 读：地址字节→数据字节   ≥2us   */
#define PAW_T_SRAD_BURST_US     35      /* Burst：0x16→读数据      ≥2us   */
#define PAW_T_SCLK_NCS_US       1       /* 最后 SCLK→NCS 拉高（写）≥1us   */
#define PAW_T_BEXIT_US          1       /* Burst 结束 NCS 保持高   ≥500ns */
#define PAW_T_BETWEEN_US        5       /* 两条写命令最小间隔      ≥5us   */
#define PAW_INIT_SETTLE_MS      50      /* 上电等待                ≥50ms  */
#define PAW_RESET_WAIT_MS       10      /* 软复位后等待            ≥5ms   */
#define PAW_CAL_POLL_MAX        60      /* 校准轮询 0x6C 次数（每次 1ms） */
#define PAW_CAL_ROUND_MAX       3       /* 校准轮询轮数（每轮 60 次，含手册 142 恢复流程） */
#define PAW_RETRY_DELAY_MS      500     /* 初始化失败后的重试间隔 */
#define PAW_GIVEUP_ATTEMPTS     10      /* ID 正确但校准一直不过时，失败多少次后先继续跑 */

/* --- 上报参数 --- */
/* 分辨率(CPI) = 灵敏度，也就是"移动 1mm 坐标变化多少计数"：
 *     坐标变化量 = CPI / 25.4  （计数/mm）
 *        400 CPI → 16 计数/mm      800 CPI → 31 计数/mm
 * 常用档位：400 / 800 / 1600 / 3200 / 5000（范围 50~26000，步进 50）
 * 注意：CPI 越低，1 个计数代表的物理距离越大，抗零漂死区换算出的物理尺寸也越大。 */
#define PAW_DEFAULT_CPI         800
#define PAW_POLL_PERIOD_MS      1       /* 采样周期 ms（1 → 传感器 1000 次/秒） */
#define PAW_REPORT_PERIOD_MS    1       /* 上报周期 ms（1 → 1000 帧/秒；2 → 500 帧/秒） */
/* 1kHz 时串口占用：15 字节 × 1000 = 15KB/s，460800 波特率(约46KB/s)够用；
 * 若上位机丢帧，可改回 2，或把 MX_USART1_UART_Init() 的波特率提到 921600 */

/* --- 抗零漂参数（详见说明文档第 10 章）---
 * 三级滤波：有效样本门限 → 死区累加 → 静默清零
 *   静止时：噪声累加值永远到不了门限，且会被静默清零，输出恒为 0
 *   移动时：累加值很快越过门限，总位移不丢（慢速运动只是被量化到门限的整数倍）
 */
#define PAW_JITTER_TH           4       /* 死区门限(计数)：净位移不足该值不出数。越大越稳、越"钝"；
                                         * 4 计数 @800CPI ≈ 127µm，@5000CPI ≈ 20µm */
#define PAW_STILL_TIMEOUT_MS    150     /* 连续多少个上报周期没出数就把累加器清零（消除噪声随机游走）。
                                         * 只有比 门限/超时 更慢的运动才会被当噪声丢掉（约 0.85mm/s，人手达不到） */
#define PAW_SQUAL_MIN           10      /* 表面质量门限：SQUAL 低于此值视为无效样本，不累加。0 = 关闭 */

/* --- 上报帧格式：共 15 字节，小端 ---
 *   0    0xAA  帧头1
 *   1    0x55  帧头2
 *   2    10    payload 长度
 *   3    SEQ   帧序号 0~255
 *   4-5  ΔX    int16，自上次上报以来的累计位移
 *   6-7  ΔY    int16
 *   8    SQUAL 表面质量 uint8
 *   9-10 CPI   uint16
 *   11   MOTION 0x02 原始值：bit7=MOT, bit3=Lift_Stat, bit1:0=OP_Mode
 *   12-13 SHUTTER uint16 = (Shutter_Upper<<8)|Shutter_Lower
 *   14   CHK   第 0~13 字节累加和的低 8 位
 */
#define PAW_FRAME_HEAD0         0xAA
#define PAW_FRAME_HEAD1         0x55
#define PAW_PAYLOAD_LEN         10
#define PAW_FRAME_LEN           15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 外设句柄（本工程未在 CubeMX 中勾选 SPI/USART，故在此定义） */
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart1;

/* 传感器本次 Burst 结果 */
static uint8_t  g_burst[12];        /* 原始 12 字节，调试时可直接 watch */
static uint8_t  g_motion;           /* g_burst[0] */
static uint8_t  g_squal;            /* g_burst[6] */
static int16_t  g_dx, g_dy;         /* g_burst[2..5] */
static uint16_t g_shutter;          /* g_burst[10..11] */
static uint16_t g_cpi = PAW_DEFAULT_CPI;
static uint8_t  g_cal_val = 0;      /* 最近一次读到的 0x6C（校准标志）值，供诊断用 */

/* 上报相关 */
static uint8_t  g_txbuf[PAW_FRAME_LEN];
static uint8_t  g_seq = 0;
static int32_t  g_acc_x = 0;
static int32_t  g_acc_y = 0;
static uint32_t g_still_cnt = 0;    /* 连续多少个上报周期没有输出位移（抗零漂用） */
static uint32_t g_last_poll = 0;
static uint32_t g_last_report = 0;

/* 上电初始化寄存器表（Datasheet 6.2 节，第 1~140 项）
 * 写 0x7F 即切换扩展寄存器页，顺序不可打乱 */
static const PAW_RegVal_t g_paw_init1[] =
{
  {0x7F, 0x07}, {0x40, 0x41}, {0x7F, 0x00}, {0x40, 0x80},
  {0x7F, 0x0E}, {0x55, 0x0D}, {0x56, 0x1B}, {0x57, 0xE8}, {0x58, 0xD5},
  {0x7F, 0x14}, {0x42, 0xBC}, {0x43, 0x74}, {0x4B, 0x20}, {0x4D, 0x00}, {0x53, 0x0E},
  {0x7F, 0x05}, {0x43, 0x64}, {0x44, 0x04}, {0x4D, 0x06}, {0x51, 0x40}, {0x53, 0x40},
  {0x55, 0xCA}, {0x5A, 0xE8}, {0x5B, 0xEA}, {0x61, 0x31}, {0x62, 0x64}, {0x6D, 0xB8},
  {0x6E, 0x0F}, {0x70, 0x02}, {0x4A, 0x2A}, {0x60, 0x26},
  {0x7F, 0x06}, {0x6D, 0x70}, {0x6E, 0x60}, {0x6F, 0x04}, {0x53, 0x02}, {0x55, 0x11},
  {0x7A, 0x01}, {0x7D, 0x51},
  {0x7F, 0x07}, {0x41, 0x10}, {0x42, 0x32}, {0x43, 0x00},
  {0x7F, 0x08}, {0x71, 0x4F},
  {0x7F, 0x09}, {0x62, 0x1F}, {0x63, 0x1F}, {0x65, 0x03}, {0x66, 0x03}, {0x67, 0x1F},
  {0x68, 0x1F}, {0x69, 0x03}, {0x6A, 0x03}, {0x6C, 0x1F}, {0x6D, 0x1F}, {0x51, 0x04},
  {0x53, 0x20}, {0x54, 0x20}, {0x71, 0x0C}, {0x72, 0x07}, {0x73, 0x07},
  {0x7F, 0x0A}, {0x4A, 0x14}, {0x4C, 0x14}, {0x55, 0x19},
  {0x7F, 0x14}, {0x4B, 0x30}, {0x4C, 0x03}, {0x61, 0x0B}, {0x62, 0x0A}, {0x63, 0x02},
  {0x7F, 0x15}, {0x4C, 0x02}, {0x56, 0x02}, {0x41, 0x91}, {0x4D, 0x0A},
  {0x7F, 0x0C}, {0x4A, 0x10}, {0x4B, 0x0C}, {0x4C, 0x40},
  {0x41, 0x25}, {0x55, 0x18}, {0x56, 0x14}, {0x49, 0x0A}, {0x42, 0x00}, {0x43, 0x2D},
  {0x44, 0x0C}, {0x54, 0x1A}, {0x5A, 0x0D}, {0x5F, 0x1E}, {0x5B, 0x05}, {0x5E, 0x0F},
  {0x61, 0x22}, {0x62, 0x41},
  {0x7F, 0x0D}, {0x48, 0xDD}, {0x4F, 0x03}, {0x52, 0x49}, {0x51, 0x00}, {0x54, 0x5B},
  {0x53, 0x00}, {0x56, 0x64}, {0x55, 0x00}, {0x58, 0xA5}, {0x57, 0x02}, {0x5A, 0x29},
  {0x5B, 0x47}, {0x5C, 0x81}, {0x5D, 0x40}, {0x71, 0xDC}, {0x70, 0x07}, {0x73, 0x00},
  {0x72, 0x08}, {0x75, 0xDC}, {0x74, 0x07}, {0x77, 0x00}, {0x76, 0x08},
  {0x7F, 0x10}, {0x4C, 0xD0},
  {0x7F, 0x00}, {0x4F, 0x63}, {0x4E, 0x00}, {0x52, 0x63}, {0x51, 0x00}, {0x54, 0x54},
  {0x5A, 0x10}, {0x77, 0x4F}, {0x47, 0x01}, {0x5B, 0x40}, {0x64, 0x60}, {0x65, 0x06},
  {0x66, 0x13}, {0x67, 0x0F}, {0x78, 0x01}, {0x79, 0x9C}, {0x40, 0x00}, {0x55, 0x02},
  {0x23, 0x70}, {0x22, 0x01},
};

/* 校准完成后的收尾寄存器表（Datasheet 6.2 节，第 143~148 项） */
static const PAW_RegVal_t g_paw_init2[] =
{
  {0x22, 0x00}, {0x55, 0x00}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x00}, {0x68, 0x01},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

static void     PAW_DelayUs(uint32_t us);
static void     PAW_WriteReg(uint8_t addr, uint8_t val);
static uint8_t  PAW_ReadReg(uint8_t addr);
static HAL_StatusTypeDef PAW_BurstRead(void);
static HAL_StatusTypeDef PAW_Init(void);
static void     PAW_SetCpi(uint16_t cpi);
static uint8_t  PAW_GetProductId(void);
static uint8_t  PAW_SampleValid(void);
static void     PAW_ReportSend(void);

/* 诊断用串口文本输出 */
static void     UartSendStr(const char *s);
static void     UartSendHex8(uint8_t v);

/* USER CODE END PFP */

/* User code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*----------------------------------------------------------------------------
 * 诊断输出：往 USART1 发纯文本（上位机解析时会自动跳过非帧数据）
 *--------------------------------------------------------------------------*/
static void UartSendStr(const char *s)
{
  uint16_t len = 0;

  while (s[len] != '\0')
  {
    len++;
  }
  if (len > 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, len, 100);
  }
}

static void UartSendHex8(uint8_t v)
{
  const char hex[] = "0123456789ABCDEF";
  char b[2];

  b[0] = hex[(v >> 4) & 0x0F];
  b[1] = hex[v & 0x0F];
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, 2, 100);
}

/*----------------------------------------------------------------------------
 * 微秒延时：基于 DWT 周期计数器（Cortex-M3 内核自带）
 *--------------------------------------------------------------------------*/
static void PAW_DelayUs(uint32_t us)
{
  uint32_t start  = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

/*---------------------------------------------------------------------------
 * 写单个寄存器：NCS 低 → {地址|0x80, 数据} → NCS 高
 *-------------------------------------------------------------------------*/
static void PAW_WriteReg(uint8_t addr, uint8_t val)
{
  uint8_t buf[2];

  buf[0] = (uint8_t)(addr | 0x80);
  buf[1] = val;

  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_RESET);
  PAW_DelayUs(PAW_T_NCS_SCLK_US);

  (void)HAL_SPI_Transmit(&hspi1, buf, 2, PAW_SPI_TIMEOUT_MS);

  PAW_DelayUs(PAW_T_SCLK_NCS_US);
  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_SET);
  PAW_DelayUs(PAW_T_BETWEEN_US);
}

/*---------------------------------------------------------------------------
 * 读单个寄存器：NCS 低 → 地址(MSB=0) → tSRAD → 读 1 字节 → NCS 高
 *-------------------------------------------------------------------------*/
static uint8_t PAW_ReadReg(uint8_t addr)
{
  uint8_t tx = (uint8_t)(addr & 0x7F);
  uint8_t rx = 0;

  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_RESET);
  PAW_DelayUs(PAW_T_NCS_SCLK_US);

  (void)HAL_SPI_Transmit(&hspi1, &tx, 1, PAW_SPI_TIMEOUT_MS);
  PAW_DelayUs(PAW_T_SRAD_US);
  (void)HAL_SPI_Receive(&hspi1, &rx, 1, PAW_SPI_TIMEOUT_MS);

  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_SET);
  PAW_DelayUs(PAW_T_BETWEEN_US);

  return rx;
}

/*---------------------------------------------------------------------------
 * Motion Burst：NCS 低 → 0x16 → tSRAD → 连续读 12 字节 → NCS 高
 * 字节顺序：Motion / Observation / DX_L / DX_H / DY_L / DY_H /
 *          SQUAL / RawSum / RawMax / RawMin / Shutter_Upper / Shutter_Lower
 *-------------------------------------------------------------------------*/
static HAL_StatusTypeDef PAW_BurstRead(void)
{
  uint8_t cmd = PAW_REG_MOTION_BURST;

  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_RESET);
  PAW_DelayUs(PAW_T_NCS_SCLK_US);

  if (HAL_SPI_Transmit(&hspi1, &cmd, 1, PAW_SPI_TIMEOUT_MS) != HAL_OK)
  {
    HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_SET);
    return HAL_ERROR;
  }

  PAW_DelayUs(PAW_T_SRAD_BURST_US);

  if (HAL_SPI_Receive(&hspi1, g_burst, 12, PAW_SPI_TIMEOUT_MS) != HAL_OK)
  {
    HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_SET);
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_SET);
  PAW_DelayUs(PAW_T_BEXIT_US);

  g_motion  = g_burst[0];
  g_dx      = (int16_t)((uint16_t)g_burst[2] | ((uint16_t)g_burst[3] << 8));
  g_dy      = (int16_t)((uint16_t)g_burst[4] | ((uint16_t)g_burst[5] << 8));
  g_squal   = g_burst[6];
  g_shutter = (uint16_t)(((uint16_t)g_burst[10] << 8) | (uint16_t)g_burst[11]);

  return HAL_OK;
}

/*---------------------------------------------------------------------------
 * 上电初始化：软复位 → 140 条寄存器 → 轮询 0x6C 校准 → 收尾 6 条 → 校验 ID
 *-------------------------------------------------------------------------*/
static HAL_StatusTypeDef PAW_Init(void)
{
  uint32_t i;
  uint8_t  round;
  uint8_t  ok = 0;

  /* DWT 周期计数器，供 PAW_DelayUs 使用 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* 1. 释放复位，等待电源稳定 */
  HAL_GPIO_WritePin(PAW_NRESET_PORT, PAW_NRESET_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_SET);
  HAL_Delay(PAW_INIT_SETTLE_MS);

  /* 2. NCS 高→低，复位 SPI 端口 */
  PAW_DelayUs(PAW_T_BEXIT_US);
  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_RESET);
  PAW_DelayUs(PAW_T_NCS_SCLK_US);
  HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN, GPIO_PIN_SET);
  PAW_DelayUs(PAW_T_BEXIT_US);

  /* 3. 软复位 */
  PAW_WriteReg(PAW_REG_POWER_UP_RESET, 0x5A);
  HAL_Delay(PAW_RESET_WAIT_MS);

  /* 4. 写入初始化寄存器表 */
  for (i = 0; i < (sizeof(g_paw_init1) / sizeof(g_paw_init1[0])); i++)
  {
    PAW_WriteReg(g_paw_init1[i].reg, g_paw_init1[i].val);
  }

  /* 5. 轮询 0x6C，读到 0x80 表示校准完成
   *    手册 142：1ms 间隔最多读 60 次；若读不到，按 142.a/b/c 清 0x6C 后重来
   *    这里最多做 PAW_CAL_ROUND_MAX 轮，每轮 60 次 × 1ms
   *    每次读到的值记录在 g_cal_val，便于失败时打印出来定位 */
  for (round = 0; round < PAW_CAL_ROUND_MAX; round++)
  {
    HAL_Delay(1);
    for (i = 0; i < PAW_CAL_POLL_MAX; i++)
    {
      g_cal_val = PAW_ReadReg(0x6C);
      if (g_cal_val == 0x80)
      {
        ok = 1;
        break;
      }
      HAL_Delay(1);
    }
    if (ok != 0)
    {
      break;
    }

    /* 手册 142 失败分支：0x7F=0x14 → 0x6C=0x00 → 0x7F=0x00 */
    PAW_WriteReg(PAW_REG_BANK_SEL, 0x14);
    PAW_WriteReg(0x6C, 0x00);
    PAW_WriteReg(PAW_REG_BANK_SEL, 0x00);
    HAL_Delay(50);
  }

  /* 6. 收尾寄存器 */
  for (i = 0; i < (sizeof(g_paw_init2) / sizeof(g_paw_init2[0])); i++)
  {
    PAW_WriteReg(g_paw_init2[i].reg, g_paw_init2[i].val);
  }

  /* 7. 上电后把 0x02/0x03/0x04/0x05/0x06 各读一次，清掉残留运动数据 */
  (void)PAW_ReadReg(PAW_REG_MOTION);
  (void)PAW_ReadReg(PAW_REG_DELTA_X_L);
  (void)PAW_ReadReg(PAW_REG_DELTA_X_H);
  (void)PAW_ReadReg(PAW_REG_DELTA_Y_L);
  (void)PAW_ReadReg(PAW_REG_DELTA_Y_H);

  /* 8. 校验通信：读器件 ID 应为 0x51 */
  if (PAW_GetProductId() != PAW_PRODUCT_ID_VALUE)
  {
    return HAL_ERROR;
  }

  return (ok != 0) ? HAL_OK : HAL_TIMEOUT;
}

static uint8_t PAW_GetProductId(void)
{
  return PAW_ReadReg(PAW_REG_PRODUCT_ID);
}

/*---------------------------------------------------------------------------
 * 设置分辨率 50~26000 CPI：寄存器值 = CPI/50 - 1，最后写 0x47=0x01 生效
 * （0x0000=50CPI，0x0063=5000CPI 默认，0x0207=26000CPI 最大）
 *-------------------------------------------------------------------------*/
static void PAW_SetCpi(uint16_t cpi)
{
  uint16_t v;

  if (cpi < 50)
  {
    cpi = 50;
  }
  if (cpi > 26000)
  {
    cpi = 26000;
  }
  cpi = (uint16_t)((cpi / 50) * 50);
  v = (uint16_t)(cpi / 50) - 1;

  PAW_WriteReg(PAW_REG_RES_X_LOW,  (uint8_t)(v & 0xFF));
  PAW_WriteReg(PAW_REG_RES_X_HIGH, (uint8_t)(v >> 8));
  PAW_WriteReg(PAW_REG_RES_Y_LOW,  (uint8_t)(v & 0xFF));
  PAW_WriteReg(PAW_REG_RES_Y_HIGH, (uint8_t)(v >> 8));
  PAW_WriteReg(PAW_REG_SET_RESOLUTION, 0x01);

  g_cpi = cpi;
}

/*---------------------------------------------------------------------------
 * 抗零漂 第 1 级：判断本次 Burst 样本是否有效
 *   MOT(bit7)=1  有位移
 *   Lift(bit3)=0 没有抬升
 *   SQUAL 够高   表面纹理可靠（SQUAL 过低时的位移基本是噪声）
 *-------------------------------------------------------------------------*/
static uint8_t PAW_SampleValid(void)
{
  if ((g_motion & 0x80) == 0)
  {
    return 0;
  }
  if ((g_motion & 0x08) != 0)
  {
    return 0;
  }
  if ((PAW_SQUAL_MIN > 0) && (g_squal < PAW_SQUAL_MIN))
  {
    return 0;
  }
  return 1;
}

/*---------------------------------------------------------------------------
 * 通过 USART1 发送一帧（15 字节）
 *
 * 抗零漂 第 2、3 级在这里做：
 *   第2级 死区累加：净位移 |acc| 不足 PAW_JITTER_TH 就不出数（出数后清零，
 *          误差被限制在 ±PAW_JITTER_TH 计数内且不会累积）
 *   第3级 静默清零：连续 PAW_STILL_TIMEOUT_MS 个周期都没出数，说明累加器里
 *          攒的是零漂噪声，直接清零，避免噪声随机游走慢慢"漂"出去
 *-------------------------------------------------------------------------*/
static void PAW_ReportSend(void)
{
  int32_t out_x = 0;
  int32_t out_y = 0;
  uint8_t sum = 0;
  uint8_t i;

  if (g_acc_x >= PAW_JITTER_TH)
  {
    out_x = g_acc_x;
    g_acc_x = 0;
  }
  else if (g_acc_x <= -PAW_JITTER_TH)
  {
    out_x = g_acc_x;
    g_acc_x = 0;
  }

  if (g_acc_y >= PAW_JITTER_TH)
  {
    out_y = g_acc_y;
    g_acc_y = 0;
  }
  else if (g_acc_y <= -PAW_JITTER_TH)
  {
    out_y = g_acc_y;
    g_acc_y = 0;
  }

  /* 帧里 ΔX/ΔY 是 int16，超出范围就饱和（改了上报周期时兜底） */
  if (out_x > 32767)  { out_x = 32767; }
  if (out_x < -32768) { out_x = -32768; }
  if (out_y > 32767)  { out_y = 32767; }
  if (out_y < -32768) { out_y = -32768; }

  if ((out_x == 0) && (out_y == 0))
  {
    if (++g_still_cnt >= PAW_STILL_TIMEOUT_MS)
    {
      g_acc_x = 0;          /* 判定为静止：丢掉攒下来的零漂 */
      g_acc_y = 0;
      g_still_cnt = 0;
    }
  }
  else
  {
    g_still_cnt = 0;
  }

  g_txbuf[0]  = PAW_FRAME_HEAD0;
  g_txbuf[1]  = PAW_FRAME_HEAD1;
  g_txbuf[2]  = PAW_PAYLOAD_LEN;
  g_txbuf[3]  = g_seq++;
  g_txbuf[4]  = (uint8_t)((uint16_t)out_x & 0xFF);
  g_txbuf[5]  = (uint8_t)(((uint16_t)out_x >> 8) & 0xFF);
  g_txbuf[6]  = (uint8_t)((uint16_t)out_y & 0xFF);
  g_txbuf[7]  = (uint8_t)(((uint16_t)out_y >> 8) & 0xFF);
  g_txbuf[8]  = g_squal;
  g_txbuf[9]  = (uint8_t)(g_cpi & 0xFF);
  g_txbuf[10] = (uint8_t)(g_cpi >> 8);
  g_txbuf[11] = g_motion;
  g_txbuf[12] = (uint8_t)(g_shutter & 0xFF);
  g_txbuf[13] = (uint8_t)(g_shutter >> 8);

  for (i = 0; i < (PAW_FRAME_LEN - 1); i++)
  {
    sum = (uint8_t)(sum + g_txbuf[i]);
  }
  g_txbuf[PAW_FRAME_LEN - 1] = sum;

  /* 阻塞发送：15 字节 @460800 约 0.33ms；嫌占用主循环可改 HAL_UART_Transmit_DMA */
  (void)HAL_UART_Transmit(&huart1, g_txbuf, PAW_FRAME_LEN, 5);

  /* 注意：这里不能清 g_acc_x/g_acc_y，未达门限的余量要留到下一帧继续累加 */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* ================== 诊断输出 ==================
   * 1) 上电先发一条横幅：只要串口能收到这行文字，就说明
   *    MCU 在跑、USART1 波特率/接线/COM 口都是对的。
   * 2) 传感器初始化失败时不再死等，而是每 500ms 打印
   *    实际读到的 PRODUCT_ID，并自动重试，方便定位问题。
   *
   *   PRODUCT_ID 读数的含义：
   *     0x51  → 正常
   *     0x00  → MISO 被拉低（接错/短路/传感器没供电）
   *     0xFF  → MISO 悬空（没接传感器、NCS 没拉低、模块没供电）
   *     其他  → 通信时序或电平不匹配
   */
  UartSendStr("\r\n[PAW3395] UART1 460800 8N1 OK, start init...\r\n");

  {
    uint8_t  wr;
    uint32_t attempt = 0;

    while (PAW_Init() != HAL_OK)
    {
      /* 写通道自检：向分辨率寄存器 0x48 写 0x1F 再读回，能原样读回说明写也通了 */
      PAW_WriteReg(PAW_REG_RES_X_LOW, 0x1F);
      wr = PAW_ReadReg(PAW_REG_RES_X_LOW);

      UartSendStr("[PAW3395] FAIL: ID=0x");
      UartSendHex8(PAW_GetProductId());
      UartSendStr(" REG6C=0x");
      UartSendHex8(g_cal_val);
      UartSendStr(" REG48_wr_rd=0x");
      UartSendHex8(wr);
      UartSendStr("\r\n");

      attempt++;

      /* ID 正确说明 SPI 读写都通，只是 0x6C 校准标志没置起来：
       * 试够次数就先放行，让数据流跑起来继续观察（否则会一直卡在这里） */
      if ((PAW_GetProductId() == PAW_PRODUCT_ID_VALUE) && (attempt >= PAW_GIVEUP_ATTEMPTS))
      {
        UartSendStr("[PAW3395] WARN: reg 0x6C never 0x80, continue anyway\r\n");
        break;
      }
      HAL_Delay(PAW_RETRY_DELAY_MS);
    }
  }

  UartSendStr("[PAW3395] streaming, ID=0x");
  UartSendHex8(PAW_GetProductId());
  UartSendStr(" REG6C=0x");
  UartSendHex8(g_cal_val);
  UartSendStr("\r\n");

  PAW_SetCpi(PAW_DEFAULT_CPI);      /* 需要别的分辨率就改这里 */

  g_last_poll = HAL_GetTick();
  g_last_report = g_last_poll;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. 按 PAW_POLL_PERIOD_MS 读一次 Motion Burst */
    if ((uint32_t)(HAL_GetTick() - g_last_poll) >= PAW_POLL_PERIOD_MS)
    {
      g_last_poll = HAL_GetTick();

      if (PAW_BurstRead() == HAL_OK)
      {
        /* 只有有效样本才累加（MOT=1、未抬升、SQUAL 合格） */
        if (PAW_SampleValid() != 0)
        {
          g_acc_x += g_dx;
          g_acc_y += g_dy;
        }
      }
    }

    /* 2. 按 PAW_REPORT_PERIOD_MS 通过 USART1 发一帧 */
    if ((uint32_t)(HAL_GetTick() - g_last_report) >= PAW_REPORT_PERIOD_MS)
    {
      g_last_report = HAL_GetTick();
      PAW_ReportSend();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* PA4=NCS、PA3=NRESET：推挽输出，初始高电平 */
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(PAW_NCS_PORT, PAW_NCS_PIN | PAW_NRESET_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = PAW_NCS_PIN | PAW_NRESET_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA2=MOTION：输入（可选，用不到可删掉这一段） */
    GPIO_InitStruct.Pin  = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*---------------------------------------------------------------------------
 * SPI1 初始化（同时完成时钟、引脚配置，不依赖 MSP）
 *   SCK=PA5  MISO=PA6  MOSI=PA7，NCS 用 PA4 软件控制
 *   Mode 3（CPOL=1/CPHA=1）、8bit、MSB、72MHz/16 = 4.5MHz（手册上限 10MHz）
 *-------------------------------------------------------------------------*/
static void MX_SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_SPI1_CLK_ENABLE();

  /* PA5=SCK、PA7=MOSI 复用推挽输出 */
  GPIO_InitStruct.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA6=MISO 输入 */
  GPIO_InitStruct.Pin  = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;         /* CPOL = 1 */
  hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;           /* CPHA = 1 → Mode 3 */
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = PAW_SPI_PRESCALER;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/*---------------------------------------------------------------------------
 * USART1 初始化（同时完成时钟、引脚配置，不依赖 MSP）
 *   TX=PA9  RX=PA10，460800 8N1
 *-------------------------------------------------------------------------*/
static void MX_USART1_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9=TX 复用推挽输出 */
  GPIO_InitStruct.Pin   = GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA10=RX 输入 */
  GPIO_InitStruct.Pin  = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 460800;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
