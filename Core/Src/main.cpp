/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "n2k_raw_bridge.h"
#include "devicelist_handler.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void bridge_uart_print(const char *msg) {
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 100u);
}

/* ── UART trace for identity PGNs only ─────────────────────────────────
 * Logs ISO Request (59904) and Address Claim (60928) lines.
 * Telemetry PGNs (WIND, POS, COG_SOG, TEMP, ...) are intentionally NOT
 * decoded here \u2014 the 5-second [STATS] heartbeat in the main loop
 * confirms bus health without flooding UART, and the Flutter app is
 * the canonical telemetry decoder.
 *
 * If you need on-target per-PGN debug for a new sensor, wrap a decoder
 * in a per-PGN rate-limiter (mirroring the s_prop_count pattern that
 * used to live here for PGN 126720) so it never floods at full sensor
 * rate. Don't reintroduce always-on decoders. */
static void bridge_log_n2k_rx_event(const N2K_RawBridgeRxEvent_t *event) {
  if ((event->pgn != 60928u) &&  /* AddressClaim */
      (event->pgn != 59904u)) {  /* ISO Request  */
    return;
  }

  char line[180];
  const uint8_t *d  = event->data;
  uint8_t        dlc = event->dlc;
  unsigned       src = (unsigned)event->src;
  unsigned       dst = (unsigned)event->dst;

  if (event->pgn == 60928u) {
    /* PGN 60928 \u2014 Address Claim (8 bytes = NMEA2000 NAME, little-endian). */
    if (dlc < 8u) {
      return;
    }
    uint64_t name64 = 0u;
    for (uint8_t i = 0u; i < 8u; i++) {
      name64 |= ((uint64_t)d[i]) << (8u * i);
    }
    uint16_t mfg = (uint16_t)((name64 >> 21u) & 0x7FFu);
    uint8_t  fn  = (uint8_t)((name64 >> 40u) & 0xFFu);
    uint8_t  cls = (uint8_t)(((name64 >> 48u) & 0xFFu) >> 1u);
    (void)snprintf(line, sizeof(line),
      "ADDR_CLAIM  src=%2u  mfg=%u  class=%u  fn=%u  "
      "name=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
      src, (unsigned)mfg, (unsigned)cls, (unsigned)fn,
      d[7], d[6], d[5], d[4], d[3], d[2], d[1], d[0]);
  } else {
    /* PGN 59904 \u2014 ISO Request: 3-byte requested-PGN payload. */
    uint32_t req_pgn = 0u;
    if (dlc >= 3u) {
      req_pgn = (uint32_t)d[0] | ((uint32_t)d[1] << 8u) | ((uint32_t)d[2] << 16u);
    }
    (void)snprintf(line, sizeof(line),
      "ISO_REQUEST src=%2u dst=%3u req_pgn=%lu\r\n",
      src, dst, (unsigned long)req_pgn);
  }

  bridge_uart_print(line);
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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  bridge_uart_print("N2K bridge boot\r\n");

  N2K_RawBridge_Init(&hcan, &hspi1);
  bridge_uart_print("N2K bridge init ok\r\n");
  DeviceListHandler_Init();
  bridge_uart_print("DeviceListHandler init ok\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    N2K_RawBridgeRxEvent_t event;
    N2K_RawBridgeAssembledEvent_t assembled;

    N2K_RawBridge_Process();

    DeviceListHandler_PollUart();
    DeviceListHandler_PollState();

    while (N2K_RawBridge_PopLogEvent(&event) != 0u) {
      bridge_log_n2k_rx_event(&event);
      DeviceListHandler_OnRxEvent(&event);
    }

    while (N2K_RawBridge_PopAssembledEvent(&assembled) != 0u) {
      DeviceListHandler_OnAssembledEvent(&assembled);
    }

    /* No idle delay here — any HAL_Delay() in the main loop starves the
     * CAN RX ring buffer (128 slots, 91 frames/s) during SPI bursts.
     * The inter-frame pacing inside N2K_RawBridge_Process is sufficient. */

    /* Periodic CAN/SPI stats heartbeat — printed every 5 s.
     * can_rx=0 after bus activity → MCU is not receiving CAN frames.
     * can_rx>0 but no PGN lines → rx-event path is broken. */
    {
      static uint32_t s_last_stats_ms = 0u;
      uint32_t now_ms = HAL_GetTick();
      if ((now_ms - s_last_stats_ms) >= 5000u) {
        s_last_stats_ms = now_ms;
        N2K_RawBridgeStats_t   st  = N2K_RawBridge_GetStats();
        N2K_RawBridgePgnDebug_t dbg = N2K_RawBridge_GetPgnDebug();
        char line[200];
        (void)snprintf(line, sizeof(line),
          "[STATS] t=%lus can_rx=%lu ovf=%lu spi_tx=%lu spi_rx=%lu "
          "parse_errs=%lu last_pgn=%lu last_src=%u updates=%lu\r\n",
          (unsigned long)(now_ms / 1000u),
          (unsigned long)st.can_rx_frames,
          (unsigned long)st.can_rx_overflow,
          (unsigned long)st.spi_tx_frames,
          (unsigned long)st.spi_rx_packets,
          (unsigned long)st.spi_parse_errors,
          (unsigned long)dbg.last_can_rx_pgn,
          (unsigned)dbg.last_can_rx_src,
          (unsigned long)dbg.can_rx_pgn_updates);
        bridge_uart_print(line);
      }
    }

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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" void Error_Handler(void)
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
extern "C" void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
