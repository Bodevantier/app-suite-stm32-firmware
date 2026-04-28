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

/* ── Raw byte read helpers (little-endian, matching NMEA2000 library) ──── */
static uint16_t n2k_u16(const uint8_t *d, uint8_t o) {
  return (uint16_t)d[o] | ((uint16_t)d[o + 1u] << 8u);
}
static int16_t n2k_s16(const uint8_t *d, uint8_t o) {
  return (int16_t)n2k_u16(d, o);
}
static uint32_t n2k_u32(const uint8_t *d, uint8_t o) {
  return (uint32_t)d[o] | ((uint32_t)d[o+1u] << 8u) |
         ((uint32_t)d[o+2u] << 16u) | ((uint32_t)d[o+3u] << 24u);
}
static int32_t n2k_s32(const uint8_t *d, uint8_t o) {
  return (int32_t)n2k_u32(d, o);
}
/* N2K NA sentinels (from N2kMsg.h) */
static uint8_t n2k_u16_ok(uint16_t v) { return (v != 0xFFFFu); }
static uint8_t n2k_s16_ok(int16_t v)  { return (v != (int16_t)0x7FFF); }
static uint8_t n2k_u32_ok(uint32_t v) { return (v != 0xFFFFFFFFu); }
static uint8_t n2k_s32_ok(int32_t v)  { return (v != (int32_t)0x7FFFFFFF); }

/* ── Value formatters (integer math only — newlib-nano has no %%f) ─────── */

/* Unsigned angle: uint16 × 0.0001 rad → degrees, 1 dp.
   Multiplier: 0.0001 × 57.2958 × 10 ≈ 573/100000 per unit */
static void fmt_udeg(char *b, size_t z, uint16_t v) {
  if (!n2k_u16_ok(v)) { (void)snprintf(b, z, "n/a"); return; }
  int32_t t = (int32_t)v * 573 / 100000;  /* tenths of degrees */
  (void)snprintf(b, z, "%ld.%lddeg", (long)(t / 10), (long)(t % 10));
}

/* Signed angle: int16 × 0.0001 rad → degrees, 1 dp */
static void fmt_sdeg(char *b, size_t z, int16_t v) {
  if (!n2k_s16_ok(v)) { (void)snprintf(b, z, "n/a"); return; }
  int32_t t = (int32_t)v * 573 / 100000;
  long ta = (long)(t < 0 ? -t : t);
  (void)snprintf(b, z, "%ld.%lddeg", (long)(t / 10), ta % 10);
}

/* Speed: uint16 × 0.01 m/s → knots, 1 dp (×1.94384, approx ×1944/100000) */
static void fmt_knots(char *b, size_t z, uint16_t v) {
  if (!n2k_u16_ok(v)) { (void)snprintf(b, z, "n/a"); return; }
  int32_t t = (int32_t)v * 1944 / 10000;  /* tenths of knots */
  (void)snprintf(b, z, "%ld.%ldkt", (long)(t / 10), (long)(t % 10));
}

/* Lat/lon: int32 × 1e-7 deg → 4 decimal places */
static void fmt_latlon(char *b, size_t z, int32_t v) {
  if (!n2k_s32_ok(v)) { (void)snprintf(b, z, "n/a"); return; }
  char sign = '+';
  uint32_t uv;
  if (v < 0) {
    sign = '-';
    uv = (uint32_t)(-v);
  } else {
    uv = (uint32_t)v;
  }
  (void)snprintf(b, z, "%c%lu.%04lu", sign,
                 (unsigned long)(uv / 10000000u),
                 (unsigned long)((uv % 10000000u) / 1000u));
}

/* Temperature: uint16 × 0.01 K → °C, 1 dp */
static void fmt_celsius(char *b, size_t z, uint16_t v) {
  if (!n2k_u16_ok(v)) { (void)snprintf(b, z, "n/a"); return; }
  int32_t c = (int32_t)v - 27315;  /* hundredths of °C */
  long ca = (long)(c < 0 ? -c : c);
  (void)snprintf(b, z, "%ld.%ldC", (long)(c / 100), ca % 100 / 10);
}

/* Depth: uint32 × 0.01 m → metres, 1 dp */
static void fmt_depth(char *b, size_t z, uint32_t v) {
  if (!n2k_u32_ok(v)) { (void)snprintf(b, z, "n/a"); return; }
  (void)snprintf(b, z, "%lu.%lum", (unsigned long)(v / 100),
                 (unsigned long)((v % 100) / 10));
}

/* Signed distance: s32 x 0.01 m -> metres with 2 dp */
static void fmt_smeters_2(char *b, size_t z, int32_t v) {
  if (!n2k_s32_ok(v)) { (void)snprintf(b, z, "n/a"); return; }
  long a = (long)(v < 0 ? -v : v);
  (void)snprintf(b, z, "%ld.%02ldm", (long)(v / 100), (long)(a % 100));
}

/* DOP: s16 × 0.01 (positive) */
static void fmt_dop(char *b, size_t z, int16_t v) {
  if (!n2k_s16_ok(v) || v < 0) { (void)snprintf(b, z, "n/a"); return; }
  (void)snprintf(b, z, "%d.%02d", (int)(v / 100), (int)(v % 100));
}

/* Copy N2K 0xFF-padded fixed-width ASCII string */
static void n2k_copy_str(char *dst, size_t dst_sz,
                         const uint8_t *src, size_t src_len) {
  size_t i;
  for (i = 0u; i + 1u < dst_sz && i < src_len; i++) {
    uint8_t c = src[i];
    if (c == 0xFFu || c == 0x00u) break;
    dst[i] = (char)c;
  }
  dst[i] = '\0';
}

/* ── PGN name lookup ──────────────────────────────────────────────────── */
static const char *bridge_pgn_name(uint32_t pgn) {
  switch (pgn) {
    case  59392u: return "ISO_ACK";
    case  59904u: return "ISO_REQUEST";
    case  60928u: return "ADDRESS_CLAIM";
    case  65240u: return "COMMANDED_ADDR";
    case 126208u: return "GROUP_FUNCTION";
    case 126464u: return "PGN_LIST";
    case 126993u: return "HEARTBEAT";
    case 126996u: return "PRODUCT_INFO";
    case 126998u: return "CONFIG_INFO";
    case 126720u: return "PROPRIETARY";
    case 127250u: return "HEADING";
    case 127251u: return "RATE_OF_TURN";
    case 127257u: return "ATTITUDE";
    case 127258u: return "MAG_VARIATION";
    case 127488u: return "ENGINE_RAPID";
    case 127489u: return "ENGINE_DYNAMIC";
    case 127505u: return "FLUID_LEVEL";
    case 127508u: return "BATTERY";
    case 128259u: return "SPEED";
    case 128267u: return "DEPTH";
    case 128275u: return "DISTANCE_LOG";
    case 129025u: return "POSITION_RAPID";
    case 129026u: return "COG_SOG";
    case 129029u: return "GNSS";
    case 129283u: return "XTE";
    case 129284u: return "NAV_DATA";
    case 129285u: return "ROUTE_WP";
    case 129539u: return "GNSS_DOP";
    case 129540u: return "GNSS_SATS";
    case 130306u: return "WIND";
    case 130310u: return "ENV_PARAMS";
    case 130312u: return "TEMP";
    case 130313u: return "HUMIDITY";
    case 130314u: return "PRESSURE";
    case 130316u: return "TEMP_EXT";
    default:      return NULL;
  }
}

static uint8_t bridge_is_fast_packet(uint32_t pgn) {
  switch (pgn) {
    case  65240u: case 126208u: case 126996u: case 126998u:
    case 129029u: case 129284u: case 129285u: case 129539u:
    case 129540u: return 1u;
    default:      return 0u;
  }
}

/* ── Per-PGN decode & print ───────────────────────────────────────────── */
static void bridge_log_n2k_rx_event(const N2K_RawBridgeRxEvent_t *event) {
  /* Only log identity/device-list PGNs.
   * High-frequency telemetry (WIND, POS, COG_SOG, …) is suppressed here —
   * the 5-second [STATS] heartbeat confirms bus health without flooding. */
  if ((event->pgn != 60928u) &&  /* AddressClaim */
      (event->pgn != 59904u)) {  /* ISO Request  */
    return;
  }

  char line[180];
  const uint8_t *d  = event->data;
  uint32_t       pgn = event->pgn;
  uint8_t        dlc = event->dlc;
  unsigned       src = (unsigned)event->src;
  unsigned       dst = (unsigned)event->dst;

  line[0] = '\0';

  switch (pgn) {

  /* PGN 130306 - Wind  (spd: u16×0.01 m/s, ang: u16×0.0001 rad) */
  case 130306u: {
    if (dlc < 6u) break;
    static const char *refs[] = {"True","Mag","App","BoatRef","WaterRef","?","?","NA"};
    char spd[12], ang[12];
    fmt_knots(spd, sizeof(spd), n2k_u16(d, 1u));
    fmt_udeg (ang, sizeof(ang), n2k_u16(d, 3u));
    (void)snprintf(line, sizeof(line),
      "WIND      spd=%7s  ang=%8s  ref=%-8s  src=%2u\r\n",
      spd, ang, refs[d[5] & 0x07u], src);
    break;
  }

  /* PGN 129026 - COG & SOG rapid  (COG: u16×0.0001 rad, SOG: u16×0.01 m/s) */
  case 129026u: {
    if (dlc < 6u) break;
    char cog[12], sog[12];
    fmt_udeg (cog, sizeof(cog), n2k_u16(d, 2u));
    fmt_knots(sog, sizeof(sog), n2k_u16(d, 4u));
    (void)snprintf(line, sizeof(line),
      "COG_SOG   cog=%8s  sog=%7s  ref=%-4s  src=%2u\r\n",
      cog, sog, (d[1] & 0x03u) == 0u ? "True" : "Mag", src);
    break;
  }

  /* PGN 129025 - Position rapid  (lat/lon: s32×1e-7 deg) */
  case 129025u: {
    if (dlc < 8u) break;
    char lat[14], lon[14];
    fmt_latlon(lat, sizeof(lat), n2k_s32(d, 0u));
    fmt_latlon(lon, sizeof(lon), n2k_s32(d, 4u));
    (void)snprintf(line, sizeof(line),
      "POS       lat=%10s  lon=%10s  src=%2u\r\n", lat, lon, src);
    break;
  }

  /* PGN 129283 - Cross Track Error (s32 x 0.01 m at bytes 2-5) */
  case 129283u: {
    if (dlc < 6u) break;
    char xte[14];
    fmt_smeters_2(xte, sizeof(xte), n2k_s32(d, 2u));
    (void)snprintf(line, sizeof(line),
      "XTE       err=%10s  src=%2u\r\n", xte, src);
    break;
  }

  /* PGN 127250 - Vessel heading  (u16×0.0001 rad, dev/var: s16×0.0001 rad) */
  case 127250u: {
    if (dlc < 7u) break;
    char hdg[12], dev[12], var[12];
    fmt_udeg(hdg, sizeof(hdg), n2k_u16(d, 1u));
    fmt_sdeg(dev, sizeof(dev), n2k_s16(d, 3u));
    fmt_sdeg(var, sizeof(var), n2k_s16(d, 5u));
    (void)snprintf(line, sizeof(line),
      "HDG  hdg=%s  dev=%s  var=%s  ref=%s  src=%u\r\n",
      hdg, dev, var, (d[7] & 0x03u) == 0u ? "True" : "Mag", src);
    break;
  }

  /* PGN 127257 - Attitude  (yaw/pitch/roll: s16×0.0001 rad) */
  case 127257u: {
    if (dlc < 7u) break;
    char yaw[12], pitch[12], roll[12];
    fmt_sdeg(yaw,   sizeof(yaw),   n2k_s16(d, 1u));
    fmt_sdeg(pitch, sizeof(pitch), n2k_s16(d, 3u));
    fmt_sdeg(roll,  sizeof(roll),  n2k_s16(d, 5u));
    (void)snprintf(line, sizeof(line),
      "ATTITUDE  yaw=%s  pitch=%s  roll=%s  src=%u\r\n",
      yaw, pitch, roll, src);
    break;
  }

  /* PGN 127258 - Magnetic variation  (s16×0.0001 rad at bytes 4-5) */
  case 127258u: {
    if (dlc < 6u) break;
    char var[12];
    fmt_sdeg(var, sizeof(var), n2k_s16(d, 4u));
    (void)snprintf(line, sizeof(line),
      "MAGVAR  var=%s  src=%u\r\n", var, src);
    break;
  }

  /* PGN 128267 - Depth  (u32×0.01 m at bytes 1-4) */
  case 128267u: {
    if (dlc < 5u) break;
    char dep[12];
    fmt_depth(dep, sizeof(dep), n2k_u32(d, 1u));
    (void)snprintf(line, sizeof(line),
      "DEPTH  dep=%s  src=%u\r\n", dep, src);
    break;
  }

  /* PGN 128259 - Speed  (u16×0.01 m/s at bytes 1-2) */
  case 128259u: {
    if (dlc < 3u) break;
    char ws[12];
    fmt_knots(ws, sizeof(ws), n2k_u16(d, 1u));
    (void)snprintf(line, sizeof(line),
      "SPEED  water=%s  src=%u\r\n", ws, src);
    break;
  }

  /* PGN 130310 - Env params  (water/air temp: u16×0.01 K; pressure: u16 hPa) */
  case 130310u: {
    if (dlc < 6u) break;
    char wt[12], at[12];
    fmt_celsius(wt, sizeof(wt), n2k_u16(d, 1u));
    fmt_celsius(at, sizeof(at), n2k_u16(d, 3u));
    (void)snprintf(line, sizeof(line),
      "ENV  water=%s  air=%s  pressure=%uhPa  src=%u\r\n",
      wt, at, (unsigned)n2k_u16(d, 5u), src);
    break;
  }

  /* PGN 130312 - Temperature  (u16×0.01 K at bytes 3-4) */
  case 130312u: {
    if (dlc < 5u) break;
    static const char *tsrc[] = {
      "Sea","Outside","Inside","EngineRm","Cabin",
      "LiveWell","BaitWell","Fridge","Heating","DewPoint","?","?","?","?","Exhaust"
    };
    char tmp[12];
    uint8_t tidx = d[2] & 0x0Fu;
    fmt_celsius(tmp, sizeof(tmp), n2k_u16(d, 3u));
    (void)snprintf(line, sizeof(line),
      "TEMP  source=%s  temp=%s  inst=%u  src=%u\r\n",
      tidx < 15u ? tsrc[tidx] : "?", tmp, (unsigned)d[1], src);
    break;
  }

  /* PGN 60928 - Address Claim (8 bytes = NMEA2000 NAME) */
  case 60928u: {
    if (dlc < 8u) break;
    uint64_t name64 = 0u;
    for (uint8_t i = 0u; i < 8u; i++) { name64 |= ((uint64_t)d[i]) << (8u * i); }
    uint16_t mfg = (uint16_t)((name64 >> 21u) & 0x7FFu);
    uint8_t  fn  = (uint8_t)((name64 >> 40u) & 0xFFu);
    uint8_t  cls = (uint8_t)(((name64 >> 48u) & 0xFFu) >> 1u);
    (void)snprintf(line, sizeof(line),
      "ADDR_CLAIM  src=%2u  mfg=%u  class=%u  fn=%u  name=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
      src, (unsigned)mfg, (unsigned)cls, (unsigned)fn,
      d[7], d[6], d[5], d[4], d[3], d[2], d[1], d[0]);
    break;
  }

  /* Everything else — fast-packet multi-frame or unhandled single frame */
  default: {
    const char *name = bridge_pgn_name(pgn);
    if (bridge_is_fast_packet(pgn)) {
      /* Fast-packet PGNs are logged once on completion via assembled events. */
      (void)name;
      return;
    } else {
      if (pgn == 126720u) {
        static uint32_t s_prop_count = 0u;
        static uint32_t s_prop_last_ms = 0u;
        uint32_t now = HAL_GetTick();
        s_prop_count++;
        if ((now - s_prop_last_ms) < 1000u) {
          return;
        }
        (void)snprintf(line, sizeof(line),
          "PROPRIETARY pgn=126720 src=%2u dst=%3u rate=%lu/s\r\n",
          src, dst, (unsigned long)s_prop_count);
        s_prop_count = 0u;
        s_prop_last_ms = now;
        break;
      }
      (void)snprintf(line, sizeof(line),
        "%-9s pgn=%-6lu  src=%2u  dst=%3u\r\n",
        name ? name : "UNKNOWN", (unsigned long)pgn, src, dst);
    }
    break;
  }

  } /* switch */

  if (line[0] != '\0') {
    bridge_uart_print(line);
  }
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
