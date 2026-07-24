/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Production RYLR896 LoRa RTCM receiver and ZED-F9P injector
  ******************************************************************************
  *
  * Data path:
  * RYLR896 -> USART2 -> STM32F103 -> USART1 binary RTCM -> ZED-F9P
  *
  * Reliability:
  * - Receives Base64 RTCM fragments.
  * - Reassembles complete RTCM frames.
  * - Validates RTCM CRC-24Q.
  * - Writes only valid binary RTCM to the ZED-F9P.
  * - ACKs the frame sequence only after successful UART injection.
  * - Re-ACKs duplicate retransmissions when the previous ACK was lost.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* ----------------------------- USER SETTINGS ----------------------------- */

#define LORA_TARGET_BAUD             115200U
#define GNSS_RTCM_BAUD               115200U

/* Must match the ESP32 base project. */
#define BASE_ADDRESS                 2U
#define ROVER_ADDRESS                1U
#define NETWORK_ID                   5U

/*
 * Highest-rate laboratory setting:
 * SF7, BW500 kHz, CR4/5, preamble 4.
 *
 * For more range / robustness use:
 * "AT+PARAMETER=7,7,1,4"  // BW125 kHz
 */
#define LORA_RF_PARAMETER            "AT+PARAMETER=7,9,1,4"
#define LORA_FREQUENCY               "AT+BAND=866000000"
#define LORA_RF_POWER                "AT+CRFOP=15"

/* Blue Pill onboard LED, normally active-low. */
#define STATUS_LED_PORT              GPIOC
#define STATUS_LED_PIN               GPIO_PIN_13

/* --------------------------- PROTOCOL CONSTANTS -------------------------- */

#define LORA_RESPONSE_TIMEOUT_MS     3000U
#define LORA_LINE_MAX                320U
#define LORA_LINE_QUEUE_DEPTH        8U
#define LORA_PAYLOAD_MAX             240U

#define RTCM_MAX_FRAME               1029U
#define RTCM_FRAGMENT_RAW_MAX        171U
#define RTCM_MAX_PARTS               8U
#define RTCM_FRAGMENT_HEADER_LENGTH  9U
#define RTCM_ASSEMBLY_TIMEOUT_MS     5000U
#define GNSS_TX_TIMEOUT_MS           300U



#define DEBUG_RTCM_HEX_ON_USART1     0U

/* Set 0 for one-way RTCM streaming without ACK back to the base. */
#define SEND_BASE_ACK                0U

/* ACK payload: A + four hexadecimal sequence digits. */
#define ACK_PAYLOAD_LENGTH           5U

/* ------------------------------- BUFFERS --------------------------------- */

static uint8_t loraRxByte;
static char loraBuildLine[LORA_LINE_MAX];
static volatile uint16_t loraBuildIndex = 0U;

static char loraLineQueue[LORA_LINE_QUEUE_DEPTH][LORA_LINE_MAX];
static volatile uint8_t loraQueueWrite = 0U;
static volatile uint8_t loraQueueRead = 0U;
static volatile uint8_t loraQueueCount = 0U;
static volatile uint32_t loraDroppedLines = 0U;

/* Global buffers avoid STM32F103 stack overflow. */
static char loraProcessLine[LORA_LINE_MAX];
static char loraPayload[LORA_PAYLOAD_MAX + 1U];
static uint8_t decodedFragment[RTCM_FRAGMENT_RAW_MAX];
static uint8_t assembledRtcmFrame[RTCM_MAX_FRAME];

static uint8_t fragmentData[RTCM_MAX_PARTS][RTCM_FRAGMENT_RAW_MAX];
static uint16_t fragmentLength[RTCM_MAX_PARTS];
static uint8_t fragmentReceived[RTCM_MAX_PARTS];

/* ---------------------------- ASSEMBLY STATE ----------------------------- */

static uint16_t activeSequence = 0U;
static uint8_t expectedFragments = 0U;
static uint8_t receivedFragments = 0U;
static bool sequenceActive = false;
static uint32_t lastFragmentTick = 0U;

static uint16_t lastCompletedSequence = 0U;
static bool lastCompletedSequenceValid = false;

/* Statistics are available in the debugger. */
static volatile uint32_t statValidRtcmFrames = 0U;
static volatile uint32_t statInvalidRtcmFrames = 0U;
static volatile uint32_t statInjectedRtcmFrames = 0U;
static volatile uint32_t statInjectionFailures = 0U;
static volatile uint32_t statAckSent = 0U;
static volatile uint32_t statAckSendFailures = 0U;
static volatile uint32_t statDuplicateFrames = 0U;
static volatile uint32_t statAssemblyTimeouts = 0U;
static volatile uint32_t statBase64Errors = 0U;
static volatile uint32_t statMalformedPackets = 0U;

/* ------------------------------- RTCM CRC -------------------------------- */

static uint32_t Crc24Q(const uint8_t *data, uint16_t length)
{
  uint32_t crc = 0U;

  for (uint16_t i = 0U; i < length; i++)
  {
    crc ^= (uint32_t)data[i] << 16U;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc <<= 1U;

      if ((crc & 0x1000000UL) != 0U)
      {
        crc ^= 0x1864CFBUL;
      }
    }
  }

  return crc & 0xFFFFFFUL;
}

static bool IsValidRtcmFrame(const uint8_t *frame, uint16_t length)
{
  if ((frame == NULL) || (length < 6U) || (frame[0] != 0xD3U))
  {
    return false;
  }

  if ((frame[1] & 0xFCU) != 0U)
  {
    return false;
  }

  const uint16_t payloadLength =
      (uint16_t)(((uint16_t)(frame[1] & 0x03U) << 8U) |
                 (uint16_t)frame[2]);

  if ((uint16_t)(payloadLength + 6U) != length)
  {
    return false;
  }

  const uint32_t calculated = Crc24Q(frame, (uint16_t)(length - 3U));
  const uint32_t received =
      ((uint32_t)frame[length - 3U] << 16U) |
      ((uint32_t)frame[length - 2U] << 8U) |
      (uint32_t)frame[length - 1U];

  return calculated == received;
}

/* ------------------------------ BASE64 ----------------------------------- */

static int Base64Value(char value)
{
  if ((value >= 'A') && (value <= 'Z')) return value - 'A';
  if ((value >= 'a') && (value <= 'z')) return value - 'a' + 26;
  if ((value >= '0') && (value <= '9')) return value - '0' + 52;
  if (value == '+') return 62;
  if (value == '/') return 63;
  if (value == '=') return -2;
  return -1;
}

static bool Base64Decode(const char *input,
                         uint16_t inputLength,
                         uint8_t *output,
                         uint16_t outputCapacity,
                         uint16_t *outputLength)
{
  if ((input == NULL) ||
      (output == NULL) ||
      (outputLength == NULL) ||
      (inputLength == 0U) ||
      ((inputLength & 3U) != 0U))
  {
    return false;
  }

  uint16_t in = 0U;
  uint16_t out = 0U;

  while (in < inputLength)
  {
    const int a = Base64Value(input[in++]);
    const int b = Base64Value(input[in++]);
    const int c = Base64Value(input[in++]);
    const int d = Base64Value(input[in++]);

    if ((a < 0) || (b < 0) || (c == -1) || (d == -1))
    {
      return false;
    }

    if (out >= outputCapacity)
    {
      return false;
    }

    output[out++] =
        (uint8_t)(((uint32_t)a << 2U) | ((uint32_t)b >> 4U));

    if (c == -2)
    {
      if (d != -2)
      {
        return false;
      }

      break;
    }

    if (out >= outputCapacity)
    {
      return false;
    }

    output[out++] =
        (uint8_t)(((uint32_t)b << 4U) | ((uint32_t)c >> 2U));

    if (d == -2)
    {
      break;
    }

    if (out >= outputCapacity)
    {
      return false;
    }

    output[out++] =
        (uint8_t)(((uint32_t)c << 6U) | (uint32_t)d);
  }

  *outputLength = out;
  return true;
}

/* ----------------------------- HEX PARSING ------------------------------- */

static int HexNibble(char value)
{
  if ((value >= '0') && (value <= '9')) return value - '0';
  if ((value >= 'A') && (value <= 'F')) return value - 'A' + 10;
  if ((value >= 'a') && (value <= 'f')) return value - 'a' + 10;
  return -1;
}

static bool ParseHexField(const char *text,
                          uint8_t digits,
                          uint16_t *value)
{
  if ((text == NULL) ||
      (value == NULL) ||
      (digits == 0U) ||
      (digits > 4U))
  {
    return false;
  }

  uint16_t result = 0U;

  for (uint8_t i = 0U; i < digits; i++)
  {
    const int nibble = HexNibble(text[i]);

    if (nibble < 0)
    {
      return false;
    }

    result = (uint16_t)((result << 4U) | (uint16_t)nibble);
  }

  *value = result;
  return true;
}

/* -------------------------- LORA CONFIGURATION --------------------------- */

static void LoRa_ClearPollingInput(void)
{
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
  {
    volatile uint32_t discard = huart2.Instance->DR;
    (void)discard;
  }
}

static bool LoRa_WaitForOK(uint32_t timeoutMs)
{
  char line[96];
  uint16_t index = 0U;
  const uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < timeoutMs)
  {
    uint8_t value = 0U;

    if (HAL_UART_Receive(&huart2, &value, 1U, 10U) != HAL_OK)
    {
      continue;
    }

    if (value == '\r')
    {
      continue;
    }

    if (value == '\n')
    {
      line[index] = '\0';

      if ((index > 0U) && (strcmp(line, "+OK") == 0))
      {
        return true;
      }

      if ((index >= 5U) && (strncmp(line, "+ERR=", 5U) == 0))
      {
        return false;
      }

      index = 0U;
    }
    else if (index < (sizeof(line) - 1U))
    {
      line[index++] = (char)value;
    }
    else
    {
      index = 0U;
    }
  }

  return false;
}

static bool LoRa_SendCommandWaitOK(const char *command, uint32_t timeoutMs)
{
  if (command == NULL)
  {
    return false;
  }

  LoRa_ClearPollingInput();

  if (HAL_UART_Transmit(&huart2,
                        (uint8_t *)command,
                        (uint16_t)strlen(command),
                        500U) != HAL_OK)
  {
    return false;
  }

  if (HAL_UART_Transmit(&huart2,
                        (uint8_t *)"\r\n",
                        2U,
                        100U) != HAL_OK)
  {
    return false;
  }

  return LoRa_WaitForOK(timeoutMs);
}

static bool LoRa_SetHostBaud(uint32_t baud)
{
  if (HAL_UART_DeInit(&huart2) != HAL_OK)
  {
    return false;
  }

  huart2.Init.BaudRate = baud;
  return HAL_UART_Init(&huart2) == HAL_OK;
}

static bool LoRa_ProbeBaud(uint32_t baud)
{
  if (!LoRa_SetHostBaud(baud))
  {
    return false;
  }

  HAL_Delay(80U);
  return LoRa_SendCommandWaitOK("AT", 600U);
}

static bool LoRa_ConfigureReceiver(void)
{
  static const uint32_t baudCandidates[] =
  {
    115200U,
    57600U,
    38400U,
    19200U,
    9600U
  };

  uint32_t detectedBaud = 0U;

  for (uint8_t i = 0U;
       i < (uint8_t)(sizeof(baudCandidates) / sizeof(baudCandidates[0]));
       i++)
  {
    if (LoRa_ProbeBaud(baudCandidates[i]))
    {
      detectedBaud = baudCandidates[i];
      break;
    }
  }

  if (detectedBaud == 0U)
  {
    return false;
  }

  if (detectedBaud != LORA_TARGET_BAUD)
  {
    if (!LoRa_SendCommandWaitOK("AT+IPR=115200",
                                LORA_RESPONSE_TIMEOUT_MS))
    {
      return false;
    }

    if (!LoRa_SetHostBaud(LORA_TARGET_BAUD))
    {
      return false;
    }

    HAL_Delay(120U);

    if (!LoRa_SendCommandWaitOK("AT", LORA_RESPONSE_TIMEOUT_MS))
    {
      return false;
    }
  }

  char command[48];

  (void)snprintf(command,
                 sizeof(command),
                 "AT+ADDRESS=%u",
                 (unsigned int)ROVER_ADDRESS);
  if (!LoRa_SendCommandWaitOK(command,
                              LORA_RESPONSE_TIMEOUT_MS)) return false;

  (void)snprintf(command,
                 sizeof(command),
                 "AT+NETWORKID=%u",
                 (unsigned int)NETWORK_ID);
  if (!LoRa_SendCommandWaitOK(command,
                              LORA_RESPONSE_TIMEOUT_MS)) return false;

  if (!LoRa_SendCommandWaitOK(LORA_FREQUENCY,
                              LORA_RESPONSE_TIMEOUT_MS)) return false;
  if (!LoRa_SendCommandWaitOK(LORA_RF_PARAMETER,
                              LORA_RESPONSE_TIMEOUT_MS)) return false;
  if (!LoRa_SendCommandWaitOK(LORA_RF_POWER,
                              LORA_RESPONSE_TIMEOUT_MS)) return false;

  return true;
}

/* ------------------------------- ACK ------------------------------------- */

static bool LoRa_SendFrameAck(uint16_t sequence)
{
  char command[40];

  const int length = snprintf(command,
                              sizeof(command),
                              "AT+SEND=%u,%u,A%04X\r\n",
                              (unsigned int)BASE_ADDRESS,
                              (unsigned int)ACK_PAYLOAD_LENGTH,
                              (unsigned int)sequence);

  if ((length <= 0) || ((size_t)length >= sizeof(command)))
  {
    statAckSendFailures++;
    return false;
  }

  /*
   * RX interrupt remains active. Do not call blocking HAL_UART_Receive here.
   * The module's +OK response is received as a normal line and ignored by the
   * main parser. If this ACK is lost over the air, the base retransmits the
   * same frame and this rover re-ACKs the duplicate sequence.
   */
  if (HAL_UART_Transmit(&huart2,
                        (uint8_t *)command,
                        (uint16_t)length,
                        500U) != HAL_OK)
  {
    statAckSendFailures++;
    return false;
  }

  statAckSent++;
  return true;
}

static bool TransmitRtcmToGnssOrDebug(const uint8_t *frame, uint16_t length)
{
  if ((frame == NULL) || (length == 0U))
  {
    return false;
  }

#if DEBUG_RTCM_HEX_ON_USART1
  static const char hexDigits[] = "0123456789ABCDEF";
  char text[4];

  const char prefix[] = "RTCM HEX: ";
  if (HAL_UART_Transmit(&huart1,
                        (uint8_t *)prefix,
                        (uint16_t)(sizeof(prefix) - 1U),
                        GNSS_TX_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  for (uint16_t i = 0U; i < length; i++)
  {
    text[0] = hexDigits[(frame[i] >> 4U) & 0x0FU];
    text[1] = hexDigits[frame[i] & 0x0FU];
    text[2] = ' ';

    if (HAL_UART_Transmit(&huart1,
                          (uint8_t *)text,
                          3U,
                          GNSS_TX_TIMEOUT_MS) != HAL_OK)
    {
      return false;
    }
  }

  text[0] = '\r';
  text[1] = '\n';
  return HAL_UART_Transmit(&huart1,
                           (uint8_t *)text,
                           2U,
                           GNSS_TX_TIMEOUT_MS) == HAL_OK;
#else
  return HAL_UART_Transmit(&huart1,
                           (uint8_t *)frame,
                           length,
                           GNSS_TX_TIMEOUT_MS) == HAL_OK;
#endif
}

/* --------------------------- RTCM REASSEMBLY ----------------------------- */

static void ResetRtcmAssembly(void)
{
  activeSequence = 0U;
  expectedFragments = 0U;
  receivedFragments = 0U;
  sequenceActive = false;
  lastFragmentTick = 0U;

  memset(fragmentLength, 0, sizeof(fragmentLength));
  memset(fragmentReceived, 0, sizeof(fragmentReceived));
}

static void StartRtcmAssembly(uint16_t sequence, uint8_t totalFragments)
{
  ResetRtcmAssembly();
  activeSequence = sequence;
  expectedFragments = totalFragments;
  sequenceActive = true;
  lastFragmentTick = HAL_GetTick();
}

static void CompleteRtcmFrameIfReady(void)
{
  if ((!sequenceActive) ||
      (receivedFragments != expectedFragments))
  {
    return;
  }

  uint16_t frameLength = 0U;

  for (uint8_t i = 0U; i < expectedFragments; i++)
  {
    if (fragmentReceived[i] == 0U)
    {
      return;
    }

    if ((uint32_t)frameLength + fragmentLength[i] > RTCM_MAX_FRAME)
    {
      statInvalidRtcmFrames++;
      ResetRtcmAssembly();
      return;
    }

    memcpy(&assembledRtcmFrame[frameLength],
           fragmentData[i],
           fragmentLength[i]);

    frameLength = (uint16_t)(frameLength + fragmentLength[i]);
  }

  if (!IsValidRtcmFrame(assembledRtcmFrame, frameLength))
  {
    statInvalidRtcmFrames++;
    ResetRtcmAssembly();
    return;
  }

  statValidRtcmFrames++;

  if (TransmitRtcmToGnssOrDebug(assembledRtcmFrame, frameLength))
  {
    statInjectedRtcmFrames++;
    lastCompletedSequence = activeSequence;
    lastCompletedSequenceValid = true;

    HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);

    if (SEND_BASE_ACK != 0U)
    {
      (void)LoRa_SendFrameAck(activeSequence);
    }
  }
  else
  {
    statInjectionFailures++;
  }

  ResetRtcmAssembly();
}

static void ProcessRtcmFragmentPayload(char *payload)
{
  if (payload == NULL)
  {
    statMalformedPackets++;
    return;
  }

  const uint16_t payloadLength = (uint16_t)strlen(payload);

  if ((payloadLength < RTCM_FRAGMENT_HEADER_LENGTH) ||
      (payload[0] != 'R'))
  {
    statMalformedPackets++;
    return;
  }

  uint16_t sequence = 0U;
  uint16_t fragmentIndexField = 0U;
  uint16_t fragmentCountField = 0U;

  if (!ParseHexField(&payload[1], 4U, &sequence) ||
      !ParseHexField(&payload[5], 2U, &fragmentIndexField) ||
      !ParseHexField(&payload[7], 2U, &fragmentCountField))
  {
    statMalformedPackets++;
    return;
  }

  const uint8_t fragmentIndex = (uint8_t)fragmentIndexField;
  const uint8_t fragmentCount = (uint8_t)fragmentCountField;

  if ((fragmentCount == 0U) ||
      (fragmentCount > RTCM_MAX_PARTS) ||
      (fragmentIndex >= fragmentCount))
  {
    statMalformedPackets++;
    return;
  }

  /*
   * Ignore duplicate completed frames and do not inject the same RTCM twice.
   */
  if (lastCompletedSequenceValid &&
      (sequence == lastCompletedSequence))
  {
    statDuplicateFrames++;
    if (SEND_BASE_ACK != 0U)
    {
      (void)LoRa_SendFrameAck(sequence);
    }
    return;
  }

  const char *encodedData = &payload[RTCM_FRAGMENT_HEADER_LENGTH];
  const uint16_t encodedLength =
      (uint16_t)(payloadLength - RTCM_FRAGMENT_HEADER_LENGTH);

  uint16_t decodedLength = 0U;

  if (!Base64Decode(encodedData,
                    encodedLength,
                    decodedFragment,
                    sizeof(decodedFragment),
                    &decodedLength))
  {
    statBase64Errors++;
    return;
  }

  if ((decodedLength == 0U) ||
      (decodedLength > RTCM_FRAGMENT_RAW_MAX))
  {
    statMalformedPackets++;
    return;
  }

  if ((!sequenceActive) || (sequence != activeSequence))
  {
    StartRtcmAssembly(sequence, fragmentCount);
  }
  else if (fragmentCount != expectedFragments)
  {
    StartRtcmAssembly(sequence, fragmentCount);
  }

  memcpy(fragmentData[fragmentIndex],
         decodedFragment,
         decodedLength);

  fragmentLength[fragmentIndex] = decodedLength;

  if (fragmentReceived[fragmentIndex] == 0U)
  {
    fragmentReceived[fragmentIndex] = 1U;
    receivedFragments++;
  }

  lastFragmentTick = HAL_GetTick();
  CompleteRtcmFrameIfReady();
}

/* ---------------------------- LORA PARSING ------------------------------- */

static void ProcessLoRaLine(char *line)
{
  if (line == NULL)
  {
    return;
  }

  /* +OK from an ACK command is expected and can be ignored. */
  if ((strcmp(line, "+OK") == 0) ||
      (strncmp(line, "+ERR=", 5U) == 0))
  {
    return;
  }

  if (strncmp(line, "+RCV=", 5U) != 0)
  {
    return;
  }

  char *addressEnd = strchr(&line[5], ',');
  if (addressEnd == NULL)
  {
    statMalformedPackets++;
    return;
  }

  *addressEnd = '\0';
  const uint16_t sourceAddress =
      (uint16_t)strtoul(&line[5], NULL, 10);

  if (sourceAddress != BASE_ADDRESS)
  {
    return;
  }

  char *lengthEnd = strchr(addressEnd + 1, ',');
  if (lengthEnd == NULL)
  {
    statMalformedPackets++;
    return;
  }

  *lengthEnd = '\0';
  const uint16_t declaredLength =
      (uint16_t)strtoul(addressEnd + 1, NULL, 10);

  if ((declaredLength == 0U) ||
      (declaredLength > LORA_PAYLOAD_MAX))
  {
    statMalformedPackets++;
    return;
  }

  char *payloadStart = lengthEnd + 1;

  /* The RYLR896 appends ,RSSI,SNR after exactly declaredLength bytes. */
  if (strlen(payloadStart) < declaredLength)
  {
    statMalformedPackets++;
    return;
  }

  memcpy(loraPayload, payloadStart, declaredLength);
  loraPayload[declaredLength] = '\0';

  ProcessRtcmFragmentPayload(loraPayload);
}

static bool PopLoRaLine(char *destination)
{
  if (destination == NULL)
  {
    return false;
  }

  bool available = false;

  __disable_irq();

  if (loraQueueCount > 0U)
  {
    strncpy(destination,
            loraLineQueue[loraQueueRead],
            LORA_LINE_MAX);

    destination[LORA_LINE_MAX - 1U] = '\0';

    loraQueueRead =
        (uint8_t)((loraQueueRead + 1U) % LORA_LINE_QUEUE_DEPTH);

    loraQueueCount--;
    available = true;
  }

  __enable_irq();
  return available;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (loraRxByte == '\n')
    {
      if (loraBuildIndex > 0U)
      {
        loraBuildLine[loraBuildIndex] = '\0';

        if (loraQueueCount < LORA_LINE_QUEUE_DEPTH)
        {
          memcpy(loraLineQueue[loraQueueWrite],
                 loraBuildLine,
                 (size_t)loraBuildIndex + 1U);

          loraQueueWrite =
              (uint8_t)((loraQueueWrite + 1U) % LORA_LINE_QUEUE_DEPTH);

          loraQueueCount++;
        }
        else
        {
          loraDroppedLines++;
        }
      }

      loraBuildIndex = 0U;
    }
    else if (loraRxByte != '\r')
    {
      if (loraBuildIndex < (LORA_LINE_MAX - 1U))
      {
        loraBuildLine[loraBuildIndex++] = (char)loraRxByte;
      }
      else
      {
        loraBuildIndex = 0U;
      }
    }

    (void)HAL_UART_Receive_IT(&huart2, &loraRxByte, 1U);
  }
}

/* -------------------------------- MAIN ----------------------------------- */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  ResetRtcmAssembly();

  /* Startup indication. */
  for (uint8_t i = 0U; i < 3U; i++)
  {
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
    HAL_Delay(80U);
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET);
    HAL_Delay(80U);
  }

  if (!LoRa_ConfigureReceiver())
  {
    Error_Handler();
  }

  loraBuildIndex = 0U;
  loraQueueWrite = 0U;
  loraQueueRead = 0U;
  loraQueueCount = 0U;

  if (HAL_UART_Receive_IT(&huart2, &loraRxByte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  while (1)
  {
    while (PopLoRaLine(loraProcessLine))
    {
      ProcessLoRaLine(loraProcessLine);
    }

    if (sequenceActive &&
        ((HAL_GetTick() - lastFragmentTick) > RTCM_ASSEMBLY_TIMEOUT_MS))
    {
      statAssemblyTimeouts++;
      ResetRtcmAssembly();
    }
  }
}

/* ---------------------------- CUBE INITIALIZATION ------------------------ */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                               RCC_CLOCKTYPE_SYSCLK |
                               RCC_CLOCKTYPE_PCLK1 |
                               RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                          FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = GNSS_RTCM_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = LORA_TARGET_BAUD;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(STATUS_LED_PORT,
                    STATUS_LED_PIN,
                    GPIO_PIN_SET);

  GPIO_InitStruct.Pin = STATUS_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_PORT, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
    /* Rapid blinking indicates startup/configuration failure. */
    HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);

    for (volatile uint32_t delayCount = 0U;
         delayCount < 300000U;
         delayCount++)
    {
      __NOP();
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
