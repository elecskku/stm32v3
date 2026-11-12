/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <limits.h>

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18

#define UART_DATA_SIZE       4096U
#define FLASH_PAGE_SIZE      256U
#define FLASH_SECTOR_SIZE    4096U
#define FLASH_TOTAL_SIZE     (4U * 1024U * 1024U)

#define FLASH_CMD_JEDEC_ID       0x9F
#define FLASH_CMD_WRITE_ENABLE   0x06
#define FLASH_CMD_READ_STATUS1   0x05
#define FLASH_CMD_PAGE_PROGRAM   0x02
#define FLASH_CMD_READ_DATA      0x03
#define FLASH_CMD_SECTOR_ERASE   0x20
#define FLASH_CMD_CHIP_ERASE     0xC7

uint8_t uart_rx_buffer[UART_DATA_SIZE];

/* Timing reported by the PC T command (milliseconds, accumulated per demo). */
uint32_t timing_uart_rx_ms = 0;
uint32_t timing_flash_write_ms = 0;
uint32_t timing_flash_read_ms = 0;
uint32_t timing_gray_ms = 0;
uint32_t timing_uart_tx_ms = 0;

/* Remember last successfully stored packet for ACK-loss retries. */
uint8_t last_packet_valid = 0;
uint32_t last_packet_seq = 0;
uint16_t last_packet_len = 0;
uint32_t last_packet_crc = 0;


/* File information kept in MCU SRAM.
 * These values are lost if the STM32 is reset or powered off. */
uint32_t stored_file_size = 0;
uint32_t stored_file_crc  = 0;

volatile uint32_t flash_erase_ms = 0;
volatile uint32_t flash_write_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Flash_CS_Low(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void Flash_CS_High(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static void Flash_Read_ID(uint8_t *id)
{
    uint8_t cmd = FLASH_CMD_JEDEC_ID;

    Flash_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, id, 3, HAL_MAX_DELAY);
    Flash_CS_High();
}

static void Flash_WriteEnable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;

    Flash_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    Flash_CS_High();
}

static uint8_t Flash_ReadStatus(void)
{
    uint8_t cmd = FLASH_CMD_READ_STATUS1;
    uint8_t status = 0;

    Flash_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    Flash_CS_High();

    return status;
}

static HAL_StatusTypeDef Flash_WaitBusy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (Flash_ReadStatus() & 0x01U)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Flash_SectorErase(uint32_t address)
{
    uint8_t cmd[4];

    if (address >= FLASH_TOTAL_SIZE)
        return HAL_ERROR;

    Flash_WriteEnable();

    cmd[0] = FLASH_CMD_SECTOR_ERASE;
    cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(address & 0xFFU);

    Flash_CS_Low();
    if (HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }
    Flash_CS_High();

    return Flash_WaitBusy(5000);
}

static HAL_StatusTypeDef Flash_ChipErase(void)
{
    uint8_t cmd = FLASH_CMD_CHIP_ERASE;

    Flash_WriteEnable();

    Flash_CS_Low();
    if (HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }
    Flash_CS_High();

    /* Chip erase may take several seconds. */
    return Flash_WaitBusy(60000);
}

static HAL_StatusTypeDef Flash_PageProgram(uint32_t address,
                                           const uint8_t *data,
                                           uint16_t length)
{
    uint8_t cmd[4];

    if ((length == 0U) ||
        (length > FLASH_PAGE_SIZE) ||
        (address + length > FLASH_TOTAL_SIZE))
    {
        return HAL_ERROR;
    }

    Flash_WriteEnable();

    cmd[0] = FLASH_CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(address & 0xFFU);

    Flash_CS_Low();

    if (HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    Flash_CS_High();

    return Flash_WaitBusy(1000);
}

volatile uint32_t flash_debug_page = 0;

static HAL_StatusTypeDef Flash_Write(uint32_t address,
                                     const uint8_t *data,
                                     uint32_t length)
{
    uint32_t offset = 0;

    flash_debug_page = 0;

    if (address + length > FLASH_TOTAL_SIZE)
        return HAL_ERROR;

    while (offset < length)
    {
        uint32_t current_address = address + offset;
        uint32_t page_offset = current_address % FLASH_PAGE_SIZE;
        uint32_t writable = FLASH_PAGE_SIZE - page_offset;
        uint32_t remaining = length - offset;

        uint16_t chunk =
            (uint16_t)((remaining < writable) ? remaining : writable);

        flash_debug_page++;

        if (Flash_PageProgram(current_address,
                              data + offset,
                              chunk) != HAL_OK)
        {
            return HAL_ERROR;
        }

        offset += chunk;
    }

    return HAL_OK;
}


static HAL_StatusTypeDef Flash_Read(uint32_t address,
                                    uint8_t *data,
                                    uint32_t length)
{
    uint8_t cmd[4];

    if (address + length > FLASH_TOTAL_SIZE)
        return HAL_ERROR;

    cmd[0] = FLASH_CMD_READ_DATA;
    cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(address & 0xFFU);

    Flash_CS_Low();

    if (HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    if (HAL_SPI_Receive(&hspi1, data, (uint16_t)length, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    Flash_CS_High();
    return HAL_OK;
}

static uint32_t CRC32_Update(uint32_t crc,
                             const uint8_t *data,
                             uint32_t length)
{
    static uint32_t table[256];
    static uint8_t initialized = 0;

    // Lookup Table 최초 1회 생성
    if (!initialized)
    {
        for (uint32_t i = 0; i < 256; i++)
        {
            uint32_t value = i;

            for (uint8_t j = 0; j < 8; j++)
            {
                if (value & 1U)
                    value = (value >> 1) ^ 0xEDB88320U;
                else
                    value >>= 1;
            }

            table[i] = value;
        }

        initialized = 1;
    }

    // Table 기반 CRC32 계산
    for (uint32_t i = 0; i < length; i++)
    {
        uint8_t index = (uint8_t)(crc ^ data[i]);

        crc = (crc >> 8) ^ table[index];
    }

    return crc;
}

static uint16_t ReadU16LE(const uint8_t *data)
{
    return ((uint16_t)data[0]) |
           ((uint16_t)data[1] << 8);
}

static uint32_t ReadU32LE(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

/* 24-bit uncompressed BMP layout, required for grayscale readback. */
typedef struct {
    uint32_t pixel_offset;
    uint32_t width;
    uint32_t row_stride;
    uint32_t height;
} BMP_Info;

static uint8_t BMP_ReadInfo(BMP_Info *bmp)
{
    uint8_t h[54];
    uint32_t dib;
    int32_t width, height;
    uint64_t stride, end;
    uint16_t planes, bits;

    if (stored_file_size < sizeof(h) ||
        Flash_Read(0, h, sizeof(h)) != HAL_OK)
        return 0;

    dib = ReadU32LE(h + 14);
    width = (int32_t)ReadU32LE(h + 18);
    height = (int32_t)ReadU32LE(h + 22);
    planes = ReadU16LE(h + 26);
    bits = ReadU16LE(h + 28);
    if (h[0] != 'B' || h[1] != 'M' || dib != 40U ||
        width <= 0 || height == 0 || height == INT32_MIN ||
        planes != 1U || bits != 24U || ReadU32LE(h + 30) != 0U)
        return 0;

    bmp->pixel_offset = ReadU32LE(h + 10);
    bmp->width = (uint32_t)width;
    bmp->height = (height < 0) ? (uint32_t)(-height) : (uint32_t)height;
    stride = (((uint64_t)bmp->width * 3U + 3U) / 4U) * 4U;
    end = (uint64_t)bmp->pixel_offset + stride * bmp->height;
    if (bmp->pixel_offset < 54U || stride > UINT32_MAX ||
        end > stored_file_size)
        return 0;
    bmp->row_stride = (uint32_t)stride;
    return 1;
}

/* Preserve BMP header, padding and 24-bit layout; replace B,G,R with Gray. */
static HAL_StatusTypeDef BMP_GrayChunk(uint32_t start, uint32_t length,
                                       const BMP_Info *bmp)
{
    uint64_t chunk_end = (uint64_t)start + length;
    uint64_t pixel_base = bmp->pixel_offset;
    uint64_t pixel_end = pixel_base + (uint64_t)bmp->row_stride * bmp->height;
    uint64_t begin, end;
    uint32_t first_row, last_row;

    if (chunk_end <= pixel_base || start >= pixel_end)
        return HAL_OK;
    begin = (start > pixel_base) ? start : pixel_base;
    end = (chunk_end < pixel_end) ? chunk_end : pixel_end;
    first_row = (uint32_t)((begin - pixel_base) / bmp->row_stride);
    last_row = (uint32_t)((end - 1U - pixel_base) / bmp->row_stride);

    for (uint32_t row = first_row; row <= last_row; ++row)
    {
        uint64_t row_start = pixel_base + (uint64_t)row * bmp->row_stride;
        uint64_t row_end = row_start + (uint64_t)bmp->width * 3U;
        uint64_t active_begin = (begin > row_start) ? begin : row_start;
        uint64_t active_end = (end < row_end) ? end : row_end;
        if (active_begin >= active_end)
            continue;

        uint32_t first_pixel = (uint32_t)((active_begin - row_start) / 3U);
        uint32_t last_pixel = (uint32_t)((active_end - 1U - row_start) / 3U);
        for (uint32_t x = first_pixel; x <= last_pixel; ++x)
        {
            uint32_t px = (uint32_t)(row_start + (uint64_t)x * 3U);
            uint8_t bgr[3];
            if (px >= start && (uint64_t)px + 3U <= chunk_end)
            {
                uint32_t i = px - start;
                bgr[0] = uart_rx_buffer[i];
                bgr[1] = uart_rx_buffer[i + 1U];
                bgr[2] = uart_rx_buffer[i + 2U];
            }
            else if (Flash_Read(px, bgr, 3U) != HAL_OK)
                return HAL_ERROR;

            /* Integer approximation: Gray = 0.114 B + 0.587 G + 0.299 R */
            uint8_t gray = (uint8_t)((114U*bgr[0] + 587U*bgr[1] +
                                      299U*bgr[2] + 500U) / 1000U);
            for (uint32_t c = 0; c < 3U; ++c)
                if ((uint64_t)px + c >= start && (uint64_t)px + c < chunk_end)
                    uart_rx_buffer[px + c - start] = gray;
        }
    }
    return HAL_OK;
}

static void UART_SendResponse(uint8_t response)
{
    (void)HAL_UART_Transmit(&huart2, &response, 1, 1000);
}

/* A previously accepted packet may be resent if its ACK was lost. */
static void HandleFinalPacketRetry(uint8_t first_byte)
{
    uint8_t header[6], crc_bytes[4];
    uint32_t crc, seq;
    uint16_t len;
    header[0] = first_byte;
    if (HAL_UART_Receive(&huart2, header + 1, 5, 5000) != HAL_OK)
        return;
    seq = ReadU32LE(header);
    len = ReadU16LE(header + 4);
    if (len == 0U || len > UART_DATA_SIZE)
        return;
    if (HAL_UART_Receive(&huart2, uart_rx_buffer, len, 5000) != HAL_OK ||
        HAL_UART_Receive(&huart2, crc_bytes, 4, 5000) != HAL_OK)
        return;
    crc = CRC32_Update(0xFFFFFFFFU, header, 6);
    crc = CRC32_Update(crc, uart_rx_buffer, len) ^ 0xFFFFFFFFU;
    if (last_packet_valid && seq == last_packet_seq &&
        len == last_packet_len && crc == last_packet_crc &&
        ReadU32LE(crc_bytes) == crc)
        UART_SendResponse(ACK);
    else
        UART_SendResponse(NAK);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* Keep Flash deselected while idle. */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint8_t cmd;

    if (HAL_UART_Receive(&huart2, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        continue;
    }

    switch (cmd)
    {
        /* I: ACK + JEDEC ID(3B) + Status(1B) */
        case 'I':
        {
            uint8_t id[3];
            uint8_t status;
            uint8_t response = ACK;

            Flash_Read_ID(id);
            status = Flash_ReadStatus();

            HAL_UART_Transmit(&huart2, &response, 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, id, 3, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, &status, 1, HAL_MAX_DELAY);
            break;
        }

        /* E: Chip erase, then ACK. */
        case 'E':
        {
            uint8_t response;

            if (Flash_ChipErase() == HAL_OK)
            {
                stored_file_size = 0;
                stored_file_crc = 0;
                last_packet_valid = 0;
                response = ACK;
            }
            else
            {
                response = CAN;
            }

            HAL_UART_Transmit(&huart2, &response, 1, HAL_MAX_DELAY);
            break;
        }

        /* W: 8-byte metadata, start ACK, then Stop-and-Wait packets. */
        case 'W':
        {
            uint8_t metadata[8];
            uint32_t expected_size, expected_crc;
            uint32_t expected_sequence = 0, total_size = 0;
            uint32_t running_crc = 0xFFFFFFFFU;
            uint8_t write_failed = 0;

            last_packet_valid = 0;
            stored_file_size = 0;
            stored_file_crc = 0;
            timing_uart_rx_ms = timing_flash_write_ms = 0;
            timing_flash_read_ms = timing_gray_ms = timing_uart_tx_ms = 0;

            if (HAL_UART_Receive(&huart2, metadata, 8, 5000) != HAL_OK)
            {
                UART_SendResponse(CAN);
                break;
            }
            expected_size = ReadU32LE(metadata);
            expected_crc = ReadU32LE(metadata + 4);
            if (expected_size == 0U || expected_size > FLASH_TOTAL_SIZE)
            {
                UART_SendResponse(CAN);
                break;
            }
            UART_SendResponse(ACK); /* PC may now send packet #0. */

            while (total_size < expected_size)
            {
                uint8_t header[6], crc_bytes[4];
                uint32_t sequence, received_crc, packet_crc, flash_address;
                uint16_t length;
                uint32_t t;

                /* A header may arrive after the flash operation/ACK. */
                if (HAL_UART_Receive(&huart2, header, 6, 5000) != HAL_OK)
                {
                    write_failed = 1;
                    break;
                }
                sequence = ReadU32LE(header);
                length = ReadU16LE(header + 4);
                if (length == 0U || length > UART_DATA_SIZE)
                {
                    UART_SendResponse(CAN);
                    write_failed = 1;
                    break;
                }

                t = HAL_GetTick();
                if (HAL_UART_Receive(&huart2, uart_rx_buffer, length, 5000) != HAL_OK ||
                    HAL_UART_Receive(&huart2, crc_bytes, 4, 5000) != HAL_OK)
                {
                    UART_SendResponse(CAN); /* Partial frame: cannot safely resync. */
                    write_failed = 1;
                    break;
                }
                timing_uart_rx_ms += HAL_GetTick() - t;

                received_crc = ReadU32LE(crc_bytes);
                packet_crc = CRC32_Update(0xFFFFFFFFU, header, 6);
                packet_crc = CRC32_Update(packet_crc, uart_rx_buffer, length);
                packet_crc ^= 0xFFFFFFFFU;

                if (packet_crc != received_crc)
                {
                    UART_SendResponse(NAK);
                    continue;
                }
                if (last_packet_valid && sequence == last_packet_seq)
                {
                    /* Resend ACK only for the exact previous packet. */
                    if (length == last_packet_len && received_crc == last_packet_crc)
                        UART_SendResponse(ACK);
                    else
                        UART_SendResponse(NAK);
                    continue;
                }
                if (sequence != expected_sequence ||
                    length != ((expected_size - total_size > UART_DATA_SIZE)
                        ? UART_DATA_SIZE : (expected_size - total_size)))
                {
                    UART_SendResponse(NAK);
                    continue;
                }
                flash_address = total_size;

                t = HAL_GetTick();
                /* Erasing each sector makes standalone W work without an E command. */
                if (Flash_Write(flash_address, uart_rx_buffer, length) != HAL_OK)
                {
                    UART_SendResponse(CAN);
                    write_failed = 1;
                    break;
                }
                timing_flash_write_ms += HAL_GetTick() - t;
                running_crc = CRC32_Update(running_crc, uart_rx_buffer, length);
                total_size += length;
                expected_sequence++;
                last_packet_valid = 1;
                last_packet_seq = sequence;
                last_packet_len = length;
                last_packet_crc = received_crc;

                if (total_size == expected_size)
                {
                    /* Full-file transmitted CRC check; V later checks Flash contents. */
                    if ((running_crc ^ 0xFFFFFFFFU) != expected_crc)
                    {
                        UART_SendResponse(CAN);
                        write_failed = 1;
                        break;
                    }
                    stored_file_size = expected_size;
                    stored_file_crc = expected_crc;
                }
                UART_SendResponse(ACK); /* Flash program and checks have finished. */
            }

            if (write_failed)
            {
                stored_file_size = 0;
                stored_file_crc = 0;
                last_packet_valid = 0;
            }
            break;
        }

        /* R: send 24-bit grayscale BMP without modifying Flash (original retained). */
        case 'R':
        {
            BMP_Info bmp;
            uint8_t size_bytes[4];
            uint32_t offset = 0, read_crc = 0xFFFFFFFFU;

            if (stored_file_size == 0U || !BMP_ReadInfo(&bmp))
            {
                UART_SendResponse(NAK);
                break;
            }

            UART_SendResponse(ACK);
            WriteU32LE(size_bytes, stored_file_size);
            if (HAL_UART_Transmit(&huart2, size_bytes, 4, 1000) != HAL_OK)
                break;

            while (offset < stored_file_size)
            {
                uint32_t length = stored_file_size - offset;
                uint32_t t;
                if (length > UART_DATA_SIZE) length = UART_DATA_SIZE;

                t = HAL_GetTick();
                if (Flash_Read(offset, uart_rx_buffer, length) != HAL_OK)
                    break;
                timing_flash_read_ms += HAL_GetTick() - t;

                t = HAL_GetTick();
                if (BMP_GrayChunk(offset, length, &bmp) != HAL_OK)
                    break;
                timing_gray_ms += HAL_GetTick() - t;

                read_crc = CRC32_Update(read_crc, uart_rx_buffer, length);
                t = HAL_GetTick();
                if (HAL_UART_Transmit(&huart2, uart_rx_buffer,
                                      (uint16_t)length, 10000) != HAL_OK)
                    break;
                timing_uart_tx_ms += HAL_GetTick() - t;
                offset += length;
            }
            if (offset == stored_file_size)
            {
                uint8_t crc_bytes[4];
                WriteU32LE(crc_bytes, read_crc ^ 0xFFFFFFFFU);
                (void)HAL_UART_Transmit(&huart2, crc_bytes, 4, 1000);
            }
            break;
        }

        /* V: stored CRC(4B) + CRC recalculated from Flash(4B) */
        case 'V':
        {
            uint32_t calculated_crc = 0xFFFFFFFFU;
            uint32_t offset = 0;
            uint8_t result[8];

            if (stored_file_size == 0U)
            {
                WriteU32LE(&result[0], 0U);
                WriteU32LE(&result[4], 0xFFFFFFFFU);
                HAL_UART_Transmit(&huart2, result, 8, HAL_MAX_DELAY);
                break;
            }

            while (offset < stored_file_size)
            {
                uint32_t remaining = stored_file_size - offset;
                uint32_t chunk = (remaining > UART_DATA_SIZE)
                                 ? UART_DATA_SIZE
                                 : remaining;

                if (Flash_Read(offset, uart_rx_buffer, chunk) != HAL_OK)
                {
                    calculated_crc = 0;
                    break;
                }

                calculated_crc = CRC32_Update(calculated_crc,
                                              uart_rx_buffer,
                                              chunk);
                offset += chunk;
            }

            if (offset == stored_file_size)
                calculated_crc ^= 0xFFFFFFFFU;

            WriteU32LE(&result[0], stored_file_crc);
            WriteU32LE(&result[4], calculated_crc);

            HAL_UART_Transmit(&huart2, result, 8, HAL_MAX_DELAY);
            break;
        }

        /* T: ACK + 5 little-endian uint32_t durations in milliseconds. */
        case 'T':
        {
            uint8_t data[20];
            WriteU32LE(data + 0, timing_uart_rx_ms);
            WriteU32LE(data + 4, timing_flash_write_ms);
            WriteU32LE(data + 8, timing_flash_read_ms);
            WriteU32LE(data + 12, timing_gray_ms);
            WriteU32LE(data + 16, timing_uart_tx_ms);
            UART_SendResponse(ACK);
            (void)HAL_UART_Transmit(&huart2, data, sizeof(data), 1000);
            break;
        }

        default:
        {
            /* Packet retransmission after final ACK loss has no command framing.
             * Detect only if the first byte matches the final sequence LSB.
             * For a production protocol add an explicit packet/command delimiter. */
            if (last_packet_valid && stored_file_size != 0U &&
                cmd == (uint8_t)(last_packet_seq & 0xFFU))
                HandleFinalPacketRetry(cmd);
            else
                UART_SendResponse(NAK);
            break;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
