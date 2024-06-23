/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @example examples/ble_peripheral/ble_app_buttonless_dfu
 *
 * @brief Secure DFU Buttonless Service Application main file.
 *
 * This file contains the source code for a sample application using the proprietary
 * Secure DFU Buttonless Service. This is a template application that can be modified
 * to your needs. To extend the functionality of this application, please find
 * locations where the comment "// YOUR_JOB:" is present and read the comments.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "ble_hci.h"
#include "custom_board.h"

#include "ble_nus.h"
#include "app_uart.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"

// From 14.2.0 app

#include "nrf_serial.h"
#include "nrf_queue.h"
#include "nrf_drv_pwm.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nordic_common.h"
#include "nrf_delay.h"

// audio driver
#include "nrf_drv_i2s.h"

// FatFs
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "app_fifo.h"

#include "SEGGER_RTT.h"



#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

/************************************************************
*   Include support for SGTL5000. 
************************************************************/
#include "drv_sgtl5000.h"
#define AUDIO_FRAME_WORDS                   320
#define I2S_BUFFER_SIZE_WORDS               AUDIO_FRAME_WORDS * 2   // Double buffered - I2S lib will switch between using first and second half
static uint32_t  m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS];
static uint32_t  m_i2s_rx_buffer[I2S_BUFFER_SIZE_WORDS];
    
/* Include car sample to demonstrate that sample can be played from application as well */
#define SAMPLE_LEN                  960000

//extern const uint8_t                car_sample[SAMPLE_LEN];
//static uint8_t * p_sample           = (uint8_t *)car_sample;
static uint32_t sample_idx          = 0;

#define FILE_NAME   "Snooze.raw"
#define TEST_STRING "SD card example."
FRESULT rc;

#define SDC_SCK_PIN     17  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    13  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    24  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      7  ///< SDC chip select (CS) pin.

#define buffer_size 16384
uint8_t buffer[buffer_size];
app_fifo_t audio_stream;
uint8_t err_cnt;



static FATFS fs;
static DIR dir;
static FILINFO fno;
static FIL file;
uint8_t readbytes[128];
uint8_t data[33920];

uint8_t rx_bytes[3] = {};
uint8_t rx_byte[1] = {};
//static UINT rx_bytes;
FSIZE_t SAMPLE_LENGTH;

uint32_t bytes_written;
FRESULT ff_result;
DSTATUS disk_state = STA_NOINIT;

static enum
{
    SGTL5000_STATE_UNINITIALIZED,   /* Not initialized */
    SGTL5000_STATE_CONFIGURATION,   /* Providing MCLK and configuring via TWI, but not streaming */
    SGTL5000_STATE_IDLE,            /* Initialized, but not running */
    SGTL5000_STATE_RUNNING,         /* Actively streaming audio to/from application */
    SGTL5000_STATE_RUNNING_LOOPBACK,/* Actively running audio loopback (microphone -> speaker) */
    SGTL5000_STATE_RUNNING_SAMPLE,  /* Actively streaming sample */
} m_state = SGTL5000_STATE_UNINITIALIZED;

/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

/**
 * @brief Function for demonstrating FAFTS usage.
 */
static void fatfs_init()
{
    

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
        return;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return;
    }

    return;
}

UINT audio_stream_handler(const BYTE *p,  UINT btf ){
    UINT cnt = 0;
    uint32_t err_code;
    uint32_t data_len;



    if (btf == 0) {     /* Sense call */
        /* Return stream status (0: Busy, 1: Ready) */
        /* When once it returned ready to sense call, it must accept a byte at least */
        /* at subsequent transfer call, or f_forward will fail with FR_INT_ERR. */
        err_code = app_fifo_write(&audio_stream, NULL, &data_len);
        // Check if request was successful
        if (err_code == NRF_SUCCESS)
        {
            cnt = 1;
        }
    }
    else {              /* Transfer call */
        do {    /* Repeat while there is any data to be sent and the stream is ready */
            err_code = app_fifo_put(&audio_stream, *p++);
            //err_code = app_fifo_write(&audio_stream, p, &data_len);
            // Check if write was successful
            if (err_code == NRF_SUCCESS)
            {
                cnt++;
            }
            else {
            NRF_LOG_INFO("fifo error");
            NRF_LOG_FLUSH();
            }
            
        } while (cnt < btf &&  (app_fifo_write(&audio_stream, NULL, &data_len) == NRF_SUCCESS));
    }

    return cnt;
}


static bool i2s_sgtl5000_driver_evt_handler(drv_sgtl5000_evt_t * p_evt)
{
    bool ret = false;
    ret_code_t return_val = NRF_SUCCESS;
    ret_code_t err_code;
    uint32_t data_len;
    UINT dmy = 0;
    FRESULT rc;

    //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler %d", p_evt->evt);

    switch (p_evt->evt)
    {
        case DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler RX BUF RECEIVED");
                // TODO: Handle RX as desired - for this example, we dont use RX for anything
                //uint16_t * p_buffer  = (uint16_t *) p_evt->param.rx_buf_received.p_data_received;
            }
            break;
        case DRV_SGTL5000_EVT_I2S_TX_BUF_REQ:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler TX BUF REQ");
                
                /* Play sample! 16kHz sample played on 32kHz frequency! If frequency is changed, this approach needs to change. */
                /* Playback of this 16kHz sample depends on I2S MCK, RATIO, Alignment, format, and channels! Needs to be DIV8, RATIO 128X, alignment LEFT, format I2S, channels LEFT. */
                
                uint32_t * p_buffer  = (uint32_t *) p_evt->param.tx_buf_req.p_data_to_send;
                uint32_t i2s_buffer_size_words = p_evt->param.rx_buf_received.number_of_words;
                int32_t pcm_stream[i2s_buffer_size_words];  // int16_t - i2s_buffer_size_words size; means we only cover half of data_to_send_buffer, which is fine since we are only using LEFT channel
                
                /* Clear pcm buffer */
                memset(pcm_stream, 0, sizeof(pcm_stream));

                /* Check if playing the next part of the sample will exceed the sample size, if not, copy over part of sample to be played */
                if (sample_idx < SAMPLE_LENGTH)
                {
                    /* Copy sample bytes into pcm_stream (or remaining part of sample). This should fill up half the actual I2S transmit buffer. */
                    /* We only want half becuase the sample is a 16kHz sample, and we are running the SGTL500 at 32kHz; see DRV_SGTL5000_FS_31250HZ */
                    uint32_t bytes_to_copy = ((sample_idx + sizeof(pcm_stream)) < SAMPLE_LENGTH) ? sizeof(pcm_stream) : SAMPLE_LENGTH - sample_idx;
 
                    
                    err_code = app_fifo_read(&audio_stream, NULL, &data_len);
                    if ( err_code == NRF_SUCCESS) {
                      if (data_len >= bytes_to_copy) {
                        
                        return_val = app_fifo_read(&audio_stream, pcm_stream, &bytes_to_copy);
                      }
                      else
                      {
                        err_cnt++;
                        NRF_LOG_INFO("not enough data in FIFO %d", err_cnt);
                        NRF_LOG_FLUSH();
                      }
                    }
                    else { 
                      NRF_LOG_INFO("fifo full");
                      NRF_LOG_FLUSH();
                    }
                    //memcpy(pcm_stream, &data[sample_idx], bytes_to_copy);
                    //memcpy(pcm_stream, &p_sample[sample_idx], bytes_to_copy);
                    sample_idx += bytes_to_copy;
                    ret = true;
                }
                else 
                {
                    /* End of buffer reached. */
                    NRF_LOG_INFO("end of buffer");
                    NRF_LOG_FLUSH();
                    sample_idx = 0;
                    ret = false;
                }
                
                /* Upsample the decompressed audio */
                /* i < i2s_buffer_size_words * 2 because we have a uint16_t buffer pointer */
                for (int i = 0, pcm_stream_idx = 0; i < i2s_buffer_size_words ; i += 1)
                {
                    //for (int j = i; j < (i + 1); ++j)
                    //{
                        p_buffer[i] = pcm_stream[pcm_stream_idx];
                    //}
                    ++pcm_stream_idx;
                }
            }
            break;
    }

    return ret;
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    //rf_gpio_pin_clear(LED_1);
    //nrf_gpio_pin_clear(LED_2);
    //nrf_gpio_pin_clear(LED_3);
    app_error_save_and_stop(id, pc, info);
}




#define UART_TX_BUF_SIZE                6                                           /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                6                                           /**< UART RX buffer size. */

// PWM0 Pin Definitions
#define PWM0_OUTPUT_PIN_1 LED_R
#define PWM0_OUTPUT_PIN_2 LED_G
#define PWM0_OUTPUT_PIN_3 LED_B
#define PWM0_OUTPUT_PIN_4 LED_W
#define PWM0_OUTPUT_PIN_5 LED_WW

//Communication Constants -- 3 byte arrays sent from mobile application over NUS
//Byte 1 - Command - "this is a command"
#define COMMAND     0x2

//Byte 2 - Status  - "this is the status of the somadome"
#define BREATHING   0x2
#define STANDBY     0x1
#define OFF         0x0
#define FAN         0x3
#define PLAY        0x4
#define VOLUME      0x5
#define REMOTE      0x6

//Byte 3 - Track Message - "notify which track for color selection"
#define STOP        0x0
#define TRACK1      0x1  //light blue -- Stress Free, Brain Massage
#define TRACK2      0x2  //dark blue  -- Brain Power, Success
#define TRACK3      0x3  //fuchsia    -- Universal Mind, Love
#define TRACK4      0x4  //green      -- Nourish, Weight Loss
#define TRACK5      0x5  //warm-white test
#define INTRO       0x6
#define TEST        0xD
#define START       0xE
#define FAN_ON  0xFF
#define FAN_OFF 0

uint8_t uart_rx_buffer[UART_RX_BUF_SIZE];

bool enablePWM = true;
bool pausePWM = false;
bool postRunning =false;
bool sendOK = false;
bool rxOK = true;
bool playingTrack = false;
bool intro = false;
bool tmaxEnabled = true;
bool sessionDisconnected;
uint8_t evt_cnt = 0;

//BLE Defines
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "SOMADOME-"                                  /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

//#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                1800                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_ADV_TIMEOUT_IN_SECONDS      1800                                         /**< The advertising timeout in units of seconds. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static void serial_event_handler(struct nrf_serial_s const *p_serial, nrf_serial_event_t event);

static void sleep_handler(void)
{
    __WFE();
    __SEV();
    __WFE();
}

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      RX_PIN_NUMBER,
                      TX_PIN_NUMBER,
                      31, 31,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_9600,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart1_drv_config,
                      8,
                      12,
                      31, 31,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_38400,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 3

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_QUEUES_DEF(mp3_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 3
#define SERIAL_BUFF_RX_SIZE 1



NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(mp3_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, *serial_event_handler, sleep_handler);

NRF_SERIAL_CONFIG_DEF(mp3_config, NRF_SERIAL_MODE_IRQ,
                      &mp3_queues, &mp3_buffs, NULL, sleep_handler);

NRF_SERIAL_UART_DEF(serial_uart, 0);
NRF_SERIAL_UART_DEF(mp3_uart, 0);


APP_TIMER_DEF(burn_in_timer_id);
APP_TIMER_DEF(post_timer_id);
APP_TIMER_DEF(cli_timer_id);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                            /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static void advertising_start(bool erase_bonds);                                    /**< Forward declaration of advertising start function */

#define CLI_EXAMPLE_MAX_CMD_CNT (20u)
#define CLI_EXAMPLE_MAX_CMD_LEN (33u)



/* buffer holding dynamicly created user commands */
static char m_dynamic_cmd_buffer[CLI_EXAMPLE_MAX_CMD_CNT][CLI_EXAMPLE_MAX_CMD_LEN];
/* commands counter */
static uint8_t m_dynamic_cmd_cnt;

static char current_color[10] = {'F','F','F','F','F','F','0','0','0','0'};
static bool fan_status = false;
static bool heater_status = false;


static uint32_t m_counter;
static bool m_counter_active = false;


// Instance for UART0
nrf_drv_uart_t uart_driver_instance = NRF_DRV_UART_INSTANCE(0);

// Instance for PWM Module 0
static nrf_drv_pwm_t pwm_m0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t pwm_m1 = NRF_DRV_PWM_INSTANCE(1);

// Declare variables holding PWM sequence values
nrf_pwm_values_individual_t seq_values[] = {{ 0, 0, 0, 0 }};
nrf_pwm_values_individual_t pwm1_seq_values[] = {{ 0, 0, 0, 0 }};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};

nrf_pwm_sequence_t const seq1 =
{
    .values.p_individual = pwm1_seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(pwm1_seq_values),
    .repeats         = 0,
    .end_delay       = 0
};

// Set duty cycle between 0 and 100%
void pwm_update_duty_cycle(uint8_t red, uint8_t green, uint8_t blue, uint8_t white, uint8_t w_white)
{
    // Check if value is outside of range. If so, set to 100%
    seq_values->channel_0 = red | 0x8000;
    seq_values->channel_1 = green | 0x8000;
    seq_values->channel_2 = blue | 0x8000;
    seq_values->channel_3 = white | 0x8000;
    pwm1_seq_values->channel_0 = w_white | 0x8000;
    pwm1_seq_values->channel_1 = 0 | 0x8000;
    pwm1_seq_values->channel_2 = 0 | 0x8000;
    pwm1_seq_values->channel_3 = 0 | 0x8000;

    //if (w_white != 255) 
     nrf_drv_pwm_simple_playback(&pwm_m0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    //else {
     nrf_drv_pwm_simple_playback(&pwm_m1, &seq1, 1, NRF_DRV_PWM_FLAG_LOOP);
     // }
}

static void pwm_init(void)
{
    ret_code_t err_code;
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            PWM0_OUTPUT_PIN_1, // channel 0 - RED
            PWM0_OUTPUT_PIN_2, // channel 1 - GREEN
            PWM0_OUTPUT_PIN_3, // channel 2 - BLUE
            PWM0_OUTPUT_PIN_4, // channel 3 - WHITE
            
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 255,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    nrf_drv_pwm_config_t const config1 =
    {
        .output_pins =
        {
            PWM0_OUTPUT_PIN_5, // channel 0 - ww
            31,
            31,
            31
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 255,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    // Init PWM without error handler
    err_code = nrf_drv_pwm_init(&pwm_m0, &config0, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_pwm_init(&pwm_m1, &config1, NULL);
    APP_ERROR_CHECK(err_code);

    //pwm_update_duty_cycle(0,0,0,0,0);
}



//// YOUR_JOB: Use UUIDs for service(s) used in your application.
//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static ble_uuid_t m_adv_uuids[] = {
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
};


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


// Timeout handler for the POST timer
static void post_timer_handler(void * p_context)
{
          //NRF_LOG_RAW_INFO("POST (Power on self test) sequence initiated...\r\n");
          postRunning = true;
          
          nrf_gpio_pin_set(FAN_PIN); // fan on 
          NRF_LOG_RAW_INFO("Fan Test Started...will run for duration of POST\r\n");
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          nrf_delay_ms(1000);
          pwm_update_duty_cycle(0,255,255,0,0);  // light blue
          NRF_LOG_RAW_INFO("2000ms LED Light Blue Test Started.\r\n");
          nrf_delay_ms(1000);
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          NRF_LOG_RAW_INFO("2000ms LED Light Blue Test Completed.\r\n");
          pwm_update_duty_cycle(0,0,255,0,0); // blue
          NRF_LOG_RAW_INFO("2000ms LED Blue Test Started.\r\n");
          nrf_delay_ms(1000);
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          NRF_LOG_RAW_INFO("2000ms LED Blue Test Completed.\r\n");
          pwm_update_duty_cycle(255,0,255,0,0); // fuschia
          NRF_LOG_RAW_INFO("2000ms LED Fuschia Test Started.\r\n");
          nrf_delay_ms(1000);
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          NRF_LOG_RAW_INFO("2000ms LED Fuschia Test Completed.\r\n");
          pwm_update_duty_cycle(0,255,0,0,0); // green
          NRF_LOG_RAW_INFO("2000ms LED Green Test Started.\r\n");
          nrf_delay_ms(1000);
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          NRF_LOG_RAW_INFO("2000ms LED Green Test Completed.\r\n");
          pwm_update_duty_cycle(0,0,0,255,0); // white
          NRF_LOG_RAW_INFO("2000ms LED White Test Started.\r\n");
          nrf_delay_ms(1000);
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          NRF_LOG_RAW_INFO("2000ms LED White Test Completed.\r\n");
          //pwm_update_duty_cycle(0,0,0,0,255); // warm white 
          //NRF_LOG_RAW_INFO("2000ms LED Warm White Test Started.\r\n");
          nrf_delay_ms(1000); 
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          NRF_LOG_RAW_INFO("2000ms LED Warm White Test Completed.\r\n");
          pwm_update_duty_cycle(0,0,0,0,0); // LED off 
          nrf_delay_ms(1000); 
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          NRF_LOG_RAW_INFO("1000ms seat heater relay test started.\r\n");
          //nrf_gpio_pin_set(PLAY_OUT);
          nrf_delay_ms(1000);
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          nrf_gpio_pin_clear(22);
          NRF_LOG_RAW_INFO("1000ms power relay test completed.\r\n");
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          pwm_update_duty_cycle(255,255,255,255,0); // white
          // fan off 
          nrf_gpio_pin_clear(FAN_PIN);
          NRF_LOG_RAW_INFO("POST (Power on self test) sequence completed!\r\n");
          UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
          postRunning = false;
          sendOK = true;
          nrf_drv_gpiote_in_event_enable(4, true);
}

// Timeout handler for the burnin timer
static void burn_in_timer_handler(void * p_context)
{
    nrf_gpio_pin_set(FAN_PIN); // fan on 
    nrf_delay_ms(1000);
    pwm_update_duty_cycle(0,255,255,127,0);  // light blue
    nrf_delay_ms(2000);
    pwm_update_duty_cycle(0,0,255,0,0); // blue
    nrf_delay_ms(2000);
    pwm_update_duty_cycle(255,0,255,0,0); // fuschia
    nrf_delay_ms(2000);
    pwm_update_duty_cycle(0,255,0,0,0); // green
    nrf_delay_ms(2000);
    pwm_update_duty_cycle(255,255,255,255,0); // white
    nrf_delay_ms(2000);
    pwm_update_duty_cycle(0,0,0,0,255); // warm white 
    nrf_delay_ms(2000); 
    nrf_gpio_pin_clear(FAN_PIN); // fan off
    nrf_delay_ms(1000);
    pwm_update_duty_cycle(0,0,0,0,0); // off
    nrf_delay_ms(2000);   
}


static void timer_handle(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_counter_active)
    {
        m_counter++;
        NRF_LOG_RAW_INFO("counter1 = %d\r\n", m_counter);
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;
    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

  
     err_code = app_timer_create(&burn_in_timer_id, APP_TIMER_MODE_REPEATED, burn_in_timer_handler);
     APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&post_timer_id, APP_TIMER_MODE_SINGLE_SHOT, post_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&cli_timer_id, APP_TIMER_MODE_REPEATED, timer_handle);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    // Initalize variables for dynamic name
    ble_gap_addr_t          ble_gap_addr;
    char                    ble_gap_addr_lap[7];
    uint8_t                 ble_devicename[15]; 

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);


    // Get mac address and use second half for dynamic address
    err_code = sd_ble_gap_addr_get(&ble_gap_addr);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("BLE ID: %02X%02X%02X%02X%02X%02X",ble_gap_addr.addr[5],ble_gap_addr.addr[4],ble_gap_addr.addr[3],ble_gap_addr.addr[2],ble_gap_addr.addr[1],ble_gap_addr.addr[0]);
    snprintf(ble_gap_addr_lap,  7, "%02X%02X%02X", ble_gap_addr.addr[2], ble_gap_addr.addr[1], ble_gap_addr.addr[0]);
    memcpy(&ble_devicename, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    memcpy(&ble_devicename[9], ble_gap_addr_lap,6);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          ble_devicename,
                                          15);
    APP_ERROR_CHECK(err_code);

//    err_code = sd_ble_gap_device_name_set(&sec_mode,
//                                          (const uint8_t *)DEVICE_NAME,
//                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
   static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                          ble_yy_service_evt_t * p_evt)
   {
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
   }*/



static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint32_t err_code;
    
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        
        char command = p_evt->params.rx_data.p_data[0];
        char status = p_evt->params.rx_data.p_data[1];
        char select = p_evt->params.rx_data.p_data[2];

        if (command == COMMAND || command == 'c') {
            if(status == BREATHING || status == 'b') {
                if(select == TRACK1 || select == 'l') {
                    // Light Blue
                    enablePWM = true;
                    SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                    pwm_update_duty_cycle(0,255,255,127,0);
                }
                if(select == TRACK2 || select == 'b') {
                    // Blue
                    enablePWM = true;
                    pwm_update_duty_cycle(0,0,255,0,0);
                    //enablePWM = true;
                    SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                }
                if(select == TRACK3 || select == 'p') {
                    // Fuschia
                    enablePWM = true;
                    pwm_update_duty_cycle(255,0,255,0,0);
                    SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                }
                if(select == TRACK4 || select == 'g') {
                    // Green
                    enablePWM = true;
                    pwm_update_duty_cycle(0,255,0,0,0);
                    SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                }
                if(select == TRACK5 || select == 'k') {
                    // Low K white
                    enablePWM = true;
                    pwm_update_duty_cycle(0,0,0,0,255);
                    SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                }
            }
            if(status == STANDBY || status == 's') {
                if(select == TRACK2 || select == 'w') {
                    // White
                    enablePWM = true;
                    pwm_update_duty_cycle(255,255,255,0,0);
                    SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                }
                if(select == OFF || select == 'o') {
                    enablePWM = true;
                    pwm_update_duty_cycle(0,0,0,0,0);
                    SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                }
               if(select == 'b') {
                    err_code = app_timer_start(burn_in_timer_id, APP_TIMER_TICKS(1000), NULL);
                    APP_ERROR_CHECK(err_code);
                }
                if(select == 's') {
                    err_code = app_timer_stop(burn_in_timer_id);
                    APP_ERROR_CHECK(err_code);
                }
            }
            if(status == FAN || status == 'f') {
                if(select == FAN_OFF || select == '1') {
                    nrf_gpio_pin_clear(FAN_PIN);

                }
                else if (select == FAN_ON || select == '2') {
                  nrf_gpio_pin_set(FAN_PIN);
                }
            }

        }
    }
}
/**@snippet [Handling the data received over BLE] */



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t                  err_code;


    nrf_ble_qwr_init_t        qwr_init  = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};
    ble_nus_init_t nus_init;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize the async SVCI interface to bootloader.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Add code to initialize the services used by the application.
       uint32_t                           err_code;
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    
       uint32_t err_code;

       err_code = app_timer_start(post_timer_id, APP_TIMER_TICKS(100), NULL);
       APP_ERROR_CHECK(err_code);

//       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
//       APP_ERROR_CHECK(err_code); 
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);

    //Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
    //GPREGRET2 register holds the information about skipping CRC check on next boot.
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            // LED indication will be changed when advertising starts.
            pwm_update_duty_cycle(255,255,255,0,0);
            nrf_gpio_pin_clear(FAN_PIN);
            //sd_ble_gap_adv_stop();
            //sessionDisconnected = true;
            break;

        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            pwm_update_duty_cycle(0,0,0,0,0);
            nrf_delay_ms(500);
            pwm_update_duty_cycle(255,255,255,0,0);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;



        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    uint32_t err_code;
    bsp_event_t startup_event;

//    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEBUG("advertising is started");
    }
}

static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void send_tmax_command(nrf_gpiote_polarity_t button_action)
{
    if(button_action == APP_BUTTON_PUSH)
    {
        NRF_LOG_RAW_INFO("%d: TMAX TIMER START DETECTED\n", evt_cnt);
        evt_cnt++;
        if (sendOK) 
        {
          static char tx_message[] = "$";
          APP_ERROR_CHECK(nrf_serial_write(&serial_uart,
                          tx_message,
                          strlen(tx_message),
                          NULL,
                          NRF_SERIAL_MAX_TIMEOUT));
        }
    }
    if(button_action == APP_BUTTON_RELEASE)
    {
        NRF_LOG_RAW_INFO("%d: TMAX TIMER END DETECTED\n", evt_cnt);
        evt_cnt++;
        if (sendOK) 
        {
          static char tx_message[] = "+";
          APP_ERROR_CHECK(nrf_serial_write(&serial_uart,
                          tx_message,
                          strlen(tx_message),
                          NULL,
                          NRF_SERIAL_MAX_TIMEOUT));
        }
    }
}

static void tmax_event_handler(uint8_t pin_no, nrf_gpiote_polarity_t button_action)
{
    
    if(pin_no == SYS_LOCK )
    {
        send_tmax_command(button_action);
    }
    
        //if ( tmaxEnabled ) {
        //  if (sessionDisconnected == false) {
        //    err_code = sd_ble_gap_adv_stop();
         //   APP_ERROR_CHECK(err_code);
         // }
        //  tmaxEnabled = false;
       // }
       // else {
       //   err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
       //   APP_ERROR_CHECK(err_code);
       //   tmaxEnabled = true;
       //   sessionDisconnected = false;
    //    }
   
}


static void gpio_init(void)
{
    ret_code_t err_code;
    nrf_gpio_cfg_output(FAN_PIN);
    nrf_gpio_cfg_output(PLAY_OUT);
    nrf_gpio_pin_clear(FAN_PIN); 
}

static void buttons_init()
{
    ret_code_t err_code;

    static app_button_cfg_t button_cfg[1] = {
        {SYS_LOCK,APP_BUTTON_ACTIVE_LOW,NRF_GPIO_PIN_PULLUP,tmax_event_handler}
    };

    err_code = app_button_init(button_cfg,1,100);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}



static void serial_init()
{
    
    APP_ERROR_CHECK(nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config));
    
}

static void mp3_init()
{
    APP_ERROR_CHECK(nrf_serial_uninit(&serial_uart));
    APP_ERROR_CHECK(nrf_serial_init(&mp3_uart, &m_uart1_drv_config, &serial_config));
}

static void mp3_uninit()
{
    APP_ERROR_CHECK(nrf_serial_uninit(&mp3_uart));
    APP_ERROR_CHECK(nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config));
    
}
static void serial_uinit()
{
    
    APP_ERROR_CHECK(nrf_serial_uninit(&serial_uart));
}


static void send_mp3_command(char command, uint8_t option) {
    uint8_t err_code;
    uint8_t tx_message[2];
    rxOK = false;
    mp3_init();
    //nrf_delay_ms(2000);
    if (sendOK) 
    {
      tx_message[0] = command;
      if (option) 
        tx_message[1] = option;

      err_code = nrf_serial_write(&mp3_uart,
                      tx_message,
                      strlen(tx_message),
                      NULL,
                      NRF_SERIAL_MAX_TIMEOUT);
    }
    nrf_delay_ms(1000);
    APP_ERROR_CHECK(err_code);
    mp3_uninit();
    rxOK = true;
}

FRESULT play_file (
    char *fn        /* Pointer to the audio file name to be played */
)
{
    //FRESULT rc;
    //FIL fil;
    UINT dmy = 0;
    UINT ready;
    uint32_t data_len;
    ret_code_t err_code;
    fatfs_init();
    NRF_LOG_INFO("FAT Initialized on Play Command");
    NRF_LOG_FLUSH();
    sample_idx          = 0;
    ff_result = f_open(&file, fn, FA_READ);
    
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open file: %s.", fn);
        NRF_LOG_FLUSH();
        
    }
    SAMPLE_LENGTH = f_size(&file);
    drv_sgtl5000_stop();
    NRF_LOG_INFO("Audio Driver Stopped");
    NRF_LOG_FLUSH();
    drv_sgtl5000_start();
    NRF_LOG_INFO("Audio Driver Started");
    NRF_LOG_FLUSH();

    /* Open the audio file in read only mode */
    rc = f_open(&file, fn, FA_READ);
    if (rc) return rc;

    /* Repeat until the file pointer reaches end of the file */
//    while (rc == FR_OK && !f_eof(&file)) {
//
//        err_code = app_fifo_write(&audio_stream, NULL, &data_len);
//        if (err_code == NRF_SUCCESS) {
//
//          rc = f_forward(&file, audio_stream_handler, 10000, &dmy);
//          err_code = dmy;
//
//        }
//
//        
//    }

    /* Close the file and return */
    //f_close(&fil);
    //return rc;
}

static void serial_event_handler(struct nrf_serial_s const *p_serial, nrf_serial_event_t event) {

      ret_code_t err_code;
      uint8_t size;
      uint8_t data;

      switch (event)
	{
	case NRF_SERIAL_EVENT_TX_DONE:
	    break;
	case NRF_SERIAL_EVENT_RX_DATA:

  
	    //Read one char from queue
            size = nrf_queue_utilization_get(p_serial->p_ctx->p_config->p_queues->p_rxq);
            if (size == 1) {
              err_code = nrf_queue_generic_pop(p_serial->p_ctx->p_config->p_queues->p_rxq, &data, true);
              if (data != 'c') {
                //nrf_serial_rx_drain(&serial_uart);
                err_code = nrf_queue_generic_pop(p_serial->p_ctx->p_config->p_queues->p_rxq, &data, false);
                size = nrf_queue_utilization_get(p_serial->p_ctx->p_config->p_queues->p_rxq);
                NRF_LOG_INFO("Invalid command byte, drained");
                NRF_LOG_INFO("New Queue Size: %d", size);
                NRF_LOG_FLUSH();
              }
            }
            if (size == 3) {
              err_code = nrf_queue_read(p_serial->p_ctx->p_config->p_queues->p_rxq, &rx_bytes, 3);
              NRF_LOG_INFO("bytes read: %c%c%x\r", rx_bytes[0], rx_bytes[1],rx_bytes[2]);
              NRF_LOG_FLUSH();
              if ((rx_bytes[1] == 'v') && (rx_bytes[2] > 64) ) {
                  rx_bytes[2] = 1;
                  nrf_serial_rx_drain(&serial_uart);
                  nrf_queue_reset(p_serial->p_ctx->p_config->p_queues->p_rxq);
                  size = nrf_queue_utilization_get(p_serial->p_ctx->p_config->p_queues->p_rxq);
                  NRF_LOG_INFO("New Queue Size: %d", size);
                  NRF_LOG_FLUSH();
              }
              rxOK = 1;
            }
	    break;
	case NRF_SERIAL_EVENT_DRV_ERR:
	    break;
	case NRF_SERIAL_EVENT_FIFO_ERR:
	    break;
	default:
	    break;
	}

}

/**@brief Function for application main ent
 */
void main(void) {

    uint32_t err_code;

    uint32_t data_len;
    UINT dmy = 0;
    rxOK = 0;
    bool erase_bonds;
    gpio_init();
    log_init();
    serial_init();
    timers_init();
    power_management_init();
    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    buttons_init();
    fatfs_init();
    NRF_LOG_FLUSH();
    err_code = app_fifo_init(&audio_stream, buffer, (uint16_t)sizeof(buffer));
    //audio_init();

    // Enable audio
    drv_sgtl5000_init_t sgtl_drv_params;
    sgtl_drv_params.i2s_tx_buffer           = (void*)m_i2s_tx_buffer;
    sgtl_drv_params.i2s_rx_buffer           = (void*)m_i2s_rx_buffer;
    sgtl_drv_params.i2s_buffer_size_words   = I2S_BUFFER_SIZE_WORDS/2;
    sgtl_drv_params.i2s_evt_handler         = i2s_sgtl5000_driver_evt_handler;
    sgtl_drv_params.fs                      = DRV_SGTL5000_FS_31250HZ;
    
    m_i2s_tx_buffer[0] = 167;
    m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS/2] = 167;
    NRF_LOG_INFO("size of  m_i2s_tx_buffer %d, %d", sizeof(m_i2s_tx_buffer) / sizeof(uint32_t), I2S_BUFFER_SIZE_WORDS);
    NRF_LOG_INFO("i2s_initial_tx_buffer addr1: %d, addr2: %d", m_i2s_tx_buffer, m_i2s_tx_buffer + I2S_BUFFER_SIZE_WORDS/2);
    NRF_LOG_INFO("i2s_initial_Rx_buffer addr1: %d, addr2: %d", m_i2s_rx_buffer, m_i2s_rx_buffer + I2S_BUFFER_SIZE_WORDS/2);
    
    drv_sgtl5000_init(&sgtl_drv_params);
    
    //drv_sgtl5000_stop();
    NRF_LOG_INFO("Audio initialization done.");
    //m_state = SGTL5000_STATE_IDLE;
    
    drv_sgtl5000_volume_set((float)32);

    //err_code = nrf_cli_init(&m_cli_rtt, NULL, true, true, NRF_LOG_SEVERITY_INFO);
    //APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Application started\n");  
    tmaxEnabled = false;
    // Start execution.
    application_timers_start();
    advertising_start(erase_bonds);
    bool pwmEnabled = false;
    

    //cli_start();

    //NRF_LOG_RAW_INFO("Somadome Command Line Interface.\r\n");
    //NRF_LOG_RAW_INFO("Please press the Tab key to see all available commands.\r\n");

    
    // Enter main loop.
    for (;;) {
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
        if (!pausePWM) 
        {

          // enable disable as needed
          if(!enablePWM) 
          {
            if(pwmEnabled) 
            {
              nrf_drv_pwm_uninit(&pwm_m0);
              nrf_drv_pwm_uninit(&pwm_m1);
              pwmEnabled = false;
            }
          }
          else {
            if(!pwmEnabled) 
            {
              pwm_init();
              
              // nrf_gpio_pin_toggle(22);
              pwmEnabled = true;
              //nrf_drv_uart_tx(&uart_driver_instance, (char *) "$",  1);
            }
          }
        }
        if (playingTrack) {
            if (rc == FR_OK && !f_eof(&file)) {

            err_code = app_fifo_write(&audio_stream, NULL, &data_len);
            if (err_code == NRF_SUCCESS) {

              rc = f_forward(&file, audio_stream_handler, 1280, &dmy);
              err_code = dmy; 

              }

            }
            
            else {
              if ((rc == FR_OK) && (f_eof(&file))) {
                NRF_LOG_INFO("End of File Reached");
                NRF_LOG_FLUSH();
                playingTrack = false;
                drv_sgtl5000_stop();
                NRF_LOG_INFO("Audio Driver Stopped on Finish");
                NRF_LOG_FLUSH();
                f_close(&file);
                err_code = app_fifo_init(&audio_stream, buffer, (uint16_t)sizeof(buffer));
                NRF_LOG_INFO("FIFO Initialized on Finish");
                NRF_LOG_FLUSH();
                app_fifo_flush(&audio_stream);
                fatfs_init();
                NRF_LOG_INFO("FAT Initialized on Finish");
                NRF_LOG_FLUSH();
                sample_idx = 0;
                if (!intro) {
                  static char tx_message[] = "+";
                  APP_ERROR_CHECK(nrf_serial_write(&serial_uart,
                                  tx_message,
                                  strlen(tx_message),
                                  NULL,
                                  NRF_SERIAL_MAX_TIMEOUT));
                }
                else {
                  intro = false;
                }
              }
              if (rc != FR_OK) {
                NRF_LOG_INFO("File Playback Error");
                NRF_LOG_FLUSH();
                drv_sgtl5000_stop();
                NRF_LOG_INFO("Audio Driver Stopped on Error");
                NRF_LOG_FLUSH();
                err_code = app_fifo_init(&audio_stream, buffer, (uint16_t)sizeof(buffer));
                NRF_LOG_INFO("FIFO Initialized on Error");
                NRF_LOG_FLUSH();
                app_fifo_flush(&audio_stream);
                playingTrack = false;
                //fclose(&file);
                fatfs_init();
                NRF_LOG_INFO("FAT Initialized on Error");
                NRF_LOG_FLUSH();
                sample_idx = 0;
              }
            }
        }
        if (rxOK) {
            char command = rx_bytes[0];
            char status = rx_bytes[1];
            char select = rx_bytes[2];
            if (command == COMMAND || command == 'c') {
                if (status == BREATHING || status == 'b') {
                    if (select == TRACK1 || select == 'l') {
                        // Focus
                        enablePWM = true;
                        pwm_update_duty_cycle(0,255,255,127,0); // light blue
                        NRF_LOG_INFO("light blue");
                    }
                    if(select == TRACK2 || select == 'b') {
                        // Blue
                        enablePWM = true;
                        NRF_LOG_INFO("led ");
                        pwm_update_duty_cycle(0,0,255,0,0);
                        //enablePWM = true;
                        //SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                    }
                    if(select == TRACK3 || select == 'p') {
                        // Fuschia
                        enablePWM = true;
                        NRF_LOG_INFO("led ");
                        pwm_update_duty_cycle(255,0,255,0,0);
                        //SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                    }
                    if(select == TRACK4 || select == 'g') {
                        // Green
                        enablePWM = true;
                        NRF_LOG_INFO("led ");
                        pwm_update_duty_cycle(0,255,0,0,0);
                        //SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                    }
                    if(select == TRACK5 || select == 'k') {
                        // Low K white
                        enablePWM = true;
                        NRF_LOG_INFO("led ");
                        pwm_update_duty_cycle(0,0,0,0,255);
                        //SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                    }
                }
                if(status == STANDBY || status == 's') {
                    if(select == TRACK2 || select == 'w') {
                        // White
                        enablePWM = true;
                        NRF_LOG_INFO("led ");
                        pwm_update_duty_cycle(255,255,255,0,0);

                        //SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                    }
                    if(select == OFF || select == 'o') {
                        enablePWM = true;
                        pwm_update_duty_cycle(0,0,0,0,0);
                        //SEGGER_RTT_WriteString(0, (char*)p_evt->params.rx_data.p_data);
                    }
                    if(select == 'b') {
                        err_code = app_timer_start(burn_in_timer_id, APP_TIMER_TICKS(1000), NULL);
                        APP_ERROR_CHECK(err_code);
                    }
                    if(select == 's') {
                        err_code = app_timer_stop(burn_in_timer_id);
                        APP_ERROR_CHECK(err_code);
                    }
                }
                if(status == FAN || status == 'f') {
                    if(select == FAN_OFF || select == '1') {
                        nrf_gpio_pin_clear(FAN_PIN);
                        NRF_LOG_INFO("fan off");

                    }
                    else if (select == FAN_ON || select == '2') {
                        nrf_gpio_pin_set(FAN_PIN);
                        NRF_LOG_INFO("fan on");
                    }
                }
                if(status == REMOTE || status == 'r') {
                    if(select == START || select == 's') {
                        nrf_gpio_pin_set(PLAY_OUT);
                        nrf_delay_ms(50);
                        nrf_gpio_pin_clear(PLAY_OUT);
                        NRF_LOG_INFO("Remote Start pressed");

                    }
                }
                if (status == VOLUME || status == 'v') {
                    //send_mp3_command('v', select);
                    NRF_LOG_INFO("Set volume %s", select);
                    drv_sgtl5000_stop();
                    NRF_LOG_INFO("Audio Driver Stopped for Volume");
                    NRF_LOG_FLUSH();
                    drv_sgtl5000_volume_set(select);
                    drv_sgtl5000_start();
                    NRF_LOG_INFO("Audio Driver Started for Volume");
                    NRF_LOG_FLUSH();
                }
                if ((status == PLAY || status == 't')) {
                    if (select != 's') {
                      if (playingTrack) {
                        drv_sgtl5000_stop();
                        NRF_LOG_INFO("Audio Driver Stopped on Track Change");
                        NRF_LOG_FLUSH();
                        app_fifo_flush(&audio_stream);
                        f_close(&file);
                        fatfs_init();
                        playingTrack = false;
                      }
                      //else {
                      //  fatfs_init();
                      //}
                    }
                    if (select == INTRO || select == 'k') {
                        //send_mp3_command('t', 0x0E);
                        NRF_LOG_INFO("Playing Intro");
                        rc = play_file("_Intro.raw");
                        playingTrack = true;
                        intro = true;

                    }
                    if (select == TEST || select == 'l') {
                        //send_mp3_command('t',0x0D);
                        NRF_LOG_INFO("Playing Test Bell");
                        rc = play_file("_TestBell.raw");
                        playingTrack = true;

                    }
                    if (select == '1') {
                        //send_mp3_command('t',0x07);
                        NRF_LOG_INFO("Playing Focus");
                        rc = play_file("Focus.raw");
                        playingTrack = true;
                    }
                    if ( select == '2') {
                        //send_mp3_command('t',0x07);
                        NRF_LOG_INFO("Playing Motivate");
                        rc = play_file("Motivate.raw");
                        playingTrack = true;
                    }
                    if ( select == '3') {
                        //send_mp3_command('t', 0x06);
                        NRF_LOG_INFO("Playing Fit");
                        rc = play_file("Fit.raw");
                        playingTrack = true;
                    }
                    if ( select == '4') {
                        //send_mp3_command('t', 0x03);
                        NRF_LOG_INFO("Playing Perform");
                        rc = play_file("Perform.raw");
                        playingTrack = true;
                    }
                    if (select == '5') {
                        //send_mp3_command('t', 0xB);
                        NRF_LOG_INFO("Playing Heal");
                        rc = play_file("Heal.raw");
                        playingTrack = true;
                    }
                    if ( select == '6') {
                        //send_mp3_command('t',0xA);
                        NRF_LOG_INFO("Playing Recharge");
                        rc = play_file("Recharge.raw");
                        playingTrack = true;
                    }
                    if (select == '7') {
                        //send_mp3_command('t',0x07);
                        NRF_LOG_INFO("Playing Relax");
                        rc = play_file("Relax.raw");
                        playingTrack = true;
                    }
                    if (select == '8') {
                        //send_mp3_command('t',0x07);
                        NRF_LOG_INFO("Playing Snooze");
                        rc = play_file("Snooze.raw");
                        playingTrack = true;
                    }
                    if (select == '9') {
                        //send_mp3_command('t', 0x06);
                        NRF_LOG_INFO("Playing Create");
                        rc = play_file("Creativity.raw");
                        playingTrack = true;
                    }
                    if (select == 'a') {
                        //send_mp3_command('t', 0x0B);
                        NRF_LOG_INFO("Playing Overcome");
                        rc = play_file("Overcome.raw");
                        playingTrack = true;
                    }
                    if (select == 'b') {
                        //send_mp3_command('t',0x0A);
                        NRF_LOG_INFO("Playing Succeed");
                        rc = play_file("Succeed.raw");
                        playingTrack = true;
                    }
                    if (select == 'c') {
                        //send_mp3_command('t',0x07);
                        NRF_LOG_INFO("Playing Ascend");
                        rc = play_file("Ascend.raw");
                        playingTrack = true;
                    }
                    if (select == 'd') {
                        //send_mp3_command('t',0x07);
                        NRF_LOG_INFO("Playing Aspire");
                        rc = play_file("Aspire.raw");
                        playingTrack = true;

                    }
                    if (select == 'e') {
                        //send_mp3_command('t', 0x06);
                        NRF_LOG_INFO("Playing Bliss");
                        rc = play_file("Bliss.raw");
                        playingTrack = true;
                    }
                    if (select == 'f') {
                        //send_mp3_command('t', 0x03);
                        NRF_LOG_INFO("Playing Clarity");
                        rc = play_file("Clarity.raw");
                        playingTrack = true;
                    }
                    if (select == 'g') {
                        //send_mp3_command('t', 0xB);
                        NRF_LOG_INFO("Playing Confidence");
                        rc = play_file("Confidence.raw");
                        playingTrack = true;
                    }
                    if (select == 'h') {
                        //send_mp3_command('t',0xA);
                        NRF_LOG_INFO("Playing Love");
                        rc = play_file("Love.raw");
                        playingTrack = true;
                    }
                    if (select == 'i') {
                        //send_mp3_command('t', 0xB);
                        NRF_LOG_INFO("Playing Manifest");
                        rc = play_file("Manifest.raw");
                        playingTrack = true;
                    }
                    if ( select == 'j') {
                        //send_mp3_command('t',0xA);
                        NRF_LOG_INFO("Playing Prosper");
                        rc = play_file("Prosperity.raw");
                        playingTrack = true;
                    }
                    if (select == '0') {
                        //send_mp3_command('t', 0x03);
                        NRF_LOG_INFO("Playing Reclaim");
                        rc = play_file("Reclaim.raw");
                        playingTrack = true;
                    }
                    if (select == 's') {
                        //send_mp3_command('O', NULL);
                        NRF_LOG_INFO("Stop Track");
                        if (playingTrack) {
                          drv_sgtl5000_stop();
                          NRF_LOG_INFO("Audio Driver Stopped on Stop Command");
                          NRF_LOG_FLUSH();
                          f_close(&file);
                          app_fifo_flush(&audio_stream);
                          fatfs_init();
                          NRF_LOG_INFO("FAT Initialized on Stop Command");
                          NRF_LOG_FLUSH();
                          sample_idx = 0;
                        }
                        playingTrack = false;
                      
                    } // end select
                } // end PLAY status
            } // end command
            memset(&rx_bytes, '\0', 3); // reset command bytes
            rxOK = 0; // wait for new command to be received
        } // end rxOK
    } //end ;;
} // end main
//
///* Command handlers */
//static void cmd_set_color(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    long values[5] = {0,0,0,0,0};
//
//    ASSERT(p_cli);
//    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);
//
//    if ((argc == 1) || nrf_cli_help_requested(p_cli))
//    {
//        nrf_cli_help_print(p_cli, NULL, 0);
//        return;
//    }
//
//    if (argc != 2)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
//        return;//
//    }
//
//    // here is the hex string
//    uint64_t number = (unsigned long long)strtoull(argv[1], NULL, 16);       // number base 16
//    
//    values[0] = (number >> 32) ;
//    values[1] = (number >> 24 & 0xFF);
//    values[2] = (number >> 16 & 0xFF);
//    values[3] = (number >> 8 & 0xFF);
//    values[4] = (number & 0xFF);
//    pwm_update_duty_cycle(values[0],values[1],values[2], values[3], values[4]);
//    sprintf(current_color, "%s", argv[1]);
//}
//
//
//
//static void cmd_set_fans_on(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    nrf_gpio_pin_set(FAN_PIN); // fan on
//    fan_status = true;
//}
//
//static void cmd_set_fans_off(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//
//    nrf_gpio_pin_clear(FAN_PIN); // fan off
//    fan_status = false;
//    
//}
//
//static void cmd_set_fans(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    ASSERT(p_cli);
//    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);
//
//    if ((argc == 1) || nrf_cli_help_requested(p_cli))
//    {
//        nrf_cli_help_print(p_cli, NULL, 0);
//        return;
//    }
//}
//
//static void cmd_set_heater_on(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    nrf_gpio_pin_set(22);
//    heater_status = true;
//}
//
//static void cmd_set_heater_off(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    nrf_gpio_pin_clear(22);
//    heater_status = false;
//}
//
//
//
//static void cmd_set_heater(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    ASSERT(p_cli);
//    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);
//
//    if ((argc == 1) || nrf_cli_help_requested(p_cli))
//    {
//        nrf_cli_help_print(p_cli, NULL, 0);
//        return;
//    }
//}
//
//static void cmd_set_post(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    for (size_t i = 1; i < argc; i++)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "%s ", argv[i]);
//    }
//    nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "\r\n");
//}
//
//static void cmd_show_status(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//      nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "color %s\r\n", current_color);
//      nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "fans %s\r\n", fan_status ? "on" : "off");
//      nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "heater %s\r\n", heater_status ? "on" : "off");
//}
//
//
//
//static void cmd_show(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    ASSERT(p_cli);
//    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);
//
//    if ((argc == 1) || nrf_cli_help_requested(p_cli))
//    {
//        nrf_cli_help_print(p_cli, NULL, 0);
//        return;
//    }
//
//    if (argc != 2)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
//        return;
//    }
//
//    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: unknown parameter: %s\r\n", argv[0], argv[1]);
//}
//
//
//static void cmd_set(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    ASSERT(p_cli);
//    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);
//
//    if ((argc == 1) || nrf_cli_help_requested(p_cli))
//    {
//        nrf_cli_help_print(p_cli, NULL, 0);
//        return;
//    }
//
//    if (argc != 3)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
//        return;
//    }
//
//    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: unknown parameter: %s\r\n", argv[0], argv[1]);
//}
//
//
//
//
///* function required by qsort */
//static int string_cmp(const void * p_a, const void * p_b)
//{
//    ASSERT(p_a);
//    ASSERT(p_b);
//    return strcmp((const char *)p_a, (const char *)p_b);
//}
//
//static void cmd_counter_start(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    if (argc != 1)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
//        return;
//    }
//
//    m_counter_active = true;
//}
//
//static void cmd_counter_stop(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    if (argc != 1)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
//        return;
//    }
//
//    m_counter_active = false;
//}
//
//static void cmd_counter_reset(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    if (argc != 1)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
//        return;
//    }
//
//    m_counter = 0;
//}
//
//static void cmd_counter(nrf_cli_t const * p_cli, size_t argc, char **argv)
//{
//    ASSERT(p_cli);
//    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);
//
//    /* Extra defined dummy option */
//    static const nrf_cli_getopt_option_t opt[] = {
//        NRF_CLI_OPT(
//            "--test",
//            "-t",
//            "dummy option help string"
//        )
//    };
//
//    if ((argc == 1) || nrf_cli_help_requested(p_cli))
//    {
//        nrf_cli_help_print(p_cli, opt, ARRAY_SIZE(opt));
//        return;
//    }
//
//    if (argc != 2)
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: bad parameter count\r\n", argv[0]);
//        return;
//    }
//
//    if (!strcmp(argv[1], "-t") || !strcmp(argv[1], "--test"))
//    {
//        nrf_cli_fprintf(p_cli, NRF_CLI_NORMAL, "Dummy test option.\r\n");
//        return;
//    }
//
//    /* subcommands have their own handlers and they are not processed here */
//    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s: unknown parameter: %s\r\n", argv[0], argv[1]);
//}
//
//NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_heater)
//{
//    NRF_CLI_CMD(on,   NULL, "Set heater on", cmd_set_heater_on),
//    NRF_CLI_CMD(off,   NULL, "Set heater off", cmd_set_heater_off),
//    NRF_CLI_SUBCMD_SET_END
//};
//
//
//NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_fans)
//{
//    NRF_CLI_CMD(on,   NULL, "Set fans on", cmd_set_fans_on),
//    NRF_CLI_CMD(off,   NULL, "Set fans off", cmd_set_fans_off),
//    NRF_CLI_SUBCMD_SET_END
//};
//
//NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_set)
//{
//    NRF_CLI_CMD(color,   NULL, "Set color by hex code (FFFFFFFF)", cmd_set_color),
//    NRF_CLI_CMD(fans,   &m_sub_fans, "Set Fan on/off", cmd_set_fans),
//    NRF_CLI_CMD(heater,  &m_sub_heater, "Set heater on/off", cmd_set_heater),
//    NRF_CLI_SUBCMD_SET_END
//};
//
//NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_show)
//{
//    NRF_CLI_CMD(status, NULL, "Show status", cmd_show_status)
//    NRF_CLI_SUBCMD_SET_END
//};
//
//NRF_CLI_CMD_REGISTER(set, &m_sub_set, "set", cmd_set);
//
//NRF_CLI_CMD_REGISTER(show, &m_sub_show, "show", cmd_show);
//
//NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_counter)
//{
//    NRF_CLI_CMD(reset,  NULL, "Reset seconds counter.",  cmd_counter_reset),
//    NRF_CLI_CMD(start,  NULL, "Start seconds counter.",  cmd_counter_start),
//    NRF_CLI_CMD(stop,   NULL, "Stop seconds counter.",   cmd_counter_stop),
//    NRF_CLI_SUBCMD_SET_END
//};
//NRF_CLI_CMD_REGISTER(counter,
//                     &m_sub_counter,
//                     "Display seconds on terminal screen",
//                     cmd_counter);
//
//

/**
 * @}
 */


///**@brief Function for application main entry.
// */
//int main(void)
//{
//    uint32_t err_code;
//    bool erase_bonds;
//
//    // Initialize.
//    gpio_init();
//    log_init();
//    //serial_init();
//    timers_init();
//    power_management_init();
//    //buttons_leds_init(&erase_bonds);
//    ble_stack_init();
//    peer_manager_init();
//    gap_params_init();
//    gatt_init();
//    services_init();
//    advertising_init();
//
//    conn_params_init();
//    //buttons_init;
//
//    NRF_LOG_INFO("Buttonless DFU Application started.");
//
//    // Start execution.
//    application_timers_start();
//    advertising_start(erase_bonds);
//
//    // Enter main loop.
//    for (;;)
//    {
//        idle_state_handle();
//    }
//}

/**
 * @}
 */
