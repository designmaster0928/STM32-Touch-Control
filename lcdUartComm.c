/*
 * lcdUartComm.c
 *
 *  Created on: May 22, 2022
 *      Author: Administrator
 */

#include "lcdUartComm.h"
#include "main.h"
#include "sgtl5000.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

#define LCDUARTBUFFERSIZE		128

static int lcdUartS, lcdUartE;
static uint8_t lcdUartRBuffer[LCDUARTBUFFERSIZE];
static uint8_t lcdUartWBuffer[LCDUARTBUFFERSIZE];
static uint8_t lcdCrcTempBuf[LCDUARTBUFFERSIZE];
static uint8_t lcdRxBuffer[1];

//Communication Constants -- 3 byte arrays sent from mobile application over NUS
//Byte 1 - Command - "this is a command"
#define COMMAND     0x2
#define MAX_VOLUME	12

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

bool playingTrack = false;

void StartLcdUartReceiveProc(void)
{
  if (HAL_UART_Receive_IT(&huart1, lcdRxBuffer, 1) != HAL_OK)
    Error_Handler();
}

void LcdUartReceiveCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		lcdUartRBuffer[lcdUartS] = lcdRxBuffer[0];
		lcdUartS++;
		if (lcdUartS >= LCDUARTBUFFERSIZE)
			lcdUartS -= LCDUARTBUFFERSIZE;

		HAL_UART_Receive_IT(&huart1, lcdRxBuffer, 1);
	}
}

static int GetRBufferSize(void)
{
	return (lcdUartS + LCDUARTBUFFERSIZE - lcdUartE) % LCDUARTBUFFERSIZE;
}

static uint8_t GetRBuffer(int idx)
{
	return lcdUartRBuffer[(lcdUartE + idx) % LCDUARTBUFFERSIZE];
}

static void SetRBufferIndex(int size)
{
	lcdUartE += size;
	if (lcdUartE >= LCDUARTBUFFERSIZE)
		lcdUartE -= LCDUARTBUFFERSIZE;
}

static void SendDataToUart(uint8_t * pData, uint16_t len)
{
	HAL_UART_Transmit(&huart1, pData, len, 300);
}


uint32_t lastVolumeTime = 0;
uint8_t lastVolumeSetFlag = 0;

static void SetAudioVolume(void)
{
//	printf("Set volume %d \r\n", select);
	sgtl5000_stop_play();
	printf("Audio Driver Stopped for Volume \r\n");
	sgtl5000_set_volume(systemVolume);
	sgtl5000_start_play();
	printf("Audio Driver Started for Volume \r\n");
}

void EsimationLcdUartComm(void)
{
	int i;
	uint8_t reg_addr;
	uint8_t reg_count;
	uint16_t reg_data;
	uint8_t checkCode;
	static uint8_t endCheck = 0;

	if (GetRBufferSize() >= 3)
	{
		char command = GetRBuffer(0);
		char status = GetRBuffer(1);
		char select = GetRBuffer(2);

		printf("Received command is %c \r\n", command);
		printf("Received status is %c \r\n", status);
		printf("Received select is %c \r\n", select);

		if (/*command == COMMAND || */ command == 'c')
		{
			if(status == 'f'){
				if (select == '1') endCheck = endCheck | 0x01;// 0b00000001	// FAN OFF
				else endCheck = endCheck & 0x06; // 0b00000110				// FAN ON
			}
			else if(status == 's') {
				if(select == 'w') endCheck = endCheck | 0x02;// 0b00000010	// LED Light ON
				else endCheck = endCheck & 0x05; // 0b00000101				// LED Light OFF
			}
			else if(status == 't') {
				if(select == 's') endCheck = endCheck & 0x03; // 0b00000011	// Stop Playing Audio
				else endCheck = endCheck | 0x04;// 0b00000100				// Start Playing Audio
			}

			printf("endCheck : %d  \r\n", endCheck);




			if (/*status == BREATHING ||*/ status == 'b')
			{
				if (/*select == TRACK1 ||*/ select == 'l')
				{
					// Focus
					pwm_update_duty_cycle(0,255,255,127,0); // light blue
					printf("Light Focus \r\n");
				}
				if(/*select == TRACK2 ||*/ select == 'b') {
					// Blue
					pwm_update_duty_cycle(0,0,255,0,0);
					printf("Led Blue \r\n");
				}
				if(/*select == TRACK3 ||*/ select == 'p') {
					// Fuschia
					pwm_update_duty_cycle(255,0,255,0,0);
					printf("Light Fuschia \r\n");
				}
				if(/*select == TRACK4 ||*/ select == 'g') {
					// Green
					pwm_update_duty_cycle(0,255,0,0,0);
					printf("Light Green \r\n");
				}
				if(/*select == TRACK5 ||*/ select == 'k') {
					// Low K white
					pwm_update_duty_cycle(0,0,0,0,255);
					printf("Light Low K White \r\n");
				}
			}
			if(/*status == STANDBY ||*/ status == 's')
			{
				if(/*select == TRACK2 ||*/ select == 'w') {
					// White
					pwm_update_duty_cycle(255,255,255,0,0);
					printf("Light White \r\n");
				}
				if(/*select == OFF ||*/ select == 'o') {
					pwm_update_duty_cycle(0,0,0,0,0);
					printf("Light Off \r\n");
				}
				if(select == 'b') {
//					err_code = app_timer_start(burn_in_timer_id, APP_TIMER_TICKS(1000), NULL);
//					APP_ERROR_CHECK(err_code);
					printf("Start App Timer \r\n");
				}
				if(select == 's') {
//					err_code = app_timer_stop(burn_in_timer_id);
//					APP_ERROR_CHECK(err_code);
					printf("Stop App Timer \r\n");
				}
			}
			if(/*status == FAN ||*/ status == 'f')
			{
				if(/*select == FAN_OFF ||*/ select == '1') {
					HAL_GPIO_WritePin(FAN_ON_GPIO_Port, FAN_ON_Pin, GPIO_PIN_RESET);
					printf("Fan Off \r\n");

				}
				else if (/*select == FAN_ON ||*/ select == '2') {
					HAL_GPIO_WritePin(FAN_ON_GPIO_Port, FAN_ON_Pin, GPIO_PIN_SET);
					printf("Fan On \r\n");
				}
			}
			if(/*status == REMOTE ||*/ status == 'r')
			{
				if(select == START || select == 's') {
					HAL_GPIO_WritePin(PLAY_OUT_GPIO_Port, PLAY_OUT_Pin, GPIO_PIN_SET);
					osDelay(50);
					HAL_GPIO_WritePin(PLAY_OUT_GPIO_Port, PLAY_OUT_Pin, GPIO_PIN_RESET);
					printf("Remote Start pressed \r\n");

				}
			}
			if (/*status == VOLUME ||*/ status == 'v')
			{
				if (select > 0 && select < 64) {
					systemVolume = (((127 - MAX_VOLUME) * select) >> 6) + MAX_VOLUME;
					printf("Set volume %d \r\n", select);
					sgtl5000_stop_play();
					printf("Audio Driver Stopped for Volume \r\n");
					sgtl5000_set_volume(select);
					sgtl5000_start_play();
					printf("Audio Driver Started for Volume \r\n");
				}
			}
			if ((/*status == PLAY ||*/ status == 't'))
			{
				if (select == 's') {
					if (playingTrack) {
						printf("Audio Driver Stopped on Track Change \r\n");
						sgtl5000_stop_play();
						SetPlayState(PLAY_NONE);
						playingTrack = false;
					}
				}
				if (/*select == INTRO ||*/ select == 'k') {
					printf("Playing Intro \r\n");
					StartPlayAudioFile("_Intro.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (/*select == TEST ||*/ select == 'l')
				{
					printf("Playing Test Bell \r\n");
					StartPlayAudioFile("_TestBell.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == '1') {
					printf("Playing Focus \r\n");
					StartPlayAudioFile("Focus.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if ( select == '2') {
					printf("Playing Motivate \r\n");
					StartPlayAudioFile("Motivate.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if ( select == '3') {
					printf("Playing Fit \r\n");
					StartPlayAudioFile("Fit.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if ( select == '4') {
					printf("Playing Perform \r\n");
					StartPlayAudioFile("Perform.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == '5') {
					printf("Playing Heal \r\n");
					StartPlayAudioFile("Heal.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if ( select == '6') {
					printf("Playing Recharge \r\n");
					StartPlayAudioFile("Recharge.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == '7') {
					printf("Playing Relax \r\n");
					StartPlayAudioFile("Relax.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == '8') {
					printf("Playing Snooze \r\n");
					StartPlayAudioFile("Snooze.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == '9') {
					printf("Playing Create \r\n");
					StartPlayAudioFile("Creativity.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'a') {
					printf("Playing Overcome \r\n");
					StartPlayAudioFile("Overcome.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'b') {
					printf("Playing Succeed \r\n");
					StartPlayAudioFile("Succeed.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'c') {
					printf("Playing Ascend \r\n");
					StartPlayAudioFile("Ascend.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'd') {
					printf("Playing Aspire \r\n");
					StartPlayAudioFile("Aspire.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'e') {
					printf("Playing Bliss \r\n");
					StartPlayAudioFile("Bliss.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'f') {
					printf("Playing Clarity \r\n");
					StartPlayAudioFile("Clarity.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'g') {
					printf("Playing Confidence \r\n");
					StartPlayAudioFile("Confidence.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'h') {
					printf("Playing Love \r\n");
					StartPlayAudioFile("Love.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == 'i') {
					printf("Playing Manifest \r\n");
					StartPlayAudioFile("Manifest.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if ( select == 'j') {
					printf("Playing Prosper \r\n");
					StartPlayAudioFile("Prosperity.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
				if (select == '0') {
					printf("Playing Reclaim \r\n");
					StartPlayAudioFile("Reclaim.mp3");
					playingTrack = true;
					lastVolumeTime = HAL_GetTick();
					lastVolumeSetFlag = 1;
				}
			} // end PLAY status
			SetRBufferIndex(3);
		} // end command
		else
		{
			printf("Received command is %c \r\n", 'a');
			SetRBufferIndex(1);
		}
	}

	if (lastVolumeSetFlag == 1)
	{
		if (HAL_GetTick() - lastVolumeTime > 200) {
			SetAudioVolume();
			lastVolumeSetFlag = 0;
		}
	}
}

