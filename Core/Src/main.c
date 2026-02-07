/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stdarg.h"
#include <inttypes.h>
#include "eeprom.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

//
typedef struct
{
 uint32_t spsmax;
 uint32_t spsmin;
 uint32_t spsps;
 uint32_t jogsteps;      //
 uint32_t ssteps;        // step steps
} stotrage_t;

//
typedef struct
{
 uint16_t lsb;
 uint16_t msb;
} addr32_t;

stotrage_t ee_data;

//macros

// define bitwises for semaphore
#define JOGL        0
#define JOGR        1
#define JOGSTEPL    2
#define JOGSTEPR    3
#define STEPL       4
#define STEPR       5
#define EL          6
#define ER          7
#define EE          8
#define JOGL_DB     9
#define JOGR_DB     10
#define JOGSTEPL_DB 11
#define JOGSTEPR_DB 12
#define STEPL_DB    13
#define STEPR_DB    14
#define EL_DB       15
#define ER_DB       16
#define EE_DB       17

//PIN JOGS
#define PIN_JOGL_RESET  (HAL_GPIO_ReadPin(BUTT_JOGL_GPIO_Port, BUTT_JOGL_Pin) == GPIO_PIN_RESET)
#define PIN_JOGL_SET    (HAL_GPIO_ReadPin(BUTT_JOGL_GPIO_Port, BUTT_JOGL_Pin) == GPIO_PIN_SET)
#define PIN_JOGR_RESET  (HAL_GPIO_ReadPin(BUTT_JOGR_GPIO_Port, BUTT_JOGR_Pin) == GPIO_PIN_RESET)
#define PIN_JOGR_SET    (HAL_GPIO_ReadPin(BUTT_JOGR_GPIO_Port, BUTT_JOGR_Pin) == GPIO_PIN_SET)

//PIN STEPS
#define PIN_STEPL_RESET (HAL_GPIO_ReadPin(BUTT_STEPL_GPIO_Port, BUTT_STEPL_Pin) == GPIO_PIN_RESET)
#define PIN_STEPL_SET   (HAL_GPIO_ReadPin(BUTT_STEPL_GPIO_Port, BUTT_STEPL_Pin) == GPIO_PIN_SET)
#define PIN_STEPR_RESET (HAL_GPIO_ReadPin(BUTT_STEPR_GPIO_Port, BUTT_STEPR_Pin) == GPIO_PIN_RESET)
#define PIN_STEPR_SET   (HAL_GPIO_ReadPin(BUTT_STEPR_GPIO_Port, BUTT_STEPR_Pin) == GPIO_PIN_SET)

//PIN E
#define PIN_EL_RESET    (HAL_GPIO_ReadPin(ES_L_GPIO_Port, ES_L_Pin) == GPIO_PIN_RESET)
#define PIN_EL_SET      (HAL_GPIO_ReadPin(ES_L_GPIO_Port, ES_L_Pin) == GPIO_PIN_SET)
#define PIN_ER_RESET    (HAL_GPIO_ReadPin(ES_R_GPIO_Port, ES_R_Pin) == GPIO_PIN_RESET)
#define PIN_ER_SET      (HAL_GPIO_ReadPin(ES_R_GPIO_Port, ES_R_Pin) == GPIO_PIN_SET)


//SEMAPHORES
#define SEM_EL         ((semaphore & (1 << EL)) > 0)
#define SEM_ER         ((semaphore & (1 << ER)) > 0)
#define SEM_JOGL       ((semaphore & (1 << JOGL)) > 0)
#define SEM_JOGR       ((semaphore & (1 << JOGR)) > 0)
#define SEM_JOGSTEPL      ((semaphore & (1 << JOGSTEPL)) > 0)
#define SEM_JOGSTEPR      ((semaphore & (1 << JOGSTEPR)) > 0)

//DEBOUNCES - fixme later. some are not needed
#define DB_JOGL         ((semaphore & (1 << JOGL_DB)) > 0)
#define DB_JOGR         ((semaphore & (1 << JOGR_DB)) > 0)
#define DB_STEPL        ((semaphore & (1 << STEPL_DB)) > 0)
#define DB_STEPR        ((semaphore & (1 << STEPR_DB)) > 0)
#define DB_EL           ((semaphore & (1 << EL_DB)) > 0)
#define DB_ER           ((semaphore & (1 << ER_DB)) > 0)
#define DB_EE           ((semaphore & (1 << EE_DB)) > 0)

// end
#define pulsedur        50                  //puse duration in us

// #define steps4acc       100                //steps for acceleration
#define steps4decc      100                //steps for decceleration

#define accmult         1                   //acceleration multiplier
#define deccmult        10                  //deceleration multiplier
#define maxvelocity     160                 //minumum period for pulse (Vmax)
#define jogspeed        2400                //actual delay for the pause after 50us
#define stepspeed       1200                //actual delay for the pause after 50us
#define delayafterdir   100                 //delay between dir set and start of pulses

// float test
float t = 1.23;

//
uint32_t pulse          = 0;                //global used below
//
uint8_t cmdindex = 0;
uint8_t *cmd = &UserRxBufferFS[0]+4;        //

int tmpret;                                 //
uint32_t debugonly = 0;

uint8_t incdcprintf = 0;                    //

CDCReceiveCharTypes rcs = RX_NOTCPLT;       //
echoTypes rcs2 = RX_ECHO_OFF;               //
inputTypes it = RX_NONE;                    //

uint16_t Status;                            // used in hal_unlock
uint16_t delay;                             //fixme. global for speed adjust during jog

uint32_t semaphore = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

//DECLARES
void Tim1Start();
void Tim1Stop();
void Tim1Callback();
void Tim3Start();
void Tim3Stop();
//
void readVariables();

//
void User_TIMPeriodElapsedCallback();

//
void printSemaphore()
{
 cdcprintf("SEM:%08x: %x\r\n", debugonly++, semaphore);
}

//
uint8_t cdcprintf(const char *format, ... )
{
    uint8_t result = USBD_FAIL;

    if (incdcprintf == 1) {
        return result;
    }

    incdcprintf = 1;

    va_list ap;

    uint8_t buffx[128] = "NOT SET!";

    va_start(ap, format);
        result = vsprintf(buffx, format, ap);
    va_end(ap);
    uint8_t len = strlen((const char*)buffx);
    //here smooker USBD_OK, BUSY, FAIL
    //or usbd_cdc_datain ???

    while (result != USBD_OK) {
        result = CDC_Transmit_FS(buffx, (uint16_t)len);
    }
    incdcprintf = 0;
    return result; //
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//reciprocal
uint32_t reci(uint16_t x) {
    if (x == 0) return 0xFFFFFFFF; // div by 0
    // if (x == 1) return 0xFFFFFFFF; // div by 0
    return 1000000 / x;
}

// 1MHz = 1us resolution. 3us lag here
void delay_us(uint16_t us) {

    if (us == 0) return;
    TIM2->CNT = 0;
    TIM2->ARR = 65535;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    HAL_StatusTypeDef stat;
    // HAL_TIM_Base_Stop(&htim2);
    stat = HAL_TIM_Base_Start(&htim2);
    if (stat != 0) {
        cdcprintf("TE SEGA SI EBA MAMATA:%x\t%x\t%x\r\n", stat, TIM2->CNT, TIM2->ARR);
        BKPT;
    }
    while (TIM2->CNT < us);
    HAL_TIM_Base_Stop(&htim2);
}

//
int goJogStep(uint8_t dir, uint32_t steps, int speed)  //speed in hz           //FIXME
{
    //SET DIRECTION
    if (dir) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    }
    delay_us(1300);                 //delay after DIRECTION setting

//     // for (uint32_t pulse = 1; pulse <= steps; pulse++) {
//     while ( (pulse <= steps4acc) & SEM_JOGSTEPL & !SEM_EL ) {
//         //
//         HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
//         delay_us(pulsedur-3);       // 3us lag
//         HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);
//         delay_us(pulsedur-3);       //3us lag

//         uint16_t delay;

//         //start ramp.
//         if (pulse < steps4acc) {
//             delay = (steps4acc - pulse) * accmult + stepspeed;
//             delay_us( delay );
//         }
//         pulse++;
//     }

//     return 0;

//     //max velocity limiter
//     if ( (pulse <= (steps-steps4decc)) & (pulse >= steps4acc) ) {
//         delay_us( delay );
//     }

//     //stop ramp.
//     if (pulse > (steps-steps4decc)) {
//         delay = (pulse-steps+steps4decc) * deccmult + stepspeed;
//         delay_us( delay ); //da se prepravi smetkata s maxvelocity
//     }

     return 0;
 }

//
int JogRampUp(uint8_t dir, uint32_t steps_in)
{
    cdcprintf("in JogRampUp dir=%d\r\n", dir);

     // 0 = L, 1 = R GPIO_PIN_SET for L
    if (dir == 0) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    }
    if (dir == 1) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    }

    delay_us(delayafterdir);       //lag minimum - to be variable from V min

     uint32_t tmpsps = ee_data.spsmin * 100;
     int dsps = ee_data.spsmax-ee_data.spsmin;            //

     int time = dsps * 1000 / ee_data.spsps;            // 80000/50 = 1600ms

     delay_us(50);

     cdcprintf("into JogRampUp on steps=%d, timeacc=%dms\r\n", dsps, time);

     HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);

     while( ( tmpsps < ee_data.spsmax*100 )
            & (steps_in > 0 ) )
     {
        if ( SEM_EL & (dir == 0) ) {
            break;
        }
        if ( SEM_ER & (dir == 1) ) {
            break;
        }
        int delay_in_us = reci(tmpsps/100);
        // cdcprintf("will delay on %d - %u us\r\n", tmpsps/100, delay_in_us);
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);
        delay_us(50-3);       // 3us lag
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
        delay_us(delay_in_us-3);
        tmpsps+=ee_data.spsps;
        steps_in--;
     }

    cdcprintf("out JogRampUp on pulse=%d\r\n", steps_in);
    return 0;
}

//
int LJogRampDown()
{
    cdcprintf("in LJogRampDown\r\n");

    //
    pulse = 1;
    while ( (pulse <= steps4decc) & !SEM_EL ) {
        //
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
        delay_us(pulsedur-3);       // 3us lag
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);
        delay_us(pulsedur-3);       //3us lag

        //
        delay = pulse * deccmult + jogspeed;
        delay_us( delay ); //da se prepravi smetkata s maxvelocity
        //
        pulse++;
    }
    cdcprintf("out LJogRampDown on pulse=%d\r\n", pulse-1);
    return 0;
}

//
int LJog()
{
    cdcprintf("in LJog\r\n");

    //
    pulse = 1;
    while ( SEM_JOGL & !SEM_EL ) {
        //
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
        delay_us(pulsedur-3);       // 3us lag
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);
        delay_us(pulsedur-3);       //3us lag

        //
        delay = jogspeed;
        delay_us( delay ); //da se prepravi smetkata s maxvelocity
        //
        pulse++;
    }
    cdcprintf("out LJog on pulse=%d\r\n", pulse-1);
    return 0;
}

//receiver
void CDCReceiveChar(uint8_t* inchar)
{
    //long commands - overflow
    if (cmdindex >= 15) {
        cmdindex = 0;
        rcs = RX_OF;
        return;
    }
    //end of command - cr
    if (*inchar == 13) {
        cmdindex = 0;
        rcs = RX_CR;
        return;
    }

    //end of command - lf
    if (*inchar == 10) {
        cmdindex = 0;
        rcs = RX_CR;
        return;
    }

    //echo typing
    if (rcs2 == RX_ECHO_ON) {
        cdcprintf("%s", inchar);
    }

    cmd[cmdindex++] = *inchar;
    // UserRxBufferFS[cmdindex] = *inchar;
}

//
void dumpIO()
{
    //GPIO_PinState RESET = 0, 1 SET
    // HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)

    cdcprintf("----------------------\r\n");
    cdcprintf("Dump/set of IO\r\n");
    cdcprintf("----------------------\r\n");
    cdcprintf("JOGL (PA6)  : %d\r\n", HAL_GPIO_ReadPin(BUTT_JOGL_GPIO_Port, BUTT_JOGL_Pin));
    cdcprintf("JOGR (PA7)  : %d\r\n", HAL_GPIO_ReadPin(BUTT_JOGR_GPIO_Port, BUTT_JOGR_Pin));
    cdcprintf("STEPL (PB0) : %d\r\n", HAL_GPIO_ReadPin(BUTT_STEPL_GPIO_Port, BUTT_STEPL_Pin));
    cdcprintf("STEPR (PB1) : %d\r\n", HAL_GPIO_ReadPin(BUTT_STEPR_GPIO_Port, BUTT_STEPR_Pin));
    cdcprintf("ESL  (PB2)  : %d\r\n", HAL_GPIO_ReadPin(ES_L_GPIO_Port, ES_L_Pin));
    cdcprintf("ESR  (PB10) : %d\r\n", HAL_GPIO_ReadPin(ES_R_GPIO_Port, ES_R_Pin));
    cdcprintf("----------------------\r\n");
}

//
void dumpVars()
{
    readVariables();
    cdcprintf("----------------------\r\n");
    cdcprintf("Dump of NVARS in EEPROM\r\n");
    cdcprintf("----------------------\r\n");
    cdcprintf("spsmax       : %d\r\n", ee_data.spsmax);
    cdcprintf("spsmin       : %d\r\n", ee_data.spsmin);
    cdcprintf("spsps        : %d\r\n", ee_data.spsps);
    cdcprintf("jogsteps     : %d\r\n", ee_data.jogsteps);
    cdcprintf("ssteps       : %d\r\n", ee_data.ssteps);
    cdcprintf("----------------------\r\n");
    cdcprintf("RECI:  %d\r\n", reci(3));
    cdcprintf("----------------------\r\n");
}

//
void help()
{
    cdcprintf("\r\n----------------------\r\n");
    cdcprintf("HELP with commands\r\n");
    cdcprintf("----------------------\r\n");
    cdcprintf("reset    : resets the system\r\n");
    cdcprintf("a        : input spsmax\r\n");
    cdcprintf("b        : input spsmin\r\n");
    cdcprintf("c        : input spsps\r\n");
    cdcprintf("d        : input jogsteps\r\n");
    cdcprintf("e        : input step steps\r\n");
    cdcprintf("help     : this help\r\n");
    cdcprintf("dump     : dump variables\r\n");
    cdcprintf("dumpio   : dump IO states\r\n");
    cdcprintf("----------------------\r\n");
    cdcprintf("STATUSES/RESULTS/MEANINGS\r\n");
    cdcprintf("----------------------\r\n");
    cdcprintf("OVERFLOW : input buffer full. input discarded\r\n");
    cdcprintf("POT      : power up times counter\r\n");
    cdcprintf("----------------------\r\n");
}

//
uint16_t eewrite32(uint16_t VirtAddress, uint32_t Data) {
    uint32_t addr32 = VirtAddress*2;
    uint16_t status = FLASH_COMPLETE;

    uint16_t addrlsb =  addr32 & 0x0000ffff;
    uint16_t addrmsb =  (addr32 & 0xffff0000) >> 16;
    cdcprintf("addrlsb: 0x%x\r\n", addrlsb);
    cdcprintf("addrmsb: 0x%x\r\n", addrmsb);

    uint16_t datalsb =  Data & 0x0000ffff;
    uint16_t datamsb =  (Data & 0xffff0000) >> 16;
    cdcprintf("datalsb: 0x%x\r\n", datalsb);
    cdcprintf("datamsb: 0x%x\r\n", datamsb);

    if (status = EE_WriteVariable(addrlsb, datalsb) != FLASH_COMPLETE ) {
        cdcprintf("error during writing lsb: 0x%x\r\n", status);
        return status;
    }
    if (status = EE_WriteVariable(addrlsb+1, datamsb) != FLASH_COMPLETE ) {
        cdcprintf("error during writing msb: 0x%x\r\n", status);
        return status;
    }
    return status;      //not needed but...
}

//
uint16_t eeread32(uint16_t VirtAddress, uint32_t* Data) {   //smooker fixme. has to be uint32_t pointer ?!? will not work on bigger than 64kb ram size

    uint32_t addr32 = VirtAddress*2;
    uint16_t status = 0x99;     // 0-OK, 1-doesnotexist,0xab-no valid page, 0x99-mine ???

    addr32_t val;      //storage struct for pointer to the uint32_t variable

    uint16_t addrlsb =  addr32 & 0x0000ffff;
    uint16_t addrmsb =  (addr32 & 0xffff0000) >> 16;
    // cdcprintf("addrlsb: 0x%x\r\n", addrlsb);
    // cdcprintf("addrmsb: 0x%x\r\n", addrmsb);
    // uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data)

    if (status = EE_ReadVariable(addrlsb, &val.lsb) != 0 ) {
        cdcprintf("error during reading lsb: 0x%x\r\n", status);
        return status;
    }
    if (status = EE_ReadVariable(addrlsb+1, &val.msb) != 0 ) {
        cdcprintf("error during reading msb: 0x%x\r\n", status);
        return status;
    }

    *Data = (val.msb << 16) + (val.lsb);

    return status;      //not needed but...
}

//
void readVariables()
{
    uint16_t stat;
    if ( (stat = eeread32(1, &ee_data.spsmax)) != 0) {
        cdcprintf("spsmax read returned 0x%x\r\n", stat);
    }

    if ( (stat = eeread32(2, &ee_data.spsmin)) != 0) {
        cdcprintf("spsmin read returned 0x%x\r\n", stat);
    }

    if ( (stat = eeread32(3, &ee_data.spsps)) != 0) {
        cdcprintf("spsps read returned 0x%x\r\n", stat);
    }

    if ( (stat = eeread32(4, &ee_data.jogsteps)) != 0) {
        cdcprintf("jogsteps read returned 0x%x\r\n", stat);
    }

    if ( (stat = eeread32(5, &ee_data.ssteps)) != 0) {
        cdcprintf("ssteps read returned 0x%x\r\n", stat);
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

    semaphore = 0;

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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // HAL_TIM_Base_Start(&htim2);
  HAL_Delay(1000);      //wait for USB reenumeration

  //boot pritnf
  memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS));

  //
  cdcprintf("Malinovski 12.2025 (c) smooker&chichko %d \r\n");

  //
  HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, User_TIMPeriodElapsedCallback);
  HAL_TIM_RegisterCallback(&htim1, HAL_TIM_PERIOD_ELAPSED_CB_ID, Tim1Callback);
  Tim1Start();

  //
  Status = HAL_FLASH_Unlock();      //fixme. move the same into ee_init and into ww_writevariable
  assert_param(Status == HAL_OK);
  if(Status != HAL_OK) {
    cdcprintf("HAL_FLASH_Unlock() returned %d\r\n", Status);
  };

  cdcprintf("EE_Init returned %d\r\n", EE_Init());

  // cdcprintf("HAL_GetUIDw0()=%lu\r\n", HAL_GetUIDw0());
  // cdcprintf("HAL_GetUIDw1()=%lu\r\n", HAL_GetUIDw1());
  // cdcprintf("HAL_GetUIDw2()=%lu\r\n", HAL_GetUIDw2());

  cdcprintf("HAL_GetDEVID=%lu\r\n", HAL_GetDEVID());
  //For medium-density devices, the device ID is 0x410

  cdcprintf("Flash size=%ukiB\r\n", *(const uint16_t*)FLASHSIZE_BASE);

  dumpVars();
  dumpIO();


  // states of the outputs
  HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //migalka za watchdog/main thread
    // HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
    // HAL_Delay(200);      //ms
    // HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_SET);
    // HAL_Delay(200);      //ms

    //MOVEMENT COMMANDS

    // printSemaphore();
    //JOGL - buttons to GND, limit switches too

    if ( SEM_JOGL ) {
            cdcprintf("JOGL\r\n");
            printSemaphore();
            // LJogRampUp();
            // LJog();
            // LJogRampDown();
    }

    //JOGR - buttons to GND, limit switches too
    if ( SEM_JOGR ) {
            cdcprintf("JOGR\r\n");
            printSemaphore();
            // RJogRampUp();
            // RJog();
            // RJogRampDown();
    }

    if ( semaphore & ( 1 << JOGSTEPL) ) {
        cdcprintf("JOGSTEPL\r\n");
        // goJogStep(0, ee_data.jogsteps, 0);
        JogRampUp(0, ee_data.jogsteps);
        semaphore &= ~( 1 << JOGSTEPL);
        cdcprintf("JOGSTEPL END\r\n");
    }

    if ( semaphore & ( 1 << JOGSTEPR) ) {
        cdcprintf("JOGSTEPR\r\n");
        JogRampUp(1, ee_data.jogsteps);
        semaphore &= ~( 1 << JOGSTEPR);
        cdcprintf("JOGSTEPR END\r\n");
    }

     //END OF MOVEMENTS

    if (rcs == RX_CR) {
        // we were in variable entry mode
        if (rcs2 == RX_ECHO_ON) {
            rcs2 = RX_ECHO_OFF;
            // parse variable values
            if (it == RX_VAR1) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.spsmax);
                cdcprintf(" spsmax value : %d\r\n", ee_data.spsmax);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(1, ee_data.spsmax);
                eeread32(1, &ee_data.spsmax);
                cdcprintf(" spsmax value : %d\r\n", ee_data.spsmax);
            }
            else if (it == RX_VAR2) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.spsmin);
                cdcprintf(" spsmin value : %d\r\n", ee_data.spsmin);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(2, ee_data.spsmin);
                eeread32(2, &ee_data.spsmin);
                cdcprintf(" spsmin value : %d\r\n", ee_data.spsmin);
            }
            else if (it == RX_VAR3) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.spsps);
                cdcprintf(" spsps value : %d\r\n", ee_data.spsps);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(3, ee_data.spsps);
                eeread32(3, &ee_data.spsps);
                cdcprintf(" spsps value : %d\r\n", ee_data.spsps);
            }
            else if (it == RX_VAR4) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.jogsteps);
                cdcprintf(" jogsteps value : %d\r\n", ee_data.jogsteps);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(4, ee_data.jogsteps);
                eeread32(4, &ee_data.jogsteps);
                cdcprintf(" jogsteps value : %d\r\n", ee_data.jogsteps);
            }
            else if (it == RX_VAR5) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.ssteps);
                cdcprintf(" ssteps value : %d\r\n", ee_data.ssteps);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(5, ee_data.ssteps);
                eeread32(5, &ee_data.ssteps);
                cdcprintf(" ssteps value : %d\r\n", ee_data.ssteps);
            }
            else {
                cdcprintf(" WHAT THE FUCK ?!\r\n", cmd);
            }
        }
        // parse commands
        else if (strcmp(cmd, "reset") == 0) {
            cdcprintf("RES:reset\r\n");
            NVIC_SystemReset();
        } else if (strcmp(cmd, "dump") == 0) {
            cdcprintf("RES:dump %s\r\n", cmd);
            dumpVars();
        } else if (strcmp(cmd, "dumpio") == 0) {
            cdcprintf("RES:dumpio %s\r\n", cmd);
            dumpIO();
        } else if (strcmp(cmd, "help") == 0) {
            help();
        } else if (strcmp(cmd,"a") == 0) {
            cdcprintf("enter value for spsmax:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR1;
        } else if (strcmp(cmd,"b") == 0) {
            cdcprintf("enter value for spsmin:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR2;
        } else if (strcmp(cmd,"c") == 0) {
            cdcprintf("enter value spsps:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR3;
        } else if (strcmp(cmd,"d") == 0) {
            cdcprintf("enter value jogsteps:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR4;
        } else if (strcmp(cmd,"e") == 0) {
            cdcprintf("enter value ssteps:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR5;
        } else if (strcmp(cmd,"g") == 0) {
            cdcprintf("go running!\r\n");
            goJogStep(0, ee_data.jogsteps, 100);
            delay_us(500);
            cdcprintf("go stopped!\r\n");
        }
        else {
            cdcprintf("RES: UNKNOWN COMMAND: %s\r\n", cmd);
        }
        memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS));
    } else if (rcs == RX_OF) {
        cdcprintf("\r\nRES: OVERFLOW.\r\n");    //keep it smaller
        rcs2 = RX_ECHO_OFF;
        it = RX_NONE;
        memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS));
    }

    rcs = RX_NOTCPLT;      //reset status

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
    //1uS perdiod 48Mhz sysclk/48=1Mhz
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HX711_CLK_GPIO_Port, HX711_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PULSE_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_USER_Pin */
  GPIO_InitStruct.Pin = LED_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTT_JOGL_Pin BUTT_JOGR_Pin */
  GPIO_InitStruct.Pin = BUTT_JOGL_Pin|BUTT_JOGR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTT_STEPL_Pin BUTT_STEPR_Pin */
  GPIO_InitStruct.Pin = BUTT_STEPL_Pin|BUTT_STEPR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ES_L_Pin ES_R_Pin */
  GPIO_InitStruct.Pin = ES_L_Pin|ES_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HX711_DATA_Pin */
  GPIO_InitStruct.Pin = HX711_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HX711_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HX711_CLK_Pin */
  GPIO_InitStruct.Pin = HX711_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HX711_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PULSE_Pin DIR_Pin */
  GPIO_InitStruct.Pin = PULSE_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  cdcprintf("EXTI %04x\r\n", GPIO_Pin);

  //EL pin handling
  if( PIN_EL_RESET & (GPIO_Pin == GPIO_PIN_10) ) {
    cdcprintf("EL0\r\n");
    semaphore |= (1 << EL);
    Tim1Stop();
    Tim3Stop();
  }
  if( PIN_EL_SET & (GPIO_Pin == GPIO_PIN_10) ) {
    cdcprintf("EL1\r\n");
    semaphore &= ~(1 << EL);
  }

  //ER pin handling
  if( PIN_ER_RESET & (GPIO_Pin == GPIO_PIN_11) ) {
    cdcprintf("ER0\r\n");
    semaphore |= (1 << ER);
    Tim1Stop();
    Tim3Stop();
  }
  if( PIN_ER_SET & (GPIO_Pin == GPIO_PIN_11) ) {
    cdcprintf("ER1\r\n");
    semaphore &= ~(1 << ER);
  }

  //EL forbids L MOVEMENT
  if ( SEM_EL ) {
    cdcprintf("SEM_EL\r\n");
    semaphore &= ~((1 << JOGSTEPL) | (1 << JOGL) | (1 << STEPL));
  }
  //ER forbids R MOVEMENT
  if ( SEM_ER ) {
    cdcprintf("SEM_ER\r\n");
    semaphore &= ~((1 << JOGSTEPR) | (1 << JOGR) | (1 << STEPR));
  }

  // if we are in debouncing skip other events
  if ( (semaphore & ((1 << JOGL_DB) |  (1 << JOGR_DB)) ) ) {
    return;
  }

  // allow only LEFT commands
  if ( !SEM_EL ) {
      // JOG LEFT button pressed
      if( PIN_JOGL_RESET & (GPIO_Pin == GPIO_PIN_6) ) {
        semaphore |= (1 << JOGL_DB);
        cdcprintf("JL0\r\n");
        Tim1Start();
        Tim3Start();
      }

      // JOG LEFT button released
      if( PIN_JOGL_SET & (GPIO_Pin == GPIO_PIN_6) ) {
        cdcprintf("JL1\r\n");
        semaphore &= ~( (1 << JOGL) | (1 << JOGSTEPL) | (1 << JOGL_DB) );
        Tim1Stop();
      }
  }

  // allow only RIGHT commands
  if ( !SEM_ER ) {
      // JOG RIGHT button pressed
      if( PIN_JOGR_RESET & (GPIO_Pin == GPIO_PIN_7) ) {
        semaphore |= (1 << JOGR_DB);
        cdcprintf("JR0\r\n");
        Tim1Start();
        Tim3Start();
      }

      // JOG RIGHT button released
      if( PIN_JOGR_SET & (GPIO_Pin == GPIO_PIN_7) ) {
        cdcprintf("JR1\r\n");
        semaphore &= ~( (1 << JOGR) | (1 << JOGSTEPR) | (1 << JOGR_DB) );
        Tim1Stop();
      }
  }
}

//
void Tim3Start() {
  //
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  htim3.Instance->ARR=20; //20ms
  __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim3);
}
//
void Tim3Stop() {
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  htim3.Instance->ARR=0;
  __HAL_TIM_DISABLE_IT(&htim3,TIM_IT_UPDATE);
  __HAL_TIM_DISABLE(&htim3);
}

//
void Tim1Start() {
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    htim1.Instance->ARR=1000;    //1000ms
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim1);
}
//
void Tim1Stop() {
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    htim1.Instance->ARR=0;
    __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim1);
}

void Tim1Callback()
{
  HAL_GPIO_TogglePin(LED_USER_GPIO_Port, LED_USER_Pin);
  cdcprintf("TIM1 fired:0x%08x\r\n", debugonly++);
  HAL_GPIO_TogglePin(LED_USER_GPIO_Port, LED_USER_Pin);
  if ( !DB_JOGL & PIN_JOGL_RESET ) {
    cdcprintf("JLT10\r\n");
    semaphore |= (1 << JOGL);
  }
  if ( !DB_JOGR & PIN_JOGR_RESET ) {
    cdcprintf("JRT10\r\n");          //da pravim step 1mm
    semaphore |= (1 << JOGR);       //
  }

  Tim1Stop();
}

// buttons  release handling
void User_TIMPeriodElapsedCallback()
{
  //JOG LEFT
  if ( semaphore & (1 << JOGL_DB) ) {            // v process na debounce-vame se
    if ( PIN_JOGL_RESET ) {
        cdcprintf("JLT0\r\n");          //da pravim step 1mm
        semaphore |= (1 << JOGSTEPL);
    }
    if ( PIN_JOGL_SET ) {
        cdcprintf("JLT1\r\n");
        semaphore &= ~((1 << JOGL) | (1 << JOGSTEPL));      //release JOGs flags
    }
    semaphore &= ~(1 << JOGL_DB);                // svaliame debouncinga
  }

  //JOG RIGHT
  if ( semaphore & (1 << JOGR_DB) ) {            // v process na debounce-vame se
    if ( PIN_JOGR_RESET ) {
        cdcprintf("JRT0\r\n");          //da pravim step 1mm
        semaphore |= (1 << JOGSTEPR);
    }
    if ( PIN_JOGR_SET ) {
        cdcprintf("JRT1\r\n");
        semaphore &= ~((1 << JOGR) | (1 << JOGSTEPR));      //release JOGs flags
    }
    semaphore &= ~(1 << JOGR_DB);                // svaliame debouncinga
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

#ifdef  USE_FULL_ASSERT
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
