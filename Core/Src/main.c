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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

//
typedef struct
{
 uint32_t Vmax;
 uint32_t Vmin;
 uint32_t dVdt;
 uint32_t steps4unit;
 uint32_t pot;         //power up times
} stotrage_t;

//
typedef struct
{
 uint16_t lsb;
 uint16_t msb;
} addr32_t;

stotrage_t ee_data;

#define pulsedur    50                  //puse duration in us
#define steps4acc   100                 //steps for acceleration
#define steps4decc  100                 //steps for decceleration
#define accmult     2                   //acceleration multiplier
#define deccmult    10                  //deceleration multiplier
#define maxvelocity 160                 //minumum period for pulse (Vmax)

uint16_t totinpulse = 0;                //total time in pulse (period). to be used later.. maybe used in delay_us
uint8_t cmdindex = 0;
uint8_t *cmd = &UserRxBufferFS[0]+4;    //

int tmpret;                             //
uint32_t debugonly = 0;

uint8_t ingo = 0;                       //
uint8_t incdcprintf = 0;                //

CDCReceiveCharTypes rcs = RX_NOTCPLT;   //
echoTypes rcs2 = RX_ECHO_OFF;           //
inputTypes it = RX_NONE;                //

uint16_t Status;                        // used in hal_unlock

// define bitwises for semaphore
#define JOGL 0
#define JOGR 1
#define STEPL 2
#define STEPR 3
#define EL 4
#define ER 5
#define GO 6
uint32_t semaphore = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

//
void readVariables();

//
void User_TIMPeriodElapsedCallback();

//
void printSemaphore()
{
 // cdcprintf("SEM:%08x: %x\r\n", debugonly, semaphore);
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

// 1MHz = 1us resolution. 3us lag here
void delay_us(uint16_t us) {
    if (us == 0) return;
    totinpulse += us;
    TIM2->CNT = 0;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    HAL_TIM_Base_Start(&htim2);
    while (TIM2->CNT < us);
    HAL_TIM_Base_Stop(&htim2);
}

//
int goStep(uint8_t dir, uint32_t steps, int speed)  //speed in hz
{
    if (ingo > 0) {
        return 5;            //we are called twice
    }

    ingo = 1;

    if (dir) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    }
    delay_us(1300);       //lag minimum - to be variable from V min

    for (uint32_t pulse = 1; pulse <= steps; pulse++) {
        if (ingo == 0) {
            break;
        }
        totinpulse = 0;
        //
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
        delay_us(pulsedur-3);       // 3us lag
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);
        delay_us(pulsedur-3);       //3us lag

        uint16_t delay;

        //start ramp.
        if (pulse < steps4acc) {
            delay = (steps4acc - pulse) * accmult + 1200;
            delay_us( delay );   //da se prepravi smetkata s maxvelocity
            //triabva da razgynem ravnomerno ot 50ms do 1ms = d 49ms ama za kolko vreme... ?
        }

        //max velocity limiter
        if ( (pulse <= (steps-steps4decc)) & (pulse >= steps4acc) ) {
            delay_us( delay );            //1000us dopylvane
        }

        //stop ramp.
        if (pulse > (steps-steps4decc)) {
            delay = (pulse-steps+steps4decc) * deccmult + 1200;
            delay_us( delay ); //da se prepravi smetkata s maxvelocity
        }
    }
    ingo = 0;
    return 0;
}

int goJog(uint8_t dir, uint32_t steps, int speed)
{
    if (ingo > 0) {
        return 5;            //we are called twice
    }

    ingo = 1;

    if (dir) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    }
    delay_us(1300);       //lag minimum - to be variable from V min

    for (uint32_t pulse = 1; pulse <= steps; pulse++) {
        if (ingo == 0) {
            break;
        }
        totinpulse = 0;
        //
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
        delay_us(pulsedur-3);       // 3us lag
        HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);
        delay_us(pulsedur-3);       //3us lag

        uint16_t delay;

        //start ramp.
        if (pulse < steps4acc) {
            delay = (steps4acc - pulse) * accmult + 1200;
            delay_us( delay );   //da se prepravi smetkata s maxvelocity
            //triabva da razgynem ravnomerno ot 50ms do 1ms = d 49ms ama za kolko vreme... ?
        }

        //max velocity limiter
        if ( (pulse <= (steps-steps4decc)) & (pulse >= steps4acc) ) {
            delay_us( delay );            //1000us dopylvane
        }
    }
    ingo = 0;
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
    // uint32_t val1;
    // int16_t val2;
    // int8_t val3;
    // float val4;
    readVariables();
    cdcprintf("----------------------\r\n");
    cdcprintf("Dump of NVARS in EEPROM\r\n");
    cdcprintf("----------------------\r\n");
    cdcprintf("Vmax       : %d\r\n", ee_data.Vmax);
    cdcprintf("Vmin       : %d\r\n", ee_data.Vmin);
    cdcprintf("dVdt       : %d\r\n", ee_data.dVdt);
    cdcprintf("steps4unit : %d\r\n", ee_data.steps4unit);
    cdcprintf("----------------------\r\n");
}

//
void help()
{
    cdcprintf("\r\n----------------------\r\n");
    cdcprintf("HELP with commands\r\n");
    cdcprintf("----------------------\r\n");
    cdcprintf("reset    : resets the system\r\n");
    cdcprintf("a        : input Vmax\r\n");
    cdcprintf("b        : input Vmin\r\n");
    cdcprintf("c        : input dVdt\r\n");
    cdcprintf("d        : input steps4unit\r\n");
    cdcprintf("t        : test run in forward\r\n");
    cdcprintf("T        : test run in reverse\r\n");
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
    if ( (stat = eeread32(1, &ee_data.Vmax)) != 0) {
        cdcprintf("Vmax read returned 0x%x\r\n", stat);
    }

    if ( (stat = eeread32(2, &ee_data.Vmin)) != 0) {
        cdcprintf("Vmin read returned 0x%x\r\n", stat);
    }

    if ( (stat = eeread32(3, &ee_data.dVdt)) != 0) {
        cdcprintf("dVdt read returned 0x%x\r\n", stat);
    }

    if ( (stat = eeread32(4, &ee_data.steps4unit)) != 0) {
        cdcprintf("steps4unit read returned 0x%x\r\n", stat);
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
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  HAL_Delay(1000);      //wait for USB reenumeration

  //boot pritnf
  memset(UserRxBufferFS, 0, sizeof(UserRxBufferFS));

  //
  cdcprintf("Malinovski 12.2025 (c) smooker&chichko %d \r\n");

  //
  HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, User_TIMPeriodElapsedCallback);
  // __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  // __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
  // htim3.Instance->ARR=20;
  // __HAL_TIM_ENABLE(&htim3);
  // HAL_TIM_Base_Start_IT(&htim3);

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //migalka za watchdog/main thread
    HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);      //ms
    HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_SET);
    HAL_Delay(10);      //ms

    //MOVEMENT COMMANDS

    //emergencies double by interrupt handler
    if ( HAL_GPIO_ReadPin(ES_L_GPIO_Port, ES_L_Pin) == GPIO_PIN_RESET ) {
        semaphore |= (1 << EL);
    } else {
        semaphore &= ~(1 << EL);
    }
    //emergencies double by interrupt handler
    if ( HAL_GPIO_ReadPin(ES_R_GPIO_Port, ES_R_Pin) == GPIO_PIN_RESET ) {
        semaphore |= (1 << ER);
    } else {
        semaphore &= ~(1 << ER);
    }

    if ( (HAL_GPIO_ReadPin(BUTT_JOGR_GPIO_Port, BUTT_JOGR_Pin) == GPIO_PIN_RESET) ) {
        semaphore |= (1 << JOGR);
    } else {
        semaphore &= ~(1 << JOGR);
    }

    if ( (HAL_GPIO_ReadPin(BUTT_JOGL_GPIO_Port, BUTT_JOGL_Pin) == GPIO_PIN_RESET) ) {
        semaphore |= (1 << JOGL);
    } else {
        semaphore &= ~(1 << JOGL);
    }

    if ( semaphore > 0) {
        printSemaphore();
    }

    debugonly++;

    //JOGL - buttons to GND, limit switches too
    if ( (semaphore & (1 << JOGL)) && ((semaphore & (1 << EL)) == 0) ) {
            cdcprintf("JOGL\r\n");
            printSemaphore();
            while ( semaphore & (1 << JOGL) ) {
                goJog(0, 10, 100);              //jog speed
            }
            semaphore &= ~(1 << JOGL);      //not needed
    }

    //JOGR - buttons to GND, limit switches too
    if ( (semaphore & (1 << JOGR)) && ((semaphore & (1 << ER)) == 0) ) {
            cdcprintf("JOGR\r\n");
            printSemaphore();
            while ( semaphore & (1 << JOGR) ) {
                goJog(1, 10, 100);              //jog speed
            }
            semaphore &= ~(1 << JOGR);      //not needed
    }

    //STEPL
    if ( semaphore & (1 << STEPL) ) {
            cdcprintf("STEPL\r\n");
            printSemaphore();
            goStep(0, ee_data.steps4unit, 100);     //step speed
            semaphore &= ~(1 << STEPL);
    }

    //STEPR
    if ( semaphore & (1 << STEPR) ) {
            cdcprintf("STEPR\r\n");
            printSemaphore();
            goStep(1, ee_data.steps4unit, 100);     //step speed
            semaphore &= ~(1 << STEPR);
    }

    if (rcs == RX_CR) {
        // we were in variable entry mode
        if (rcs2 == RX_ECHO_ON) {
            rcs2 = RX_ECHO_OFF;
            // parse variable values
            if (it == RX_VAR1) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.Vmax);
                cdcprintf(" Vmax value : %d\r\n", ee_data.Vmax);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(1, ee_data.Vmax);
                eeread32(1, &ee_data.Vmax);
                cdcprintf(" Vmax value : %d\r\n", ee_data.Vmax);
            } else if (it == RX_VAR2) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.Vmin);
                cdcprintf(" Vmin value : %d\r\n", ee_data.Vmin);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(2, ee_data.Vmin);
                eeread32(2, &ee_data.Vmin);
                cdcprintf(" Vmin value : %d\r\n", ee_data.Vmin);
            } else if (it == RX_VAR3) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.dVdt);
                cdcprintf(" dVdt value : %d\r\n", ee_data.dVdt);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(3, ee_data.dVdt);
                eeread32(3, &ee_data.dVdt);
                cdcprintf(" dVdt value : %d\r\n", ee_data.dVdt);
            } else if (it == RX_VAR4) {
                tmpret = sscanf(cmd, "%" SCNd32, &ee_data.steps4unit);
                cdcprintf(" dVdt value : %d\r\n", ee_data.steps4unit);
                cdcprintf("ret  : %d\r\n", tmpret);
                eewrite32(4, ee_data.steps4unit);
                eeread32(4, &ee_data.steps4unit);
                cdcprintf(" steps4unit value : %d\r\n", ee_data.steps4unit);
            } else {
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
            cdcprintf("enter value for Vmax:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR1;
        } else if (strcmp(cmd,"b") == 0) {
            cdcprintf("enter value for Vmin:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR2;
        } else if (strcmp(cmd,"c") == 0) {
            cdcprintf("enter value dVdt:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR3;
        } else if (strcmp(cmd,"d") == 0) {
            cdcprintf("enter value steps4unit:");
            rcs2 = RX_ECHO_ON;
            it = RX_VAR4;
        } else if (strcmp(cmd,"g") == 0) {
            cdcprintf("go running!\r\n");
            goStep(0, ee_data.steps4unit, 100);
            // go(1, ee_data.steps4unit, 100);
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  /*Configure GPIO pins : BUTT_STEPL_Pin BUTT_STEPR_Pin ES_L_Pin ES_R_Pin */
  GPIO_InitStruct.Pin = BUTT_STEPL_Pin|BUTT_STEPR_Pin|ES_L_Pin|ES_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //
  if(GPIO_Pin == GPIO_PIN_6) {           //PA6 BUTT_JOGL
    cdcprintf("JL\r\n");                 //fixme debug
    semaphore |= (1 << JOGL);
    //
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    htim3.Instance->ARR=20;     // 100 - 20 = 80 ticks
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
  }

  //
  if(GPIO_Pin == GPIO_PIN_7) {           //PA7 BUTT_JOGR
    cdcprintf("JR\r\n");                 //fixme debug
    semaphore |= (1 << JOGR);
    //
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    htim3.Instance->ARR=20;     // 100 - 20 = 80 ticks
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
  }

  //
  if(GPIO_Pin == GPIO_PIN_0 ) {          //PB0 BUTT_STEPL
    cdcprintf("SL\r\n");
    semaphore |= (1 << STEPL);
  }

  //
  if(GPIO_Pin == GPIO_PIN_1 ) {         //PB1 BUTT_STEPR
    cdcprintf("SR\r\n");
    semaphore |= (1 << STEPR);
  }

  //
  if(GPIO_Pin == GPIO_PIN_10) {         //PB10 ES_L
    cdcprintf("EL\r\n");
    semaphore |= (1 << EL);
    ingo = 0;
  }

  //
  if(GPIO_Pin == GPIO_PIN_11) {         //PB11 ES_R
    cdcprintf("ER\r\n");
    semaphore |= (1 << ER);
    ingo = 0;
  }
}

// buttons  release handling
void User_TIMPeriodElapsedCallback()
{
  //JOG LEFT
  if ( semaphore & (1 << JOGL) ) {            // we are expecting to get the state of ....
    if (HAL_GPIO_ReadPin(BUTT_JOGL_GPIO_Port, BUTT_JOGL_Pin) == GPIO_PIN_RESET) {
        cdcprintf("JLT0\r\n");
    }
    if (HAL_GPIO_ReadPin(BUTT_JOGL_GPIO_Port, BUTT_JOGL_Pin) == GPIO_PIN_SET) {
        cdcprintf("JLT1\r\n");
        semaphore &= ~(1 << JOGL);
    }
  }

  //JOG RIGHT
  if ( semaphore & (1 << JOGR) ) {            // we are expecting to get the state of ....
    if (HAL_GPIO_ReadPin(BUTT_JOGR_GPIO_Port, BUTT_JOGR_Pin) == GPIO_PIN_RESET) {
        cdcprintf("JRT0\r\n");
    }
    if (HAL_GPIO_ReadPin(BUTT_JOGR_GPIO_Port, BUTT_JOGR_Pin) == GPIO_PIN_SET) {
        cdcprintf("JRT1\r\n");
        semaphore &= ~(1 << JOGR);
    }
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
