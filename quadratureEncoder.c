/*
   FILE          : quadratureEncoder.c
   PROJECT       : QuadratureEncoder/DC Motor/Stm32f3 Discovery Board/Linux
   PROGRAMMER    : Rohit Bhardwaj
   DESCRIPTION   : This program measures the incremental position of a turning motor shaft. A command is present that allows the position 
                   of the motor to be set, and has the motor automatically turn forwards or backwards to reach the specified position
                   
	The program make use of HAL(Hardware Abstraction Layer) which is C code that implements basic drivers for all the peripherals 
	in the STM32 Family. It makes the code more portable over STM32 Family.
*/

#include <stdint.h>
#include <stdio.h>
#include "stm32f3xx_hal.h"
#include "common.h"

uint32_t enable = 0;
uint32_t motorstop = 0;

// FUNCTION      : gpioinit()
// DESCRIPTION   : The function initialize GPIO pins for the DC motor
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void gpioinit(int mode)
{
/* Turn on clocks to I/O */
__GPIOF_CLK_ENABLE();

/* Configure GPIO pins */
GPIO_InitTypeDef  GPIO_InitStruct;
GPIO_InitStruct.Pin = (GPIO_PIN_2 | GPIO_PIN_4);
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
GPIO_InitStruct.Alternate = 0;
HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
return;

}

ADD_CMD("gpioinit",gpioinit,"              Initialize GPIO Pins");


/*Global Handle Structure*/
TIM_HandleTypeDef tim1;
TIM_OC_InitTypeDef sConfig;

// FUNCTION      : dcinit()
// DESCRIPTION   : The function initializes the timer 1 ,PWM & GPIO Pins
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void dcinit(int mode)
{
/* Turn on clocks to I/O */
__GPIOA_CLK_ENABLE();

/* Configure GPIO pins */
GPIO_InitTypeDef  GPIO_InitStruct;
GPIO_InitStruct.Pin = (GPIO_PIN_8 | GPIO_PIN_9);
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
GPIO_InitStruct.Alternate = 6;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

uint32_t rc;

/* Initialize the Timer(PWM) */ 
__TIM1_CLK_ENABLE(); 
tim1.Instance = TIM1; 
tim1.Init.Prescaler     = HAL_RCC_GetPCLK2Freq()*2/1000000; 
tim1.Init.CounterMode   = TIM_COUNTERMODE_UP; 
tim1.Init.Period        = 1000; 
tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
tim1.Init.RepetitionCounter = 0; 

/*Initalize the Timer*/
rc = HAL_TIM_Base_Init(&tim1);
if(rc != HAL_OK) 
 {
  printf("Unable to initalize Timer, rc=%d\n",(unsigned)rc);
  return;
 }

/*Start the timer*/
 rc = HAL_TIM_Base_Start(&tim1);
 if(rc != HAL_OK) 
 {
  printf("Unable to start the timer, rc=%d\n",(unsigned)rc);
  return;
 }

/*Configure output:*/
sConfig.OCMode       = TIM_OCMODE_PWM1; 
sConfig.Pulse        = 500; 
sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH; 
sConfig.OCNPolarity  = TIM_OCNPOLARITY_LOW; 
sConfig.OCFastMode   = TIM_OCFAST_DISABLE; 
sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET; 
sConfig.OCNIdleState =TIM_OCNIDLESTATE_RESET;
HAL_TIM_PWM_ConfigChannel(&tim1,&sConfig,TIM_CHANNEL_1);
HAL_TIM_PWM_ConfigChannel(&tim1,&sConfig,TIM_CHANNEL_2);

//Enable interrupt 
 HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 1);
 HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
 TIM1-> DIER |= 0;
 TIM1->CNT = 0;       /* Reset counter */
}

ADD_CMD("dcinit",dcinit,"              Initialize the Timer1,PWM & GPIO Pins");

/*Global Handle Structure*/
TIM_HandleTypeDef tim3;

// FUNCTION      : encoderinit()
// DESCRIPTION   : The function initializes the timer into encoder mode 
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void encoderinit(int mode)
{
/* Turn on clocks to I/O */
__GPIOC_CLK_ENABLE();

/* Configure GPIO pins */
GPIO_InitTypeDef  GPIO_InitStruct;
GPIO_InitStruct.Pin = (GPIO_PIN_6 | GPIO_PIN_7);
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
GPIO_InitStruct.Alternate = 2;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

HAL_StatusTypeDef rc;

__TIM3_CLK_ENABLE();

tim3.Instance = TIM3;
tim3.Init.Prescaler = 0;
tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
tim3.Init.Period = 0xffff;
tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
tim3.Init.RepetitionCounter = 0;
rc = HAL_TIM_Base_Init(&tim3);
/*Initalize the Timer*/
 if(rc != HAL_OK)
 {
   printf("Failed to initialize Timer 3 Base,  rc=%d\n",(unsigned)rc);
   return;
 }

/*Start the timer*/
 rc = HAL_TIM_Base_Start(&tim3);
 if(rc != HAL_OK) 
 {
  printf("Unable to start the Timer 3 Base, rc=%d\n",(unsigned)rc);
  return;
 }

//Initialize Encoder data structures
TIM_Encoder_InitTypeDef encoderConfig;
encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
encoderConfig.IC1Polarity = 0;
encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
encoderConfig.IC1Prescaler = 0;
encoderConfig.IC1Filter = 3;
encoderConfig.IC2Polarity = 0;
encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
encoderConfig.IC2Prescaler = 0;
encoderConfig.IC2Filter = 3;

// Call HAL encoder Init
 rc = HAL_TIM_Encoder_Init(&tim3,&encoderConfig);
 if(rc != HAL_OK) 
 {
  printf("Failed to initialize Timer 3 Encoder, rc=%d\n",(unsigned)rc);
  return;
 }

/*Start the encoder channel 1*/
 rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_1);
 if(rc != HAL_OK) 
 {
  printf("Failed to start Timer 3 Encoder, rc=%d\n",(unsigned)rc);
  return;
 }

/*Start the encoder channel 2*/
 rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_2);
 if(rc != HAL_OK)
 {
  printf("Failed to start Timer 3 Encoder, rc=%d\n",(unsigned)rc);
  return;
 }
}

ADD_CMD("encoderinit",encoderinit,"              Initialize the Timer3,Encoder& GPIO Pins");

// FUNCTION      : dc()
// DESCRIPTION   : The function set DC motor direction
//                 0 – Brake, 1 – Forward, 2 – Reverse
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void dc(int mode)     
{
  if(mode != CMD_INTERACTIVE)
  {
    return;
  }

 uint32_t dir;

 fetch_uint32_arg(&dir);

 HAL_TIM_PWM_Start(&tim1,TIM_CHANNEL_1); 

 if(dir == 0)
 {
   /* Brake */ 
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    //1A
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0);    //2A
 }

 else if(dir == 1)
  {
   /* Forward */ 
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,1);    //1A
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0);   //start 2A
  }

 else if(dir == 2)
  {
   /* Reverse*/  
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    //1A
   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,1);    //stop 2A
  }
}
ADD_CMD("dc",dc,"                  dc<dir>");

// FUNCTION      : pwm()
// DESCRIPTION   : The function controls the speed of the DC Motor
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void pwm(int mode)     
{
  if(mode != CMD_INTERACTIVE)
  {
    return;
  }

 uint32_t value;

 fetch_uint32_arg(&value);

 TIM1->CCR1 = value;
}

ADD_CMD("pwm",pwm,"                  pwm<value>");

// FUNCTION      : encoder()
// DESCRIPTION   : The function prints the current encoder count
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void encoder(int mode)     
{
  if(mode != CMD_INTERACTIVE)
  {
    return;
  }
 printf("Timer 3 Encoder : %ld\n",TIM3->CNT);
}

ADD_CMD("encoder",encoder,"                  encoder");


// FUNCTION      : TIM1_UP_TIM16_IRQHandler()
// DESCRIPTION   : The function sets the position of the motor, and the motor automatically 
//                 turn forwards or backwards to reach the specified position
// PARAMETERS    : nothing
// RETURNS       : returns nothing
void TIM1_UP_TIM16_IRQHandler(void)    //startup_stm32f303xc.s
{
 TIM1 ->SR &= ~1;//~UIF  reset the flag in timer
 if(enable)
 {
   if(TIM3->CNT < motorstop)
   {
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    //low 1A   1A = PF2   ,   2A = PF4
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,1);    //High 2A   To turn right
     if (TIM3->CNT == motorstop)
     {
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0);
     } 
    }
   else
   {
    if(TIM3->CNT > motorstop)
    {
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,1);    //low 1A   1A = PF2   ,   2A = PF4
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0);    //High 2A   To turn right
     if (TIM3->CNT == motorstop)
     {
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,0);    
      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,0);
     } 
    }
   } 
 }
}

// FUNCTION      : monitor()
// DESCRIPTION   : The function fetches the value from the commands
// PARAMETERS    : The function accepts an int value
// RETURNS       : returns nothing
void monitor(int mode)
{
  if(mode != CMD_INTERACTIVE)
  {
    return;
  }
  fetch_uint32_arg(&enable);
  fetch_uint32_arg(&motorstop);
}

ADD_CMD("motorstop",monitor,"            motorstop<enable><motorstop>");
