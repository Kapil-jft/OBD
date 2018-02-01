/*--------------------------------Includes-------------------------------------*/
#include "GPIO.h"
#include "stm32f30x_rcc.h"



/*
  Deinitializes the GPIOx peripheral registers to their default reset 
  values.
  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
*/

void GPIO_DeInit(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  if(GPIOx == GPIOA)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, DISABLE);
  }
  else if(GPIOx == GPIOB)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOB, DISABLE);
  }
  else if(GPIOx == GPIOC)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOC, DISABLE);
  }
  else if(GPIOx == GPIOD)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOD, DISABLE);
  }
  else if(GPIOx == GPIOE)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOE, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOE, DISABLE);
  }
  else
  {
    if(GPIOx == GPIOF)
    {
      RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOF, ENABLE);
      RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOF, DISABLE);
    }
  }
}

/*
	Initializes the GPIOx peripheral according to the specified parameter
	GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral
	GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that contains the configuration information for the specified GPIO peripheral
	GPIO_Pin: selects the pin to be configured
	GPIO_Pin_0->GPIO_Pin_15 for GPIOA, GPIOB, GPIOC, GPIOD and GPIOE;
	GPIO_Pin_0->GPIO_Pin_2, GPIO_Pin_4, GPIO_Pin_6, GPIO_Pin_9 and GPIO_Pin_10 for GPIOF
*/
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{ 
  uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));
  assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
  assert_param(IS_GPIO_PUPD(GPIO_InitStruct->GPIO_PuPd));

  /*-------------------------- Configure the port pins -----------------------*/
  /*-- GPIO Mode Configuration --*/
  for (pinpos = 0x00; pinpos < 0x10; pinpos++)
  {
    pos = ((uint32_t)0x01) << pinpos;

    /* Get the port pins position */
    currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

    if (currentpin == pos)
    {
      if ((GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_AF))
      {
        /* Check Speed mode parameters */
        assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));

        /* Speed mode configuration */
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
        GPIOx->OSPEEDR |= ((uint32_t)(GPIO_InitStruct->GPIO_Speed) << (pinpos * 2));

        /* Check Output mode parameters */
        assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));

        /* Output mode configuration */
        GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos));
        GPIOx->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStruct->GPIO_OType) << ((uint16_t)pinpos));
      }
      
      GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));

      GPIOx->MODER |= (((uint32_t)GPIO_InitStruct->GPIO_Mode) << (pinpos * 2));

      /* Pull-up Pull down resistor configuration */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
      GPIOx->PUPDR |= (((uint32_t)GPIO_InitStruct->GPIO_PuPd) << (pinpos * 2));
    }
  }
}


/*-----------------------------Read Input Data-------------------------------------*/
/*
	Reads the specified input port pin
	GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral
	GPIO_Pin: specifies the port bit to read
	GPIO_Pin_x where x can be (0..15) for GPIOA, GPIOB, GPIOC, GPIOD or GPIOE;
  (0..2, 4, 6, 9..10) for GPIOF.
*/

uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint8_t bitstatus = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));

  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

/*
	Reads the specified input port pin.
	GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral
*/
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  return ((uint16_t)GPIOx->IDR);
}


/*-----------------------------Read Specified Outpt Data------------------------------------*/
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint8_t bitstatus = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
  
  if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

/*
	Reads the specified Output port pin.
	GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral
*/

uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((uint16_t)GPIOx->ODR);
}

/*------------------------Sets the selected data port bits------------------------*/
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BSRR = GPIO_Pin;
}

/*-----------------------Resets the selected data port bits--------------------------*/
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BRR = GPIO_Pin;
}


/*----------------------Sets or clears the selected data port bit---------------------*/
/*
	GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  GPIO_Pin: specifies the port bit to be written.
  This parameter can be GPIO_Pin_x where x can be (0..15) for GPIOA, GPIOB, GPIOC, GPIOD or GPIOE (0..2, 4, 6, 9..10) for GPIOF.
*/
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_BIT_ACTION(BitVal));
  
  if (BitVal != Bit_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin ;
  }
}


/*----------------------Writes data to the specified GPIO data port----------------------*/
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  GPIOx->ODR = PortVal;
}


/*----------------------------GPIO Alternate functions configuration------------------------*/
/*
	GPIO_AF: selects the pin to be used as Alternate function
	GPIO_PinSource: specifies the pin for the Alternate function
*/
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  assert_param(IS_GPIO_AF(GPIO_AF));
  
  temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
}

/*
	@
*/
/*-----------------------------------------------End Of File-------------------------------------------------------*/















































	