/*--------------------INCLUDES-------------------*/
#include "UART.h"
#include "stm32f3xx.h"                  // Device header
#include "stm32f30x_rcc.h"

/*----------------Private Define--------------------*/
/*USART CR1 register clear Mask ((~(uint32_t)0xFFFFE6F3)) */
#define CR1_CLEAR_MASK            ((uint32_t)(USART_CR1_M | USART_CR1_PCE | \
                                              USART_CR1_PS | USART_CR1_TE | \
                                              USART_CR1_RE))

/*USART CR2 register clock bits clear Mask ((~(uint32_t)0xFFFFF0FF)) */
#define CR2_CLOCK_CLEAR_MASK      ((uint32_t)(USART_CR2_CLKEN | USART_CR2_CPOL | \
                                              USART_CR2_CPHA | USART_CR2_LBCL))

/*USART CR3 register clear Mask ((~(uint32_t)0xFFFFFCFF)) */
#define CR3_CLEAR_MASK            ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))

/*USART Interrupts mask */
#define IT_MASK                   ((uint32_t)0x000000FF)


/*--------Deinitializes the USARTx peripheral registers to their default reset values------*/
void USART_DeInit(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));

  if (USARTx == USART1)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
  }
  else if (USARTx == USART2)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
  }
  else if (USARTx == USART3)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
  }
  else if (USARTx == UART4)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);
  }
  else
  {
    if  (USARTx == UART5)
    {
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);
    }
  }
}


/*------------------Initializes the USARTx peripheral--------------------------*/
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
  uint32_t divider = 0, apbclock = 0, tmpreg = 0;
  RCC_ClocksTypeDef RCC_ClocksStatus;
  
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_BAUDRATE(USART_InitStruct->USART_BaudRate));
  assert_param(IS_USART_WORD_LENGTH(USART_InitStruct->USART_WordLength));
  assert_param(IS_USART_STOPBITS(USART_InitStruct->USART_StopBits));
  assert_param(IS_USART_PARITY(USART_InitStruct->USART_Parity));
  assert_param(IS_USART_MODE(USART_InitStruct->USART_Mode));
  assert_param(IS_USART_HARDWARE_FLOW_CONTROL(USART_InitStruct->USART_HardwareFlowControl));
  
  /* Disable USART */
  USARTx->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
  
  /*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);
  
  /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;
  
  /* Write to USART CR2 */
  USARTx->CR2 = tmpreg;
  
  /*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;
  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);
  
  /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
    USART_InitStruct->USART_Mode;
  
  /* Write to USART CR1 */
  USARTx->CR1 = tmpreg;
  
  /*---------------------------- USART CR3 Configuration -----------------------*/
  tmpreg = USARTx->CR3;
  /* Clear CTSE and RTSE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR3_CLEAR_MASK);
  
  /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
  tmpreg |= USART_InitStruct->USART_HardwareFlowControl;
  
  /* Write to USART CR3 */
  USARTx->CR3 = tmpreg;
  
  /*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  
  if (USARTx == USART1)
  {
    apbclock = RCC_ClocksStatus.USART1CLK_Frequency;
  }
  else if (USARTx == USART2)
  {
    apbclock = RCC_ClocksStatus.USART2CLK_Frequency;
  }
  else if (USARTx == USART3)
  {
    apbclock = RCC_ClocksStatus.USART3CLK_Frequency;
  }
  else if (USARTx == UART4)
  {
    apbclock = RCC_ClocksStatus.UART4CLK_Frequency;
  }
  else 
  {
    apbclock = RCC_ClocksStatus.UART5CLK_Frequency;
  }  
  
  /* Determine the integer part */
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
  {
    /* (divider * 10) computing in case Oversampling mode is 8 Samples */
    divider = (uint32_t)((2 * apbclock) / (USART_InitStruct->USART_BaudRate));
    tmpreg  = (uint32_t)((2 * apbclock) % (USART_InitStruct->USART_BaudRate));
  }
  else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
  {
    /* (divider * 10) computing in case Oversampling mode is 16 Samples */
    divider = (uint32_t)((apbclock) / (USART_InitStruct->USART_BaudRate));
    tmpreg  = (uint32_t)((apbclock) % (USART_InitStruct->USART_BaudRate));
  }
  
  /* round the divider : if fractional part i greater than 0.5 increment divider */
  if (tmpreg >=  (USART_InitStruct->USART_BaudRate) / 2)
  {
    divider++;
  } 
  
  /* Implement the divider in case Oversampling mode is 8 Samples */
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
  {
    /* get the LSB of divider and shift it to the right by 1 bit */
    tmpreg = (divider & (uint16_t)0x000F) >> 1;
    
    /* update the divider value */
    divider = (divider & (uint16_t)0xFFF0) | tmpreg;
  }
  
  /* Write to USART BRR */
  USARTx->BRR = (uint16_t)divider;
}



/*-------------------USART_InitStruct----------------------*/
void USART_StructInit(USART_InitTypeDef* USART_InitStruct)
{
  /* USART_InitStruct members default value */
  USART_InitStruct->USART_BaudRate = 9600;
  USART_InitStruct->USART_WordLength = USART_WordLength_8b;
  USART_InitStruct->USART_StopBits = USART_StopBits_1;
  USART_InitStruct->USART_Parity = USART_Parity_No ;
  USART_InitStruct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
}


/*---------------Initializes the USARTx peripheral Clock---------------*/
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  assert_param(IS_USART_CLOCK(USART_ClockInitStruct->USART_Clock));
  assert_param(IS_USART_CPOL(USART_ClockInitStruct->USART_CPOL));
  assert_param(IS_USART_CPHA(USART_ClockInitStruct->USART_CPHA));
  assert_param(IS_USART_LASTBIT(USART_ClockInitStruct->USART_LastBit));
/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear CLKEN, CPOL, CPHA, LBCL and SSM bits */
  tmpreg &= (uint32_t)~((uint32_t)CR2_CLOCK_CLEAR_MASK);
  /* Configure the USART Clock, CPOL, CPHA, LastBit and SSM ------------*/
  /* Set CLKEN bit according to USART_Clock value */
  /* Set CPOL bit according to USART_CPOL value */
  /* Set CPHA bit according to USART_CPHA value */
  /* Set LBCL bit according to USART_LastBit value */
  tmpreg |= (uint32_t)(USART_ClockInitStruct->USART_Clock | USART_ClockInitStruct->USART_CPOL | 
                       USART_ClockInitStruct->USART_CPHA | USART_ClockInitStruct->USART_LastBit);
  /* Write to USART CR2 */
  USARTx->CR2 = tmpreg;
}



/*----------------USART_ClockInitStruct----------------*/
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  /* USART_ClockInitStruct members default value */
  USART_ClockInitStruct->USART_Clock = USART_Clock_Disable;
  USART_ClockInitStruct->USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct->USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStruct->USART_LastBit = USART_LastBit_Disable;
}


/*----------Enables or disables the specified USART peripheral-------------*/
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected USART by setting the UE bit in the CR1 register */
    USARTx->CR1 |= USART_CR1_UE;
  }
  else
  {
    /* Disable the selected USART by clearing the UE bit in the CR1 register */
    USARTx->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);
  }
}

/*----------Enables or disables the USART's transmitter or receiver--------*/
/*
	USART_Mode_Tx: USART Transmitter
  USART_Mode_Rx: USART Receiver
	specifies the USART direction
*/
void USART_DirectionModeCmd(USART_TypeDef* USARTx, uint32_t USART_DirectionMode, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_MODE(USART_DirectionMode));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 

  if (NewState != DISABLE)
  {
    /* Enable the USART's transfer interface by setting the TE and/or RE bits 
       in the USART CR1 register */
    USARTx->CR1 |= USART_DirectionMode;
  }
  else
  {
    /* Disable the USART's transfer interface by clearing the TE and/or RE bits
       in the USART CR3 register */
    USARTx->CR1 &= (uint32_t)~USART_DirectionMode;
  }
}


/*----------Enables or disables the USART's 8x oversampling mode---------*/
/*
	This function has to be called before calling USART_Init()
*/
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the 8x Oversampling mode by setting the OVER8 bit in the CR1 register */
    USARTx->CR1 |= USART_CR1_OVER8;
  }
  else
  {
    /* Disable the 8x Oversampling mode by clearing the OVER8 bit in the CR1 register */
    USARTx->CR1 &= (uint32_t)~((uint32_t)USART_CR1_OVER8);
  }
}

/*--------Enables or disables the USART's one bit sampling method----------*/
/*
	This function has to be called before calling USART_Cmd() function
*/
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the one bit method by setting the ONEBITE bit in the CR3 register */
    USARTx->CR3 |= USART_CR3_ONEBIT;
  }
  else
  {
    /* Disable the one bit method by clearing the ONEBITE bit in the CR3 register */
    USARTx->CR3 &= (uint32_t)~((uint32_t)USART_CR3_ONEBIT);
  }
}


/*--------------Enables or disables the USART's most significant bit first--------*/
/*
	This function has to be called before calling USART_Cmd() function
	Transmitting and Receiving MSB first
*/
void USART_MSBFirstCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the most significant bit first transmitted/received following the
       start bit by setting the MSBFIRST bit in the CR2 register */
    USARTx->CR2 |= USART_CR2_MSBFIRST;
  }
  else
  {
    /* Disable the most significant bit first transmitted/received following the
       start bit by clearing the MSBFIRST bit in the CR2 register */
    USARTx->CR2 &= (uint32_t)~((uint32_t)USART_CR2_MSBFIRST);
  }
}


/*---------------Sets the system clock prescaler---------------*/
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));

  /* Clear the USART prescaler */
  USARTx->GTPR &= USART_GTPR_GT;
  /* Set the USART prescaler */
  USARTx->GTPR |= USART_Prescaler;
}


/*-----------Transmits single data through the USARTx peripheral---------*/
/*
	Data: the data to transmit
*/
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 

  /* Transmit Data */
  USARTx->TDR = (Data & (uint16_t)0x01FF);
}


/*
	Returns the most recent received data by the USARTx
*/
uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));

  /* Receive Data */
  return (uint16_t)(USARTx->RDR & (uint16_t)0x01FF);
}


/*--------------Enables or disables the USART's Overrun detection------------*/
void USART_OverrunDetectionConfig(USART_TypeDef* USARTx, uint32_t USART_OVRDetection)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_OVRDETECTION(USART_OVRDetection));
  
  /* Clear the OVR detection bit */
  USARTx->CR3 &= (uint32_t)~((uint32_t)USART_CR3_OVRDIS);
  /* Set the new value for the OVR detection bit */
  USARTx->CR3 |= USART_OVRDetection;
}



/*
	@
*/


/*-------------------------------END OF FILE-----------------------------------*/















































