/*---------------------Define to prevent recursive inclusion-------------------------*/
#ifndef __STM32F30x_USART_H
#define __STM32F30x_USART_H


#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
/*---------------------------------Includes-----------------------------------*/
#include "stm32f3xx.h"                  // Device header


	 
	 
/*----------------------USART Init Structure definition----------------------*/
typedef struct
{
  uint32_t USART_BaudRate;												/*The baud rate is computed using the following formula:
																									IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
																									FractionalDivider = ((IntegerDivider - ((uint32_t) IntegerDivider)) * 16) + 0.5*/
	
	uint32_t USART_WordLength;											/*Number of data bits transmitted or received in a frame*/
	
	uint32_t USART_StopBits;												/*Number of Stop Bits Transmitted*/
	
	uint32_t USART_Parity;													/*Parity Mode*/
	
	uint32_t USART_Mode;														/*Wether the Receive or Transmit mode is enabled or disabled*/
	
	uint32_t USART_HardwareFlowControl; 						/*wether the hardware flow control mode is enabled or disabled*/
	
} USART_InitTypeDef;


/*-------------------USART Clock Init Structure definition----------------------*/
typedef struct
{
  uint32_t USART_Clock;													/*whether the USART clock is enabled or disabled*/
	
	uint32_t USART_CPOL;													/*steady state of the serial clock.*/
	
	uint32_t USART_CPHA;													/*clock transition on which the bit capture is made*/
	
	uint32_t USART_LastBit;												/*whether the clock pulse corresponding to the last transmitted*/
	
}USART_ClockInitTypeDef;


#define IS_USART_ALL_PERIPH(PERIPH) (((PERIPH) == USART1) || \
                                     ((PERIPH) == USART2) || \
                                     ((PERIPH) == USART3) || \
                                     ((PERIPH) == UART4) || \
                                     ((PERIPH) == UART5))

/*----------------USART Word Length-----------------*/
#define USART_WordLength_8b                  ((uint32_t)0x00000000)
#define USART_WordLength_9b                  USART_CR1_M
#define IS_USART_WORD_LENGTH(LENGTH) (((LENGTH) == USART_WordLength_8b) || \
                                      ((LENGTH) == USART_WordLength_9b))


/*----------------USART Stop Bits--------------------*/
#define USART_StopBits_1                     ((uint32_t)0x00000000)
#define USART_StopBits_2                     USART_CR2_STOP_1
#define USART_StopBits_1_5                   (USART_CR2_STOP_0 | USART_CR2_STOP_1)
#define IS_USART_STOPBITS(STOPBITS) (((STOPBITS) == USART_StopBits_1) || \
                                     ((STOPBITS) == USART_StopBits_2) || \
                                     ((STOPBITS) == USART_StopBits_1_5))
																		 
																		 
/*----------------------USART_Parity------------------*/ 
 
#define USART_Parity_No                      ((uint32_t)0x00000000)
#define USART_Parity_Even                    USART_CR1_PCE
#define USART_Parity_Odd                     (USART_CR1_PCE | USART_CR1_PS) 
#define IS_USART_PARITY(PARITY) (((PARITY) == USART_Parity_No) || \
                                 ((PARITY) == USART_Parity_Even) || \
                                 ((PARITY) == USART_Parity_Odd))
																 
																 
/*------------------USART_Mode----------------------*/
#define USART_Mode_Rx                        USART_CR1_RE
#define USART_Mode_Tx                        USART_CR1_TE
#define IS_USART_MODE(MODE) ((((MODE) & (uint32_t)0xFFFFFFF3) == 0x00) && \
                              ((MODE) != (uint32_t)0x00))


/*----------------USART_Hardware_Flow_Control--------------*/
#define USART_HardwareFlowControl_None       ((uint32_t)0x00000000)
#define USART_HardwareFlowControl_RTS        USART_CR3_RTSE
#define USART_HardwareFlowControl_CTS        USART_CR3_CTSE
#define USART_HardwareFlowControl_RTS_CTS    (USART_CR3_RTSE | USART_CR3_CTSE)
#define IS_USART_HARDWARE_FLOW_CONTROL(CONTROL)\
                              (((CONTROL) == USART_HardwareFlowControl_None) || \
                               ((CONTROL) == USART_HardwareFlowControl_RTS) || \
                               ((CONTROL) == USART_HardwareFlowControl_CTS) || \
                               ((CONTROL) == USART_HardwareFlowControl_RTS_CTS))
															 
															 
/*-------------------USART_Clock-----------------------*/
#define USART_Clock_Disable                  ((uint32_t)0x00000000)
#define USART_Clock_Enable                   USART_CR2_CLKEN
#define IS_USART_CLOCK(CLOCK) (((CLOCK) == USART_Clock_Disable) || \
                               ((CLOCK) == USART_Clock_Enable))
															 
															 

/*---------------USART_Clock_Polarity-----------------*/
#define USART_CPOL_Low                       ((uint32_t)0x00000000)
#define USART_CPOL_High                      USART_CR2_CPOL
#define IS_USART_CPOL(CPOL) (((CPOL) == USART_CPOL_Low) || ((CPOL) == USART_CPOL_High))



/*-----------------USART_Clock_Phase--------------------*/
#define USART_CPHA_1Edge                     ((uint32_t)0x00000000)
#define USART_CPHA_2Edge                     USART_CR2_CPHA
#define IS_USART_CPHA(CPHA) (((CPHA) == USART_CPHA_1Edge) || ((CPHA) == USART_CPHA_2Edge))


/*-----------------USART_Last_Bit------------------*/
#define USART_LastBit_Disable                ((uint32_t)0x00000000)
#define USART_LastBit_Enable                 USART_CR2_LBCL
#define IS_USART_LASTBIT(LASTBIT) (((LASTBIT) == USART_LastBit_Disable) || \
                                   ((LASTBIT) == USART_LastBit_Enable))
																	 
/*-------------USART_Address_Detection---------------*/	
#define USART_AddressLength_4b               ((uint32_t)0x00000000)
#define USART_AddressLength_7b               USART_CR2_ADDM7
#define IS_USART_ADDRESS_DETECTION(ADDRESS) (((ADDRESS) == USART_AddressLength_4b) || \
                                             ((ADDRESS) == USART_AddressLength_7b))
																						 
/*--------------USART_StopMode_WakeUp_methods-------------*/
#define USART_WakeUpSource_AddressMatch      ((uint32_t)0x00000000)
#define USART_WakeUpSource_StartBit          USART_CR3_WUS_1
#define USART_WakeUpSource_RXNE              (uint32_t)(USART_CR3_WUS_0 | USART_CR3_WUS_1)
#define IS_USART_STOPMODE_WAKEUPSOURCE(SOURCE) (((SOURCE) == USART_WakeUpSource_AddressMatch) || \
                                                ((SOURCE) == USART_WakeUpSource_StartBit) || \
                                                ((SOURCE) == USART_WakeUpSource_RXNE))
																								
/*---------------USART_AutoBaudRate_Mode ----------------*/
#define USART_AutoBaudRate_StartBit                 ((uint32_t)0x00000000)
#define USART_AutoBaudRate_FallingEdge              USART_CR2_ABRMODE_0
#define USART_AutoBaudRate_0x7FFrame                USART_CR2_ABRMODE_1
#define USART_AutoBaudRate_0x55Frame                (USART_CR2_ABRMODE_0 | USART_CR2_ABRMODE_1)
#define IS_USART_AUTOBAUDRATE_MODE(MODE) (((MODE) == USART_AutoBaudRate_StartBit) || \
                                          ((MODE) == USART_AutoBaudRate_FallingEdge) || \
                                          ((MODE) == USART_AutoBaudRate_0x7FFrame) || \
                                          ((MODE) == USART_AutoBaudRate_0x55Frame))
																					
																					
																					
/*----------------USART_OVR_DETECTION--------------------*/
#define USART_OVRDetection_Enable            ((uint32_t)0x00000000)
#define USART_OVRDetection_Disable           USART_CR3_OVRDIS
#define IS_USART_OVRDETECTION(OVR) (((OVR) == USART_OVRDetection_Enable)|| \
                                    ((OVR) == USART_OVRDetection_Disable))
																		

/*-----------------USART_Request--------------------*/
#define USART_Request_ABRRQ                  USART_RQR_ABRRQ
#define USART_Request_SBKRQ                  USART_RQR_SBKRQ
#define USART_Request_MMRQ                   USART_RQR_MMRQ
#define USART_Request_RXFRQ                  USART_RQR_RXFRQ
#define USART_Request_TXFRQ                  USART_RQR_TXFRQ

#define IS_USART_REQUEST(REQUEST) (((REQUEST) == USART_Request_TXFRQ) || \
                                   ((REQUEST) == USART_Request_RXFRQ) || \
                                   ((REQUEST) == USART_Request_MMRQ) || \
                                   ((REQUEST) == USART_Request_SBKRQ) || \
                                   ((REQUEST) == USART_Request_ABRRQ))


/*----------------USART_Interrupt_definition-----------------*/
#define USART_IT_WU                          ((uint32_t)0x00140316)
#define USART_IT_CM                          ((uint32_t)0x0011010E)
#define USART_IT_EOB                         ((uint32_t)0x000C011B)
#define USART_IT_RTO                         ((uint32_t)0x000B011A)
#define USART_IT_PE                          ((uint32_t)0x00000108)
#define USART_IT_TXE                         ((uint32_t)0x00070107)
#define USART_IT_TC                          ((uint32_t)0x00060106)
#define USART_IT_RXNE                        ((uint32_t)0x00050105)
#define USART_IT_IDLE                        ((uint32_t)0x00040104)
#define USART_IT_LBD                         ((uint32_t)0x00080206)
#define USART_IT_CTS                         ((uint32_t)0x0009030A) 
#define USART_IT_ERR                         ((uint32_t)0x00000300)
#define USART_IT_ORE                         ((uint32_t)0x00030300)
#define USART_IT_NE                          ((uint32_t)0x00020300)
#define USART_IT_FE                          ((uint32_t)0x00010300)

#define IS_USART_CONFIG_IT(IT) (((IT) == USART_IT_PE) || ((IT) == USART_IT_TXE) || \
                                ((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                                ((IT) == USART_IT_IDLE) || ((IT) == USART_IT_LBD) || \
                                ((IT) == USART_IT_CTS) || ((IT) == USART_IT_ERR) || \
                                ((IT) == USART_IT_RTO) || ((IT) == USART_IT_EOB) || \
                                ((IT) == USART_IT_CM) || ((IT) == USART_IT_WU))

#define IS_USART_GET_IT(IT) (((IT) == USART_IT_PE) || ((IT) == USART_IT_TXE) || \
                             ((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                             ((IT) == USART_IT_IDLE) || ((IT) == USART_IT_LBD) || \
                             ((IT) == USART_IT_CTS) || ((IT) == USART_IT_ORE) || \
                             ((IT) == USART_IT_NE) || ((IT) == USART_IT_FE) || \
                             ((IT) == USART_IT_RTO) || ((IT) == USART_IT_EOB) || \
                             ((IT) == USART_IT_CM) || ((IT) == USART_IT_WU))

#define IS_USART_CLEAR_IT(IT) (((IT) == USART_IT_TC) || ((IT) == USART_IT_PE) || \
                               ((IT) == USART_IT_FE) || ((IT) == USART_IT_NE) || \
                               ((IT) == USART_IT_ORE) || ((IT) == USART_IT_IDLE) || \
                               ((IT) == USART_IT_LBD) || ((IT) == USART_IT_CTS) || \
                               ((IT) == USART_IT_RTO) || ((IT) == USART_IT_EOB) || \
                               ((IT) == USART_IT_CM) || ((IT) == USART_IT_WU))
															 
/*-------------------------USART_Global_definition-----------------------*/
#define IS_USART_BAUDRATE(BAUDRATE) (((BAUDRATE) > 0) && ((BAUDRATE) < 0x005B8D81))
#define IS_USART_DE_ASSERTION_DEASSERTION_TIME(TIME) ((TIME) <= 0x1F)
#define IS_USART_AUTO_RETRY_COUNTER(COUNTER) ((COUNTER) <= 0x7)
#define IS_USART_TIMEOUT(TIMEOUT) ((TIMEOUT) <= 0x00FFFFFF)
#define IS_USART_DATA(DATA) ((DATA) <= 0x1FF)



/* Initialization and Configuration functions *********************************/
void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_DirectionModeCmd(USART_TypeDef* USARTx, uint32_t USART_DirectionMode, FunctionalState NewState);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_MSBFirstCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_DataInvCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_InvPinCmd(USART_TypeDef* USARTx, uint32_t USART_InvPin, FunctionalState NewState);
void USART_SWAPPinCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ReceiverTimeOutCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetReceiverTimeOut(USART_TypeDef* USARTx, uint32_t USART_ReceiverTimeOut);

/* STOP Mode functions ********************************************************/
void USART_STOPModeCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_StopModeWakeUpSourceConfig(USART_TypeDef* USARTx, uint32_t USART_WakeUpSource);

/* AutoBaudRate functions *****************************************************/
void USART_AutoBaudRateCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_AutoBaudRateConfig(USART_TypeDef* USARTx, uint32_t USART_AutoBaudRate);

/* Data transfers functions ***************************************************/
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

/* Multi-Processor Communication functions ************************************/
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_MuteModeWakeUpConfig(USART_TypeDef* USARTx, uint32_t USART_WakeUp);
void USART_MuteModeCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_AddressDetectionConfig(USART_TypeDef* USARTx, uint32_t USART_AddressLength);
///* LIN mode functions *********************************************************
//void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint32_t USART_LINBreakDetectLength);
//void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);

// Half-duplex mode function **************************************************
//void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);

//Smartcard mode functions ***************************************************
//void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
//void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
//void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
//void USART_SetAutoRetryCount(USART_TypeDef* USARTx, uint8_t USART_AutoCount);
//void USART_SetBlockLength(USART_TypeDef* USARTx, uint8_t USART_BlockLength);

// IrDA mode functions ********************************************************
//void USART_IrDAConfig(USART_TypeDef* USARTx, uint32_t USART_IrDAMode);
//void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);

// RS485 mode functions *******************************************************
//void USART_DECmd(USART_TypeDef* USARTx, FunctionalState NewState);
//void USART_DEPolarityConfig(USART_TypeDef* USARTx, uint32_t USART_DEPolarity);
//void USART_SetDEAssertionTime(USART_TypeDef* USARTx, uint32_t USART_DEAssertionTime);
//void USART_SetDEDeassertionTime(USART_TypeDef* USARTx, uint32_t USART_DEDeassertionTime);

// DMA transfers management functions *****************************************/
//void USART_DMACmd(USART_TypeDef* USARTx, uint32_t USART_DMAReq, FunctionalState NewState);
//void USART_DMAReceptionErrorConfig(USART_TypeDef* USARTx, uint32_t USART_DMAOnError);

/* Interrupts and flags management functions **********************************/
void USART_ITConfig(USART_TypeDef* USARTx, uint32_t USART_IT, FunctionalState NewState);
void USART_RequestCmd(USART_TypeDef* USARTx, uint32_t USART_Request, FunctionalState NewState);
void USART_OverrunDetectionConfig(USART_TypeDef* USARTx, uint32_t USART_OVRDetection);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint32_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint32_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint32_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint32_t USART_IT);

#ifdef __cplusplus
}
#endif

#endif


/*----------------------------------END OF FILE------------------------------------*/































