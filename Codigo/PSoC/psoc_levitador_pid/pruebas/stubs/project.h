/* project.h de PRUEBA: reemplaza las APIs que genera PSoC Creator para poder
 * compilar y simular el firmware en el PC. Mismos nombres y firmas. */
#ifndef PROJECT_H_STUB
#define PROJECT_H_STUB
#include <stdint.h>

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef void (*cyisraddress)(void);

#define CY_ISR(n)        void n(void)
#define CY_ISR_PROTO(n)  void n(void)
#define CyGlobalIntEnable do { } while (0)
#define CY_INT_SYSTICK_IRQN 15u
#define BCLK__BUS_CLK__HZ   24000000u

cyisraddress CyIntSetSysVector(uint8 n, cyisraddress f);
uint32 SysTick_Config(uint32 ticks);
void   CyDelayUs(uint16 us);

/* UART_RS485 */
void   UART_RS485_Start(void);
void   UART_RS485_ClearRxBuffer(void);
uint8  UART_RS485_GetRxBufferSize(void);
uint8  UART_RS485_ReadRxData(void);
void   UART_RS485_PutArray(const uint8 *s, uint8 n);

/* UART_DBG */
void   UART_DBG_Start(void);
void   UART_DBG_PutString(const char *s);
void   UART_DBG_PutChar(char c);
uint8  UART_DBG_GetRxBufferSize(void);
uint8  UART_DBG_ReadRxData(void);

/* Pines */
void   Pin_LED_Write(uint8 v);
void   Pin_TRIG_Write(uint8 v);
uint8  Pin_ECHO_Read(void);
uint8  Pin_ECHO_ClearInterrupt(void);

/* PWM, Timer, isr */
void   PWM_Fan_Start(void);
void   PWM_Fan_WriteCompare(uint16 c);
void   Timer_US_Start(void);
uint16 Timer_US_ReadCounter(void);
void   isr_echo_StartEx(cyisraddress f);
void   isr_echo_ClearPending(void);
void   isr_echo_Enable(void);
void   isr_echo_Disable(void);

#endif
