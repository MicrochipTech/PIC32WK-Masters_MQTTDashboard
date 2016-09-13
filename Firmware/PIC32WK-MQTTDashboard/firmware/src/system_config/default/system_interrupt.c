/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

/* definitions for temperature sensor */
#define ADC_REF_VOLT                (float)3.3
#define ADC_MAX_READING             (float)4096        // bits
#define ADC_CONV_FACT               (float)(ADC_MAX_READING/ADC_REF_VOLT)   // bits/V
#define MCP9700_COEFF               (float)0.01         // V/degC
#define MCP9700_COEFF_SCALED        (int)(MCP9700_COEFF * ADC_CONV_FACT)   // bits/degC
#define MCP9700_0DEG_OUT            (float)0.52          // V @ 0degC
#define MCP9700_0DEG_OUT_SCALED     (int)(MCP9700_0DEG_OUT * ADC_CONV_FACT)   // bits/degC

void __ISR(_ADC_VECTOR, ipl6AUTO) _IntHandlerDrvAdc(void)
{
    if (true == DRV_ADC_SamplesAvailable((uint8_t)ADCHS_AN11)){
		xData.appPerData.potPosition = DRV_ADC_SamplesRead((uint8_t)ADCHS_AN11);
        xData.appPerData.potPosition=((xData.appPerData.potPosition * 100)/3936);
        xData.appPerData.potPosition=(xData.appPerData.potPosition>100)?100:xData.appPerData.potPosition;
    }
    
    if (true == DRV_ADC_SamplesAvailable((uint8_t)ADCHS_AN15)){
    	xData.appPerData.temperature = DRV_ADC_SamplesRead((uint8_t)ADCHS_AN15);
        xData.appPerData.temperature &=0xfC0;    /*ignore last 6 bits*/
        xData.appPerData.temperature=(xData.appPerData.temperature - MCP9700_0DEG_OUT_SCALED)/MCP9700_COEFF_SCALED;
    }
    
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
}


    
void __ISR(_TIMER_1_VECTOR, ipl2AUTO) IntHandlerDrvTmrInstance0(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr0);
}
    
void __ISR(_TIMER_2_VECTOR, ipl2AUTO) IntHandlerDrvTmrInstance1(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr1);
}
  
void __ISR(_UART1_TX_VECTOR, ipl3AUTO) _IntHandlerDrvUsartTransmitInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
}
void __ISR(_UART1_RX_VECTOR, ipl3AUTO) _IntHandlerDrvUsartReceiveInstance0(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart0);
}
void __ISR(_UART1_FAULT_VECTOR, ipl1AUTO) _IntHandlerDrvUsartErrorInstance0(void)
{
    DRV_USART_TasksError(sysObj.drvUsart0);
}
 
 

 

 

 

 

 
 
void __ISR(_SPI0_RX_VECTOR, ipl1AUTO) _IntHandlerSPIRxInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
void __ISR(_SPI0_TX_VECTOR, ipl1AUTO) _IntHandlerSPITxInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
void __ISR(_SPI0_FAULT_VECTOR, ipl1AUTO) _IntHandlerSPIFaultInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
 
void __ISR( _USB_VECTOR , IPL4AUTO)_IntHandler_USB_stub ( void )
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}




extern void mac_isr(unsigned int vector);
extern void timer_tick_isr(unsigned int param);
volatile unsigned int g_isTimerInterrupt_contionous = 0;
void __ISR(_RFMAC_VECTOR, ipl1AUTO0) _IntHandlerRfMacInstance0(void)
{
	mac_isr(1);
	IFS2bits.RFMACIF = 0; 
}

 void __ISR(_RFTM0_VECTOR, ipl1AUTO0) _IntHandlerRfTimer0Instance0(void)
 {
	timer_tick_isr(0);
	g_isTimerInterrupt_contionous ++;
	IFS2bits.RFTM0IF = 0; 
 } 
  
/*******************************************************************************
 End of File
*/

