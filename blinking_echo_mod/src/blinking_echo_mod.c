/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking_echo example source file
 **
 ** This is a mini example of the CIAA Firmware to test the periodical
 ** task excecution and serial port funcionality.
 ** To run this sample in x86 plataform you must enable the funcionality of
 ** uart device setting a value of une or more of folowing macros defined
 ** in header file modules/plataforms/x86/inc/ciaaDriverUart_Internal.h
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Blinking Blinking_echo example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * PR           Pablo Ridolfi
 * JuCe         Juan Cecconi
 * GMuro        Gustavo Muro
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141019 v0.0.2   JuCe add printf in each task,
 *                        remove trailing spaces
 * 20140731 v0.0.1   PR   first functional version
 */

/*==================[inclusions]=============================================*/
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "blinking_echo_mod.h"         /* <= own header */
#include "chip.h"
//#include "cr_startup_lpc43xx.c"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
/** \brief File descriptor for digital input ports
 *
 * Device path /dev/dio/in/0
 */
static int32_t fd_in;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;

/** \brief File descriptor of the RS232 uart
 *
 * Device path /dev/serial/uart/2
 */
static int32_t fd_uart2;

/** \brief Periodic Task Counter
 *
 */
static uint32_t Periodic_Task_Counter;

/*==================[external data definition]===============================*/
uint8_t data;
/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */



uint8_t leerUART(void){
	uint8_t receivedByte = 1;
	if (Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_RDR){
		receivedByte = Chip_UART_ReadByte(LPC_USART2);
	}
	return receivedByte;
}




/*OSEK_ISR_UART2_IRQHandler(){
	uint8_t byte;
	byte=leerUART();
	while ((Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_THRE) == 0); /* Wait for space in FIFO */
//	   Chip_UART_SendByte(LPC_USART2, byte);
//}
ISR(UART2_IRQHandler)
{
	uint8_t byte;
	byte=leerUART();
	data=byte;
	while ((Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_THRE) == 0); /* Wait for space in FIFO */{
		//data=byte;
		Chip_UART_SendByte(LPC_USART2, byte);
	}
}



int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /* init CIAA kernel and devices */
   ciaak_start();

   ciaaPOSIX_printf("Init Task...\n");
   /* open CIAA digital inputs */
   	   	   	   fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);



   /*Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_SCU_PinMux(
				2,
				10,
				SCU_MODE_INACT | SCU_MODE_ZIF_DIS,
				SCU_MODE_FUNC0
			 );
	Chip_GPIO_SetDir( LPC_GPIO_PORT, 0, ( 1 << 14 ), 1 );
	Chip_GPIO_SetPinState( LPC_GPIO_PORT, 0, 14, 0);/*



	/* open CIAA digital outputs */
				fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

   /* open UART connected to USB bridge (FT2232) */
				//fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

	Chip_UART_Init(LPC_USART2);
	Chip_UART_SetBaud(LPC_USART2, 9600);  /* Set Baud rate */
	Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0); /* Modify FCR (FIFO Control Register)*/
	Chip_UART_TXEnable(LPC_USART2); /* Enable UART Transmission */
	Chip_SCU_PinMux(7, 1, MD_PDN, FUNC6);              /* P7_1,FUNC6: UART2_TXD */
	Chip_SCU_PinMux(7, 2, MD_PLN|MD_EZI|MD_ZI, FUNC6); /* P7_2,FUNC6: UART2_RXD */

	/* Enable UART Rx Interrupt */
	   Chip_UART_IntEnable(LPC_USART2, UART_IER_RBRINT ); /* Receiver Buffer Register Interrupt */
	   /* Enable UART line status interrupt */
	  Chip_UART_IntEnable(LPC_USART2, UART_IER_RLSINT ); /* LPC43xx User manual page 1118 */


   /* open UART connected to RS232 connector */
				//fd_uart2 = ciaaPOSIX_open("/dev/serial/uart/2", ciaaPOSIX_O_RDWR);

   /* change baud rate for uart usb */
				//ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

   /* change FIFO TRIGGER LEVEL for uart usb */
				//ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* activate example tasks */
   Periodic_Task_Counter = 0;
  // SetRelAlarm(ActivateBlinkeo, 200, 200);

   /* Activates the SerialEchoTask task */
   ActivateTask(SerialEchoTask);

   /* end InitTask */
   TerminateTask();
}

/** \brief Serial Echo Task
 *
 * This tasks waits for input data from fd_uart1 and writes the received data
 * to fd_uart1 and fd_uart2. This taks alos blinkgs the output 5.
 *
 */
TASK(SerialEchoTask)
{
   int8_t buf[20];   /* buffer for uart operation              */
   uint8_t outputs;  /* to store outputs status                */
   int32_t ret;      /* return value variable for posix calls  */

   ciaaPOSIX_printf("SerialEchoTask...\n");
   /* send a message to the world :) */
   char message[] = "Hi! :)\nSerialEchoTask: Waiting for characters...\n";
   ciaaPOSIX_write(fd_uart1, message, ciaaPOSIX_strlen(message));

   while(1)
   {
      /* wait for any character ... */
	   	   //ret = ciaaPOSIX_read(fd_uart1, buf, 20);

      //if(ret > 0)
      //{
         /* ... and write them to the same device */
         //ciaaPOSIX_write(fd_uart1, buf, ret);

         /* also write them to the other device */
         //ciaaPOSIX_write(fd_uart2, buf, ret);
      //}

      /* blink output 5 with each loop */
	  // uint8_t byte;
	   //	byte=leerUART();
	   //	if ((Chip_UART_ReadLineStatus(LPC_USART2)== 0)){; /* Wait for space in FIFO */
			//while((Chip_UART_ReadLineStatus(LPC_USART2) & UART_LSR_THRE)==0){
				//Chip_UART_SendByte(LPC_USART2, byte);
			//}
	   	//}
	   //uint8_t dato=1;
	     	  //dato=leerUART();  //problema cuando envio se dirige directamente a subrutina y se pierde valor
	     	  if(data=='0'){
	     		  if(Chip_GPIO_ReadPortBit( LPC_GPIO_PORT, 1, 12 )==TRUE){
	     		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 1, 12, FALSE);
	     		  }
	     		  else{
	     		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 1, 12, TRUE);
	     		  }
	     		  data=1;
	     	  }

	     	 if(data=='1'){
	     	 	     		  if(Chip_GPIO_ReadPortBit( LPC_GPIO_PORT, 0, 14 )==TRUE){
	     	 	     		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 0, 14, FALSE);
	     	 	     		  }
	     	 	     		  else{
	     	 	     		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 0, 14, TRUE);
	     	 	     		  }
	     	 	     		  data=1;
	     	 	     	  }
	     	 if(data=='2'){
	     		     	 	     		  if(Chip_GPIO_ReadPortBit( LPC_GPIO_PORT, 1, 11 )==TRUE){
	     		     	 	     		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 1, 11, FALSE);
	     		     	 	     		  }
	     		     	 	     		  else{
	     		     	 	     		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 1, 11, TRUE);
	     		     	 	     		  }
	     		     	 	     		  data=1;
	     		     	 	     	  }




   }
}

/** \brief Periodic Task
 *
 * This task is activated by the Alarm ActivatePeriodicTask.
 * This task copies the status of the inputs bits 0..3 to the output bits 0..3.
 * This task also blinks the output 4
 */
 //TASK(Blinkeo)
//{
   /*
    * Example:
    *    Read inputs 0..3, update outputs 0..3.
    *    Blink output 4
    */

   /* variables to store input/output status */
  /* uint8_t inputs = 0, outputs = 0;

   /* read inputs */
  // ciaaPOSIX_read(fd_in, &inputs, 1);

   /* read outputs */
  // ciaaPOSIX_read(fd_out, &outputs, 1);

   /* update outputs with inputs */
  // outputs &= 0xF0;
   //outputs |= inputs & 0x0F;*/

   /* blink */
   //outputs ^= 0x10;

   /* write */
  /* ciaaPOSIX_write(fd_out, &outputs, 1);

   /* Print Task info */
   /*Periodic_Task_Counter++;
   ciaaPOSIX_printf("Periodic Task: %d\n", Periodic_Task_Counter);

   uint8_t dato=1;
  	  dato=leerUART();
  	  if(dato=='0'){
  		  if(Chip_GPIO_ReadPortBit( LPC_GPIO_PORT, 1, 12 )==TRUE){
  		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 1, 12, FALSE);
  		  }
  		  else{
  		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 1, 12, TRUE);
  		  }
  	  }*/

  	 /*if(dato=='2'){
  	  		  if(Chip_GPIO_ReadPortBit( LPC_GPIO_PORT, 0, 14 )==TRUE){
  	  		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 0, 14, FALSE);
  	  		  }
  	  		  else{
  	  		  	   Chip_GPIO_SetPinState( LPC_GPIO_PORT, 0, 14, TRUE);
  	  		  }
  	  	  }

   /* end PeriodicTask */
  // TerminateTask();
//}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

