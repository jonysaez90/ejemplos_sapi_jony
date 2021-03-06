/* Copyright 2014, 2015, 2016 Mariano Cerdeiro
 * Copyright 2014, Gustavo Muro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Fernando Beunza
 * All rights reserved.
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

/** \brief ADC DAC example source file
 **
 ** This is a mini example of the CIAA Firmware
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup ADC_DAC ADC & DAC example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * GMuro        Gustavo Muro
 * PR           Pablo Ridolfi
 * JuCe         Juan Cecconi
 * FB           Fernando Beunza
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20140805 v0.0.1   GMuro first functional version
 */

/*==================[inclusions]=============================================*/
#include "os.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaak.h"            /* <= ciaa kernel header */
#include "adc_dac_mod.h"
#include "chip.h"
//#include "chip_lpc18xx.h"
//#include "chip_lpc43xx.h"
//#include "adc_18xx_43xx.h"
//#include "sAPI_AnalogIO.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/** \brief File descriptor for ADC
 *
 * Device path /dev/serial/aio/in/0
 */
//static int32_t fd_adc;

/** \brief File descriptor for DAC
 *
 * Device path /dev/serial/aio/out/0
 */
//static int32_t fd_dac;


/*==================[external data definition]===============================*/

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
   /* init the ciaa kernel */
   ciaak_start();

   /* open CIAA ADC
   fd_adc = ciaaPOSIX_open("/dev/serial/aio/in/0", ciaaPOSIX_O_RDONLY);
   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);
   ciaaPOSIX_ioctl(fd_adc, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_3);

   /* open CIAA DAC
   fd_dac = ciaaPOSIX_open("/dev/serial/aio/out/0", ciaaPOSIX_O_WRONLY);
   ciaaPOSIX_ioctl(fd_dac, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);

   /* end InitTask */
   	   	   	   	   	   	   	   	   //ENABLE IMPUTS
   ADC_CLOCK_SETUP_T ADCSetup;
   /* Initialized to default values:
   		   *   - Sample rate:ADC_MAX_SAMPLE_RATE=400KHz
   		   *   - resolution: ADC_10BITS
   		   *   - burst mode: DISABLE */
	Chip_ADC_Init( LPC_ADC0, &ADCSetup );
	/* Disable burst mode */
	Chip_ADC_SetBurstCmd( LPC_ADC0, DISABLE );
	/* Set sample rate to 200KHz */
	Chip_ADC_SetSampleRate( LPC_ADC0, &ADCSetup, ADC_MAX_SAMPLE_RATE/2 );
	/* Disable all channels */
	Chip_ADC_EnableChannel( LPC_ADC0,ADC_CH1, DISABLE );
	Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH1, DISABLE );

	Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH2, DISABLE );
	Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH2, DISABLE );

	Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH3, DISABLE );
	Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH3, DISABLE );

	Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH4, DISABLE );
	Chip_ADC_Int_SetChannelCmd( LPC_ADC0, ADC_CH4, DISABLE );


   TerminateTask();
}

/** \brief Read ADC values
 *
 * This task is activated every 1ms by the AnalogicAlarm configured in
 * adc_dac.oil
 */
TASK(Analogic2)
{
   /*int32_t count;
   uint16_t hr_ciaaDac[128];

   Read ADC.
   count = ciaaPOSIX_read(fd_adc, &hr_ciaaDac, sizeof(hr_ciaaDac));

   if (count > 0)
   {
      int32_t i;

      for(i = 0; i < (count/2); i++) {
         /* signal processing. */
         /* e.g. duplicating the singal level
         hr_ciaaDac[i] <<= 1;
      }

      /* Write DAC
      ciaaPOSIX_write(fd_dac, &hr_ciaaDac, count);
   }

   /* end of Blinking */
	//uint8_t lpcAdcChannel = 66 ; //modificado para tomar desde A0
	uint16_t analogValue = 0;
	float dato=0;
	 /* typedef enum CHIP_ADC_CHANNEL {
			ADC_CH0 = 0,	< ADC channel 0
				ADC_CH1,		/**< ADC channel 1
				ADC_CH2,		/**< ADC channel 2
				ADC_CH3,		/**< ADC channel 3
				ADC_CH4,		/**< ADC channel 4
				ADC_CH5,		/**< ADC channel 5
				ADC_CH6,		/**< ADC channel 6
				ADC_CH7,		/**< ADC channel 7
			} ADC_CHANNEL_T;*/

		//Chip_ADC_EnableChannel(LPC_ADC0, lpcAdcChannel, ENABLE);
		Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH0, ENABLE);
		Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		while( (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH0, ADC_DR_DONE_STAT) != SET) );

		Chip_ADC_ReadValue( LPC_ADC0, ADC_CH0, &analogValue );
		dato= (5*analogValue/1023);
		ciaaPOSIX_printf ("El valor de adc es:%d\r\n",analogValue);
		ciaaPOSIX_printf ("El valor de tension es:%f\r\n",dato);
		Chip_ADC_EnableChannel( LPC_ADC0, ADC_CH0, DISABLE );

	//ciaaPOSIX_printf(&analogValue);

   TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

