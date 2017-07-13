/* Copyright 2015, Ocampos Artero Martin Eduardo Del Valle
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

/** \brief Blinking Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*==================[inclusions]=============================================*/

                                      /*Description*/

/*Debido a que el microcontrolador EDU CIAA no permite trabajar con los tres ADC en simultaneo, tenemos que configurar,
 * iniciar y leer los datos, una vez por cada uno
 *
 * Vamos a leer los pitch, roll,yaw, del sensor analogico MMA7361 */


#include "Template.h"       /* <= own header */
#include "salida.h"
#include "i2c.h"
#include "ADC.h"
#include "entrada.h"
#include "math.h"
#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

#define VREF 3.3 // Tension (v)
#define BITS 1024.0
#define SENS1 1.5 // 1.5 g
#define SENS2 6   // 6 g
#define CEROG 1.65 // Tension cuando esta 0 g
#define RAD_TO_DEG 57.2957786

int main(void)
{
/* Definimos funciones, variables, etc*/

      initsalida();
      iniciar_entrada();

      double Valx=0;
      double Valy=0;
      double Valz=0;

      double pitch=0;  /* Mide angulos y--z  */
      double roll=0;   /* Mide angulos x--z  */
      double yaw=0;    /* Mide angulos x--y  */



	while(1){

		/* Verificamos si el sensor esta en caida libre*/

		if(test_entrada(GPIO0)!=0)
			encendersalida(LED1);


	/*Comenzamos a config e iniciar el ADC*/

		config_ADC(ADC_ID0, channel1);                      /* ADC ID 0, CHANNEL 1*/
		init_ADC(ADC_ID0, channel1);

		/*Colocamos el pin del sensor ( G-Select) en LOW*/
		/*Config a 1.5g ( Mayor precisiòn)*/

		apagarsalida(GPIO1);

		/* Pin (SELF-TEST) lo colocamos en LOW*/

		apagarsalida(GPIO2);

		/* Leemos y desactivamos el ADC*/

		Valx=(VREF/BITS)*leer_ADC(ADC_ID0, channel1);
		Desactive_ADC(ADC_ID0);

		/*Comenzamos a config e iniciar el ADC*/

		config_ADC(ADC_ID0, channel3);
		init_ADC(ADC_ID0, channel3);

		/*Colocamos el pin del sensor ( G-Select) en LOW*/
     	/*Config a 1.5g ( Mayor precisiòn)*/

		apagarsalida(GPIO1);

		/* Pin (SELF-TEST) lo colocamos en LOW*/

		apagarsalida(GPIO2);

		/* Leemos y desactivamos el ADC*/

		Valz=(VREF/BITS)*leer_ADC(ADC_ID0, channel3);
	    Desactive_ADC(ADC_ID0);

		      roll=(atan2(-Valx, -Valz) + M_PI);
              roll=RAD_TO_DEG * roll;

 /////////////////////////////////////////////////////////////////////////////////////////////////////////////

              /*Comenzamos a config e iniciar el ADC*/

              		config_ADC(ADC_ID0, channel2);  /* ADC ID 0, CHANNEL 2*/
              		init_ADC(ADC_ID0, channel2);

              		/*Colocamos el pin del sensor ( G-Select) en LOW*/
              		/*Config a 1.5g ( Mayor precisiòn)*/

              		apagarsalida(GPIO1);

              		/* Pin (SELF-TEST) lo colocamos en LOW*/

              		apagarsalida(GPIO2);

              		/* Leemos y desactivamos el ADC*/

              		Valy=(VREF/BITS)*leer_ADC(ADC_ID0, channel2);
              		Desactive_ADC(ADC_ID0);

              		/*Comenzamos a config e iniciar el ADC*/

              		config_ADC(ADC_ID0, channel1);  /* ADC ID 0, CHANNEL 1*/
              		init_ADC(ADC_ID0, channel1);

              		/*Colocamos el pin del sensor ( G-Select) en LOW*/
                   	/*Config a 1.5g ( Mayor precisiòn)*/

              		apagarsalida(GPIO1);

              		/* Pin (SELF-TEST) lo colocamos en LOW*/

              		apagarsalida(GPIO2);

              		/* Leemos y desactivamos el ADC*/

              		Valx=(VREF/BITS)*leer_ADC(ADC_ID0, channel1);
              	    Desactive_ADC(ADC_ID0);

              		      yaw=(atan2(-Valy, -Valx) + M_PI);
                          yaw=RAD_TO_DEG * yaw;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

                          /*Comenzamos a config e iniciar el ADC*/

                          config_ADC(ADC_ID0, channel2);  /* ADC ID 0, CHANNEL 2*/
                          init_ADC(ADC_ID0, channel2);

                          /*Colocamos el pin del sensor ( G-Select) en LOW*/
                          /*Config a 1.5g ( Mayor precisiòn)*/

                          apagarsalida(GPIO1);

                          /* Pin (SELF-TEST) lo colocamos en LOW*/

                          apagarsalida(GPIO2);

                          /* Leemos y desactivamos el ADC*/

                          Valy=(VREF/BITS)*leer_ADC(ADC_ID0, channel2);
                          Desactive_ADC(ADC_ID0);

                          	/*Comenzamos a config e iniciar el ADC*/

                          config_ADC(ADC_ID0, channel3);  /* ADC ID 0, CHANNEL 3*/
                          init_ADC(ADC_ID0, channel3);

                          /*Colocamos el pin del sensor ( G-Select) en LOW*/
                          /*Config a 1.5g ( Mayor precisiòn)*/

                          	apagarsalida(GPIO1);

                          	/* Pin (SELF-TEST) lo colocamos en LOW*/

                          	apagarsalida(GPIO2);

                          	/* Leemos y desactivamos el ADC*/

                          	Valz=(VREF/BITS)*leer_ADC(ADC_ID0, channel3);
                            Desactive_ADC(ADC_ID0);

                             pitch=(atan2(-Valy, -Valz) + M_PI);
                             pitch=RAD_TO_DEG * pitch;



	}

    return 0;
}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

