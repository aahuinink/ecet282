/*
 * AudioEffects.h
 *
 *  Created on: 11 дек. 2018 г.
 *      Author: ASUS
 */
#include <stddef.h>
#include <stdint.h>
#ifndef AUDIOEFFECTS_H_
#define AUDIOEFFECTS_H_

/* %%%%%%%%%%%%%%%%%%%   Delay Digital  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 *
 *El efecto es similar al delay TC Electronic 2290. La idea es obtener repeticiones fieles a la senial original
 *
 *sin ninguna alteracion en las repeticiones. La ecuacion del efecto es:
 *
 * Y[n] = X[n] + SUM((X[n - kT])/2^k)
 *
 * Las muestras de tiempos pasados se van guardando en un buffer que debe ser declarado por
 * el usuario.
 *
 *  void *ptr  : = es un puntero al buffer donde voy a almacenar las muestras.
 	 	 	 	 	 la memoria debe ser reservada por el usuario

size_t delay_time : = tiempo de delay entre la muestra original y las repeticiones del efecto.
					se expresa en muestras y esta relacionado con la frecuencia de muestreo,
					para  Fs = 48k delay_time_ms = delay_time/48k

uint8_t gain := valor por el cual divido la amplitud de las repeticiones. Tiene que ser potencia de 2,
				se utiliza bit-shifting X[n - kT] >> gain

size_t bufferLen :=  longitud del buffer al que apunta el *ptr

*/
typedef struct DelayDSP{

	uint16_t *ptr;

	size_t delay_time;

	size_t s_index;

	size_t bufferLen;

	uint8_t gain;

}DSP_Delay_t;

/* inicializo una estructura del tipo Delay_t con un tiempo de delay y un buffer */

/* %%%%%%%%%%%%%%%%%%%   Primitivas para el manejo del efecto %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

void DSP_DelayInit(DSP_Delay_t *DelayDSP, size_t dTime, uint16_t *buffer, size_t buffer_len, uint8_t gain);

void DSP_DelayChangeDelayTime(DSP_Delay_t *DelayDSP, size_t dTime);

void DSP_DelayCleanBuffer(DSP_Delay_t *DelayDSP);

uint16_t DSP_DelaySample(DSP_Delay_t *DelayDSP, uint16_t sample);



#endif /* AUDIOEFFECTS_H_ */
