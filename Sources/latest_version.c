* main implementation: use this 'C' sample to create your own application
 *
 */

#include "MKL25Z4.h"
#include "math.h"
#include <stdint.h>
//defines mayuscula nombres funciones y variables ingles y camel case
#define red_on()    GPIOB_PDOR &= ~((uint32_t)(1 << 18))
#define red_off()   GPIOB_PDOR |=  ((uint32_t)(1 << 18))

#define green_on()  GPIOB_PDOR &= ~((uint32_t)(1 << 19))
#define green_off() GPIOB_PDOR |=  ((uint32_t)(1 << 19))

#define blue_on()   GPIOD_PDOR &= ~((uint32_t)(1 << 1))
#define blue_off()  GPIOD_PDOR |=  ((uint32_t)(1 << 1))
void initPwm() {
	SIM_BASE_PTR ->SCGC6 |= SIM_SCGC6_TPM2_MASK; // ENABLE TPM2 CLOCK GATE
	SIM_BASE_PTR ->SOPT2 |= SIM_SOPT2_TPMSRC(3); // MCGIRCLK IS SELECTED FOR TPM CLOCK
	TPM2_BASE_PTR ->SC |= TPM_SC_PS(5); // TODO especificar frequencia
	TPM2_BASE_PTR ->SC |= TPM_SC_CMOD(1); // COUNTER INC. ON EVERY CLOCK
	TPM2_BASE_PTR ->MOD = 62500; // TODO especificar frequencia
	SIM_BASE_PTR ->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_BASE_PTR ->PCR[2] = PORT_PCR_MUX(3); // TODO especificar multiplexacio del TPM2_CH0
	TPM2_BASE_PTR ->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // SELECT
	TPM2_BASE_PTR ->CONTROLS[0].CnV = TPM2_BASE_PTR ->MOD / 2; // TODO especificar duty cycle
}

void initLeds() {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	//red
	PORTB_PCR18 |= PORT_PCR_MUX(1);
	GPIOB_PDDR |= 0b1 << 18;
	GPIOB_PSOR |= 0b1 << 18;
	//green
	PORTB_PCR19 |= PORT_PCR_MUX(1);
	GPIOB_PDDR |= 0b1 << 19;
	GPIOB_PSOR |= 0b1 << 19;
	//blue
	PORTD_PCR1 |= PORT_PCR_MUX(1);
	GPIOD_PDDR |= 0b1 << 1;
	GPIOD_PSOR |= 0b1 << 1;
}

void cambioPotencia(int potencia) {
	switch (potencia) {
	case 0:
		TPM2_BASE_PTR ->CONTROLS[0].CnV = TPM2_BASE_PTR ->MOD * 0;
		break;
	case 1:
		TPM2_BASE_PTR ->CONTROLS[0].CnV = TPM2_BASE_PTR ->MOD * 0.05;
		break;
	case 2:
		TPM2_BASE_PTR ->CONTROLS[0].CnV = TPM2_BASE_PTR ->MOD * 0.1;
		break;
	case 3:
		TPM2_BASE_PTR ->CONTROLS[0].CnV = TPM2_BASE_PTR ->MOD * 0.15;
		break;
	case 4:
		TPM2_BASE_PTR ->CONTROLS[0].CnV = TPM2_BASE_PTR ->MOD * 0.20;
		break;
	}
}
void delay() {
	int i = 0;
	for (; i < 10000000; i++) {

	}
}

void mostrarLeds(int potencia) {
	switch (potencia) {
	case 0:
		red_off();
		green_off();
		blue_off();
		break;
	case 1:
		green_on();
		delay();
		green_off();
		break;
	case 2:
		red_on();
		blue_on();
		delay();
		red_off();
		blue_off();
		break;
	case 3:
		red_on();
		green_on();
		delay();
		red_off();
		green_off();
		break;
	case 4:
		red_on();
		green_on();
		blue_on();
		delay();
		red_off();
		green_off();
		blue_off();
		break;
	}
}

void ventilador() {
	//comprobar señal de encendido del mando
	int encendido = 1;
	int apagado = 0;
	int pulsador_subida = 0;
	int pulsador_bajada = 0;
	if (encendido == 1) {
		int potencia = 1;
		while (apagado != 1) {
			if (pulsador_subida == 1 && potencia < 4) {
				potencia += 1;
				cambioPotencia(potencia);
				mostrarLeds(potencia);
			}
			if (pulsador_bajada == 1 && potencia > 1) {
				potencia -= 1;
				cambioPotencia(potencia);
				mostrarLeds(potencia);
			}
		}
	}
}

void initClock() {
	MCG_BASE_PTR ->C1 = MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK; // INTERNAL CLOCK|MCGIRCLK ACTIVE(SET)
	MCG_BASE_PTR ->C2 = MCG_C2_IRCS_MASK; // SELECT FAST INTERNAL REFERENCE CLOCK (1)
	return;
}

void initSpi() {
	SIM_BASE_PTR ->SCGC4 = SIM_SCGC4_SPI0_MASK; // Enable SPI0 clock

	SPI0_BASE_PTR ->C1 = SPI_C1_SPE_MASK;  // enable, SPI System Enable, SPE
	SPI0_BASE_PTR ->C1 = SPI_C1_MSTR_MASK; // use SPI as master, Master Slave select, MSTR
	SPI0_BASE_PTR ->C1 = SPI_C1_CPHA_MASK; // clock phase at the start, CPHA

	SPI0_BASE_PTR ->C2 = SPI_C2_SPMIE_MASK; // enable interrupt, SPMIE

	SPI0_BASE_PTR ->BR = SPI_BR_SPPR(0); // select BR prescaler divisor to 1, SPPR
	SPI0_BASE_PTR ->BR = SPI_BR_SPR(0);  // select BR divisor to 1, SPR

	return;
}


uint8_t isSpiDataFull() {
	return SPI0_BASE_PTR ->S & SPI_S_SPRF_MASK; // data available in buffer
}

uint8_t readSpiBuffer() {
	return SPI0_BASE_PTR ->D; // read Spi Data register
}

int main(void) {
	initClock();

	initLeds();
	initPwm();
	initSpi();
	
	mostrarLeds(isSpiDataFull());
	cambioPotencia(isSpiDataFull());
	delay();
	return 0;
}
