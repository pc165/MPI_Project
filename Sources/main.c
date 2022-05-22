#include "MKL25Z4.h"
#include "math.h"
#include <stdint.h>

//defines mayuscula nombres funciones y variables ingles y camel case
#define RED_ON GPIOB_PDOR &= ~((uint32_t)(1 << 18))
#define RED_OFF GPIOB_PDOR |= ((uint32_t)(1 << 18))

#define GREEN_ON GPIOB_PDOR &= ~((uint32_t)(1 << 19))
#define GREEN_OFF GPIOB_PDOR |= ((uint32_t)(1 << 19))

#define BLUE_ON GPIOD_PDOR &= ~((uint32_t)(1 << 1))
#define BLUE_OFF GPIOD_PDOR |= ((uint32_t)(1 << 1))

void initPwm() {
    SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK; // ENABLE TPM2 CLOCK GATE
    SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(3);
    // MCGIRCLK IS SELECTED FOR TPM CLOCK
    TPM2_BASE_PTR->SC |= TPM_SC_PS(5);
    // TODO especificar frequencia
    TPM2_BASE_PTR->SC |= TPM_SC_CMOD(1);
    // COUNTER INC. ON EVERY CLOCK
    TPM2_BASE_PTR->MOD = 62500; // TODO especificar frequencia
    SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB_BASE_PTR->PCR[2] = PORT_PCR_MUX(3);
    // TODO especificar multiplexacio del TPM2_CH0
    TPM2_BASE_PTR->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // SELECT
    TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD / 2;                  // TODO especificar duty cycle
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

void initSystemClock() {
    MCG_BASE_PTR->C1 = MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK; // INTERNAL CLOCK|MCGIRCLK ACTIVE(SET)
    MCG_BASE_PTR->C2 = MCG_C2_IRCS_MASK;                        // SELECT FAST INTERNAL REFERENCE CLOCK (1)
    return;
}

//https://learningmicro.wordpress.com/serial-communication-interface-using-uart/
void initUart() {
	SIM->SCGC5 |= SIM_SCGC5_PORTA(1);
	 
	PORTA_PCR1 |=  PORT_PCR_MUX(2); /* PTA1 as ALT2 (UART0) */
	PORTA_PCR2 |=  PORT_PCR_MUX(2); /* PTA2 as ALT2 (UART0) */
	// Select MCGFLLCLK as UART0 clock
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
	 
	// Enable UART0 Clock
	SIM->SCGC4 |= SIM_SCGC4_UART0(1);
	
	//Baud Rate = Baud Clock / ((OSR+1) * BR)
	//9600 = 48000000 / ((15 + 1) * BR).
	//BR = 48000000 / (16 * 9600) = 312 (0x138).
	
	// Configure Baud Rate as 9600
	UART0->BDL = 0x38;
	UART0->BDH = 0x1;
	
	// Configure Serial Port as 8-N-1
	// (8 data bits, No parity and 1 stop bit)
	UART0->C1  = 0x00;
	
	// Configure Tx/Rx Interrupts
	UART0->C2  |= UART_C2_TIE(0);  // Tx Interrupt disabled
	UART0->C2  |= UART_C2_TCIE(0); // Tx Complete Interrupt disabled
	UART0->C2  |= UART_C2_RIE(1);    // Rx Interrupt enabled
	 
	// Configure Transmitter/Receiever
	UART0->C2  |= UART_C2_TE(1);     // Tx Enabled
	UART0->C2  |= UART_C2_RE(1);     // Rx Enabled
	
	// Enable UART0 Interrupt
	__NVIC_EnableIRQ(UART0_IRQn);
	
}

void initSpi() {

    // Init SPI
    SIM_BASE_PTR->SCGC4 = SIM_SCGC4_SPI0_MASK; // Enable SPI0 clock

    SPI0_BASE_PTR->C1 = SPI_C1_SPE_MASK;  // enable, SPI System Enable, SPE
    SPI0_BASE_PTR->C1 = SPI_C1_MSTR_MASK; // use SPI as master, Master Slave select, MSTR
    SPI0_BASE_PTR->C1 = SPI_C1_CPHA_MASK; // clock phase at the start, CPHA

    SPI0_BASE_PTR->C2 = SPI_C2_SPMIE_MASK; // enable interrupt, SPMIE

    SPI0_BASE_PTR->BR = SPI_BR_SPPR(0); // select BR prescaler divisor to 1, SPPR
    SPI0_BASE_PTR->BR = SPI_BR_SPR(0);  // select BR divisor to 1, SPR

    // INIT SYSYEM CLOCK for port C
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Configure port to alternative 2 (SPI mode)
    PORTB_PCR4 |= PORT_PCR_MUX(2); // SPI0_PCS0
    PORTB_PCR5 |= PORT_PCR_MUX(2); // SPI0_SCK
    PORTB_PCR6 |= PORT_PCR_MUX(2); // SPI0_MOSI
    PORTB_PCR7 |= PORT_PCR_MUX(2); // SPI0_MISO

    return;
}

uint8_t isSpiDataFull() {
    return SPI0_BASE_PTR->S & SPI_S_SPRF_MASK; // data available in buffer
}

uint8_t readSpiBuffer() {
    return SPI0_BASE_PTR->D; // read Spi Data register
}

void cambioPotencia(int potencia) {
    switch (potencia) {
    case 0:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0;
        break;
    case 1:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.05;
        break;
    case 2:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.1;
        break;
    case 3:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.15;
        break;
    case 4:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.20;
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
        RED_OFF;
        GREEN_OFF;
        BLUE_OFF;
        break;
    case 1:
        GREEN_ON;
        delay();
        GREEN_OFF;
        break;
    case 2:
        RED_ON;
        BLUE_ON;
        delay();
        RED_OFF;
        BLUE_OFF;
        break;
    case 3:
        RED_ON;
        GREEN_ON;
        delay();
        RED_OFF;
        GREEN_OFF;
        break;
    case 4:
        RED_ON;
        GREEN_ON;
        BLUE_ON;
        delay();
        RED_OFF;
        GREEN_OFF;
        BLUE_OFF;
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

int main() {
    initSystemClock();
    initSpi();
    initLeds();
    while (1) {
        GREEN_ON;
        if (isSpiDataFull()) {
            int data = readSpiBuffer();
            if (data == 1) {
                RED_ON;
            }
        }
    }
}