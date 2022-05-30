#include "derivative.h"
#include <stdint.h>

//defines mayuscula nombres funciones y variables ingles y camel case
#define RED_ON GPIOB_PDOR &= ~((uint32_t)(1 << 18))
#define RED_OFF GPIOB_PDOR |= ((uint32_t)(1 << 18))

#define GREEN_ON GPIOB_PDOR &= ~((uint32_t)(1 << 19))
#define GREEN_OFF GPIOB_PDOR |= ((uint32_t)(1 << 19))

#define BLUE_ON GPIOD_PDOR &= ~((uint32_t)(1 << 1))
#define BLUE_OFF GPIOD_PDOR |= ((uint32_t)(1 << 1))

//input from the comand
int input;

//power parameter
int power = 0;

void initPwm() {
    SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK; // ENABLE TPM2 CLOCK GATE
    SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(3); 
    // MCGIRCLK IS SELECTED FOR TPM CLOCK
    TPM2_BASE_PTR->SC |= TPM_SC_PS(5);
    
    TPM2_BASE_PTR->SC |= TPM_SC_CMOD(1);
    // COUNTER INC. ON EVERY CLOCK
    TPM2_BASE_PTR->MOD = 62500;
    SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB_BASE_PTR->PCR[2] = PORT_PCR_MUX(3);/* PTB2 as ALT3 (PWM) */
    
    TPM2_BASE_PTR->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // SELECT
    TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD / 2;
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

void UART0_IRQHandler(void) {
    if ((UART0_BASE_PTR->S1 & UART_S1_RDRF_MASK)) {
        uint8_t input2 = UART0_BASE_PTR->D & 0b00010111;
        input = input2;
        UART0_BASE_PTR->S1 |= UART0_S1_PF_MASK;
        UART0_BASE_PTR->S1 |= UART0_S1_FE_MASK;
        UART0_BASE_PTR->S1 |= UART0_S1_NF_MASK;
        UART0_BASE_PTR->S1 |= UART0_S1_IDLE_MASK;
        UART0_BASE_PTR->S1 |= UART0_S1_OR_MASK;
    }
    return;
}

//https://learningmicro.wordpress.com/serial-communication-interface-using-uart/
void initUart() {
    SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PORTA_PCR1 |= PORT_PCR_MUX(2); /* PTA1 as ALT2 (UART0) */
    PORTA_PCR2 |= PORT_PCR_MUX(2); /* PTA2 as ALT2 (UART0) */
    // Select MCGFLLCLK as UART0 clock
    SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_UART0SRC(1);

    // Enable UART0 Clock
    SIM_BASE_PTR->SCGC4 |= SIM_SCGC4_UART0_MASK;

    //Baud Rate = Baud Clock / ((OSR+1) * BR)
    //9600 = 48000000 / ((15 + 1) * BR).
    //BR = 48000000 / (16 * 9600) = 312 (0x138).

    // Configure Baud Rate as 9600
    UART0_BASE_PTR->BDL = 0x38;
    UART0_BASE_PTR->BDH = 0x1;

    // Configure Serial Port as 8-N-1
    // (8 data bits, No parity and 1 stop bit)
    UART0_BASE_PTR->C1 = 0x00;

    // Configure Tx/Rx Interrupts
    //UART0_BASE_PTR->C2 |= UART0_C2_TIE(0);  // Tx Interrupt disabled
    //UART0_BASE_PTR->C2 |= UART0_C2_TCIE(0); // Tx Complete Interrupt disabled
    UART0_BASE_PTR->C2 |= UART0_C2_RIE_MASK;    // Rx Interrupt enabled

    // Configure Transmitter/Receiever
    //UART0_BASE_PTR->C2 |= UART0_C2_TE_MASK; // Tx Enabled
    UART0_BASE_PTR->C2 |= UART_C2_RE_MASK;  // Rx Enabled

}

void setINT() {
    NVIC_BASE_PTR->ICPR = 1 << 12; // CLEAR INT
    NVIC_BASE_PTR->ISER = 1 << 12; // SET INT
}

void changePower(int pow) {
    switch (pow) {
    case 0:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0;
        break;
    case 1:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.1;
        break;
    case 2:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.4;
        break;
    case 3:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.7;
        break;
    case 4:
        TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 1;
        break;
    }
}
void delay() {
    int i = 0;
    for (; i < 10000000; i++) {
    }
}

void showLeds(int pow) {
    switch (pow) {
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

void fan() {
    if (input == 1) {
        power = 1;
        changePower(power);
        showLeds(power);
        while (input != 2) {
            if (input == 3 && power < 4) {
            	power += 1;
            	changePower(power);
                showLeds(power);
                input = 0;
            }
            if (input == 16 && power > 1) {
                power -= 1;
                changePower(power);
                showLeds(power);
                input = 0;
            }
        }
        power = 0;
        changePower(power);
        showLeds(power);
    }
}

int main() {
	/*
initSystemClock();
    initUart();
    initLeds();
    setINT();
    GREEN_ON;
    while(1){
    	if (input == 1){
    		RED_ON;
    	}
    	if (input == 2)
    	{
    		RED_OFF;
    	}
    	if (input == 3)
    	{
    	    GREEN_OFF;
    	}
    	if (input == 16)
    	{
    		GREEN_ON;
    	}
    	
    }
    initSystemClock();
    initUart();
    initLeds();
    setINT();
    changePower(power);
    showLeds(power);
    while(1){
    	fan();
    }
    */
	initSystemClock();
	initUart();
	//initPwm();
	initLeds();
	setINT();
	//changePower(power);
	//showLeds(power);
	while(1){
	}
}
