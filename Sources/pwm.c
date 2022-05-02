/*
 * main implementation: use this 'C' sample to create your own application
 *
 */
//#include "derivative.h" 
#include "MKL25Z4.h"
#include "math.h"
void initPwm()
{
	MCG_BASE_PTR->C1 = MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK; // INTERNAL CLOCK|MCGIRCLK
	MCG_BASE_PTR->C2 = MCG_C2_IRCS_MASK; // SELECT FAST INTERNAL REFERENCE CLOCK (1)
	SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK; // ENABLE TPM2 CLOCK GATE
	SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(3); // MCGIRCLK IS SELECTED FOR TPM CLOCK
	TPM2_BASE_PTR->SC |= TPM_SC_PS(5); // TODO especificar frequencia
	TPM2_BASE_PTR->SC |= TPM_SC_CMOD(1); // COUNTER INC. ON EVERY CLOCK
	TPM2_BASE_PTR->MOD = 62500; // TODO especificar frequencia
	SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_BASE_PTR->PCR[2] = PORT_PCR_MUX(3); // TODO especificar multiplexacio del TPM2_CH0
	TPM2_BASE_PTR->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // SELECT
	TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD / 2; // TODO especificar duty cycle
}
void cambioPotencia(int potencia)
{
	switch(potencia)
	{
		case 1:
			TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.05;
		case 2:
			TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.1;
		case 3:
			TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.15;
		case 4:
			TPM2_BASE_PTR->CONTROLS[0].CnV = TPM2_BASE_PTR->MOD * 0.20;
	}
}
void delay(int num)
{
	int i =0;
	for(; i< num; i++)
	{
		
	}
}

void ventilador()
{
	//comprobar señal de encendido del mando
	int señal_encendido = 1;
	int apagado_señal = 0;
	int pulsador_subida = 0;
	int pulsador_bajada = 0;
	if( señal_encendido == 1)
	{
		int potencia = 1;
		while(apagado_señal != 1)
		{
			if(pulsador_subida == 1 && potencia < 4)
			{
				potencia += 1;
				cambioPotencia(potencia);
			}
			if(pulsador_bajada == 1 && potencia > 1)
			{
				potencia -= 1;
				cambioPotencia(potencia);
			}
		}
	}
}
int main(void)
{
	int potencia = 1;
	initPwm();
	while(1){
		if(potencia <= 4)
		{
			cambioPotencia(potencia);
			delay(10000000);
			potencia += 1;
		}
		else
		{
			potencia = 1;
		}
		
	}
	return 0;
}