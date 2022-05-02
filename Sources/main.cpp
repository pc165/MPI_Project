#include "derivative.h" /* include peripheral declarations */

void initClock() {
    MCG_BASE_PTR->C1 = MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK; // INTERNAL CLOCK|MCGIRCLK ACTIVE(SET)
    MCG_BASE_PTR->C2 = MCG_C2_IRCS_MASK;                        // SELECT FAST INTERNAL REFERENCE CLOCK (1)
    return;
}

void initSpi() {
    SIM_BASE_PTR->SCGC4 = SIM_SCGC4_SPI0_MASK; // Enable SPI0 clock

    SPI0_BASE_PTR->C1 = SPI_C1_SPE_MASK;  // enable, SPI System Enable, SPE
    SPI0_BASE_PTR->C1 = SPI_C1_MSTR_MASK; // use SPI as master, Master Slave select, MSTR
    SPI0_BASE_PTR->C1 = SPI_C1_CPHA_MASK; // clock phase at the start, CPHA

    SPI0_BASE_PTR->C2 = SPI_C2_SPMIE_MASK; // enable interrupt, SPMIE

    SPI0_BASE_PTR->BR = SPI_BR_SPPR(0); // select BR prescaler divisor to 1, SPPR
    SPI0_BASE_PTR->BR = SPI_BR_SPR(0);  // select BR divisor to 1, SPR

    return;
}

uint8_t isSpiDataFull() {
    return SPI0_BASE_PTR->S & SPI_S_SPRF_MASK; // data available in buffer
}

uint8_t isSpiTransmitEmpty() {
    return SPI0_BASE_PTR->S & SPI_S_SPMF_MASK; // SPI transmit buffer empty
}

uint8_t readSpiBuffer() {
    return SPI0_BASE_PTR->D; // read Spi Data register
}

int main() {
    initClock();
    initSpi();
    return 0;
}
