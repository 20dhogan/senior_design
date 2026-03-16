#include "jet_cutter.h"

//Globial Varialbes

//UART*******************************************************************************************
void USART_Init(USART_TypeDef * USARTx){
	//Disable USART
	USARTx->CR1 &= ~USART_CR1_UE;
	
	USARTx->CR1 &= ~USART_CR1_M;
	
	USARTx->CR2 &= ~USART_CR2_STOP;
	
	USARTx->CR1 &= ~USART_CR1_PCE;
	
	USARTx->CR1 &= ~USART_CR1_OVER8;
	
	USARTx->BRR = 0x8B; //baud rate 115200
	
	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	
	USARTx->CR1 |= USART_CR1_UE;
	
	//enable interrupt
	USARTx->CR1 |= USART_CR1_RXNEIE;
	
	while((USARTx->ISR & USART_ISR_TEACK) == 0);
	
	while((USARTx->ISR & USART_ISR_REACK) == 0);
}

void USART3_Init(void){	
	 //Set PB10 and PB11 to Alternate Function mode
	 GPIOB->MODER &= 0xFF0FFFFF;
	 GPIOB->MODER |= 0x00A00000; // Alternate function Mode

	 //code from text book page 551
	 GPIOB->AFR[1] |= (0x7 << (4*2)) | (0x7 << (4*3)); //seting pins 10&11 to AF 7
	
	 GPIOB->OSPEEDR |= 0xF<<(2*10);
	
	 GPIOB->PUPDR &= 0xFF0FFFFF;
	 GPIOB->PUPDR |= 0x00500000;
	
	 GPIOB->OTYPER &= ~(0x3<<10);
      
   RCC->CCIPR &= ~RCC_CCIPR_USART3SEL;
   RCC->CCIPR |= RCC_CCIPR_USART3SEL_0;
	
	 //Enable USART clk
	 RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	
	//To enable interupts
	USART3->CR1 |= USART_CR1_RXNEIE; //Receive register not empty interrupt
	NVIC_SetPriority(USART3_IRQn, 8);
	NVIC_EnableIRQ(USART3_IRQn);

	USART_Init(USART3);
}

uint8_t USART_Read(USART_TypeDef *USARTx){	
	while(!(USARTx->ISR & USART_ISR_RXNE));
	return USARTx->RDR;
}

void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes) { //is this just a generic uart_send function that we can use for uart 3 or uart 2?
	int i;
	
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
	// wait until TXE (TX empty) is set
	// Writing USART_DR automatically clears the TXE flag
		while (!(USARTx->ISR & USART_ISR_TXE)){};
			USARTx->TDR = (buffer[i] & 0x1FF);
	}
	
	while (!(USARTx->ISR & USART_ISR_TC)){}; // wait until TC bit is set
		USARTx->ISR &= ~USART_ISR_TC;
}