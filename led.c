#include "MKL46Z4.h"


// mac dinh cac chan thanh ghi
#define PORTC_D_IRQ_NBR (IRQn_Type) 31 // Thanh ghi ngat o Port D va C la 31
#define Sw1_pin (3)
#define Sw2_pin (12)
#define green_led_pin				 (1<<5)
#define red_led_pin 				 (1<<29)

// cac ham dieu khien LED
#define GREEN_LED_TOGGLE()   PTD->PTOR |= green_led_pin;
#define RED_LED_TOGGLE()   	 PTE->PTOR |= red_led_pin;
#define GREEN_LED_OFF()      PTD->PSOR |= green_led_pin;
#define RED_LED_OFF()        PTE->PSOR |= red_led_pin;
#define GREEN_LED_ON()       PTD->PCOR |= green_led_pin;
#define RED_LED_ON()         PTE->PCOR |= red_led_pin;

// khoi tao cac chan
void initLed(void);
void initSwitch(void);
void init_systick_interupt(void);

// khoi tao cac ham
void PORTC_PORTD_IRQHandler(void);
void reset_system_state(void);

// khai bao cac bien toan cuc
int system_active = 0;
int32_t volatile msTicks = 0;	

int main()
{
	initLed();
	initSwitch();
	init_systick_interupt();
	while(1){
		
	}
	return 0;
}

// khoi tao cac chan led do va xanh
void initLed(void)
{
		// Thiet lap led xanh
		SIM->SCGC5 |= (1 << 12);   // Cap clock cho Port D
		PORTD->PCR[5] = (1 << 8);  // Thiet lap chan 5 la GPIO
		PTD->PDDR |= (1 << 5);		 // Thiet lap la output
	
		// Thiet lap led do
		SIM->SCGC5 |= (1 << 13);   // Cap clock cho Port E
		PORTE->PCR[29] = (1 << 8);  // Thiet lap chan 29 la GPIO
		PTE->PDDR |= (1 << 29);		 // Thiet lap la output
}

// khoi tao 2 switch
void initSwitch(void)
{
	// PortC pin3, input pullup - khoi tao sw1 va 2
	SIM->SCGC5 |= (1 << 11); // Cap clock cho sw1
	PORTC->PCR[Sw1_pin] = (1 << 8) | (1 << 1) | (1 << 0);  // MUX(1) | PE | PS - kich hoat theo suon xung xuong sw1
	PTC->PDDR &= ~((uint32_t)(1u<< Sw1_pin)); // Pin3 la input - sw1 output
	PORTC->PCR[Sw2_pin] = (1 << 8) | (1 << 1) | (1 << 0);  // MUX(1) | PE | PS - kich hoat theo suon xung xuong sw2
	PTC->PDDR &= ~((uint32_t)(1u<< Sw2_pin)); // Pin12 la input - sw2 output
	
	// cho phep interupt cua switch
	PORTC->PCR[Sw2_pin] |= PORT_PCR_IRQC(0xA); // thiet lap ngat suon xuong cho PORTC pin3
	PORTC->PCR[Sw1_pin] |= PORT_PCR_IRQC(0xA); // thiet lap ngat suon xuong cho PORTC pin3
	NVIC_ClearPendingIRQ(PORTC_D_IRQ_NBR); // xoa cac interupt request dang o C va D
	NVIC_EnableIRQ(PORTC_D_IRQ_NBR); // cho phep interupt request o C va D
}
//init Systick
void init_systick_interupt(void) {
    SysTick->LOAD = 24000000 - 1;  // Set reload register
    SysTick->VAL = 0;                      // Reset the SysTick counter value
    SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0);  // Enable SysTick, enable interrupt, use processor clock
}

// SysTick Handler - cho den chay luan phien
void SysTick_Handler(void) {
	msTicks++;
  if (system_active == 1) {
			RED_LED_OFF();
			GREEN_LED_TOGGLE(); // Toggle green LED
			for (int i = 0; i < 24000; i++);
  } else {
			GREEN_LED_OFF();
      RED_LED_TOGGLE();   // Toggle red LED
  }
}

// Hàm reset trang thai
void reset_system_state(void) {
    system_active = 1; // dat system dang hoat dong
		GREEN_LED_OFF(); // Tt dèn LED xanh
    RED_LED_OFF(); // tat den LED do
	
    // Kh?i t?o l?i các bi?n, thi?t b? ngo?i vi, v.v. tùy theo yêu c?u
	
}
	
// PORTC interrupt handler for switches
void PORTC_PORTD_IRQHandler(void) {
	for(int i = 0; i < 1000; i++){};
  if (PORTC->ISFR & (1U << Sw1_pin)) {
        system_active = !system_active;
        PORTC->ISFR = (1U << Sw1_pin); // xoa co
    }
    if (PORTC->ISFR & (1U << Sw2_pin)) {
        reset_system_state(); // ham reset trang thai
        PORTC->ISFR = (1U << Sw2_pin); // xoa co
    }
}