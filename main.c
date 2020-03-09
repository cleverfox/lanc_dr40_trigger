#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <math.h>

#include <atom.h>
#include <atomsem.h>
#include <atomqueue.h>
#include <atomtimer.h>

#include "hw.h"

static uint8_t idle_stack[256];
static uint8_t master_thread_stack[1024];
static ATOM_TCB master_thread_tcb;

static ATOM_QUEUE uart2_tx;
static uint8_t uart2_tx_storage[64];

static ATOM_QUEUE uart2_rx;
static uint8_t uart2_rx_storage[64];

static ATOM_QUEUE uart1_rx;
static uint8_t uart1_rx_storage[8];

static ATOM_QUEUE uart3_tx;
static uint8_t uart3_tx_storage[8];

void _fault(int, int, const char*);
int u_write(int file, char *ptr, int len);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)
void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    while(1){
    }
};

static ATOM_SEM ibusy;

void tim2_setup(void);
void tim2_setup(void) {
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral. */
	timer_reset(TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);

	/* Reset prescaler value. */
	timer_set_prescaler(TIM2, 1024);

	/* Enable preload. */
	timer_enable_preload(TIM2);

	/* Continous mode. */
	timer_continuous_mode(TIM2);

	/* Period (36kHz). */
	timer_set_period(TIM2, 65535);

	/* Disable outputs. */
	timer_disable_oc_output(TIM2, TIM_OC1);
	timer_disable_oc_output(TIM2, TIM_OC2);
	timer_disable_oc_output(TIM2, TIM_OC3);
	timer_disable_oc_output(TIM2, TIM_OC4);

	/* -- OC1 configuration -- */

	/* Configure global mode of line 1. */
	timer_disable_oc_clear(TIM2, TIM_OC1);
	timer_disable_oc_preload(TIM2, TIM_OC1);
	timer_set_oc_slow_mode(TIM2, TIM_OC1);
	timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);

	/* Set the capture compare vale for OC1. */
//	timer_set_oc_value(TIM2, TIM_OC1, 1000);

	/* ---- */

	/* ARR reload enable. */
	timer_disable_preload(TIM2);

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable commutation interrupt. */
	//timer_enable_irq(TIM2, TIM_DIER_CC1IE);
        if (atomSemCreate (&ibusy, 0) != ATOM_OK) 
            fault(3);
}

void idelay(uint32_t t);
void idelay(uint32_t t){
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
    timer_set_counter(TIM2, t);
    TIM_DIER(TIM2) |= TIM_DIER_UIE;
    //timer_enable_irq(TIM2, TIM_DIER_UIE);
    atomSemGet (&ibusy, 0);
}

void tim2_isr(void) {
    atomIntEnter();
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        //timer_clear_flag(TIM2, TIM_SR_UIF);
        TIM_SR(TIM2) &= ~TIM_SR_UIF;
        //timer_disable_irq(TIM2, TIM_DIER_UIE);
        TIM_DIER(TIM2) &= ~TIM_DIER_UIE; //disable interrupt
        atomSemPut (&ibusy);
    }
    atomIntExit(0);
}
void xcout(unsigned char c);
void icout(unsigned char c);
void i2cout(uint16_t c);

void xcout(unsigned char c){
    static char set[]="0123456789ABCDEF";
    char t[2]={set[(c>>4)&0x0f],set[c&0x0f]};
    u_write(1,t,2);
}

void x4cout(uint32_t c);
void x4cout(uint32_t c){
    static char set[]="0123456789ABCDEF";
    char t[8]={
        set[(c>>28)&0x0f],
        set[(c>>24)&0x0f],
        set[(c>>20)&0x0f],
        set[(c>>16)&0x0f],
        set[(c>>12)&0x0f],
        set[(c>>8)&0x0f],
        set[(c>>4)&0x0f],
        set[c&0x0f]
    };
    u_write(1,t,8);
}

void icout(unsigned char c){
    char t[3]="000";
    t[2]='0'+c%10;
    c/=10;
    t[1]='0'+c%10;
    c/=10;
    t[0]=c+'0';
    u_write(1,t,3);
}

void i2cout(uint16_t c){
    char t[5]="00000";
    t[4]='0'+c%10;
    c/=10;
    t[3]='0'+c%10;
    c/=10;
    t[2]='0'+c%10;
    c/=10;
    t[1]='0'+c%10;
    c/=10;
    t[0]=c+'0';
    u_write(1,t,5);
}

void usart1_isr(void) {
    static uint8_t data = 'A';
    static uint32_t pt;
    static uint8_t  cb=0;

    atomIntEnter();

    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        uint32_t t=timer_get_counter(TIM2);
        if(pt-t>0x100){
            cb=0;
        }
        data = usart_recv(USART1);
        if(cb==4) atomQueuePut(&uart1_rx,0, (void*) &data);
        cb++;
        pt=t;
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_TXE) != 0)) {
        //just disable int
        USART_CR1(USART1) &= ~USART_CR1_TXEIE;
    }
    atomIntExit(0);
}


void usart2_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();

    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART2);

        if(data=='\r' || data=='\n'){
            data='\r';
            u_write(1,(char*) &data, 1);
            data='\n';
            u_write(1,(char*) &data, 1);
        }else{
            atomQueuePut(&uart2_rx,0, (uint8_t*) &data);
        }

    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_TXE) != 0)) {
        uint8_t status = atomQueueGet(&uart2_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send(USART2, data);
        }else{
            USART_CR1(USART2) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}

void usart3_isr(void) {
    static uint8_t data = 'A';

    atomIntEnter();

    if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART3);
        //do not receive
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART3) & USART_SR_TXE) != 0)) {
        uint8_t status = atomQueueGet(&uart3_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send(USART3, data);
        }else{
            USART_CR1(USART3) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}



void rec(int recorstop);
static void master_thread(uint32_t args __maybe_unused) {
    u_write(1,(char *)"master thread\r\n",15);
    uint8_t state=0;
    uint8_t pstate=0;
    while(1){
        uint8_t data;
        uint8_t status = atomQueueGet(&uart1_rx, SYSTEM_TICKS_PER_SEC, (void*)&data);
        if(status == ATOM_OK){
            if(!data) continue;
            if(data!=pstate){
                pstate=data;
                xcout(data);
                u_write(1,(char *)"\r\n",2);
            }else{
                if(data!=state){
                    xcout(state);
                    u_write(1,(char *)"->",2);
                    xcout(data);
                    if(data & 0x10){
                        rec(1);
                        u_write(1,(char *)" REC",4);
                    }else{
                        rec(0);
                        u_write(1,(char *)" STOP",5);
                    }
                    u_write(1,(char *)"\r\n",2);
                    state=data;
                }
            }
        };
        status = atomQueueGet(&uart2_rx, -1, (void*)&data);
        if(status == ATOM_OK){
            if(data == 'r') {
                rec(1);
                //u_write(3,(char *)&cmd,1);
                u_write(1,(char *)"REC\r\n",5);
            }else if(data == 'p') {
                u_write(1,(char *)"PWR1\r\n",6);
                rec(-1);
                u_write(1,(char *)"PWR2\r\n",6);
             }else if(data == 's') {
                rec(0);
                //atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
                //u_write(3,(char *)&cmd,1);
                u_write(1,(char *)"STOP\r\n",6);
            }
        };
        //gpio_set(GPIOA, GPIO4|GPIO5);
        //atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    }
}

void rec(int recorstop){
    if(recorstop==1){
        usart_send(USART3, (uint8_t)0x8b);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
        usart_send(USART3, (uint8_t)0x0b);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
        usart_send(USART3, (uint8_t)0x8b);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
        usart_send(USART3, (uint8_t)0x0b);
    }else if(recorstop==0){
        usart_send(USART3, (uint8_t)0x88);
        atomTimerDelay(SYSTEM_TICKS_PER_SEC/10);
        usart_send(USART3, (uint8_t)0x08);
    }
}

int main(void) {
    rcc_clock_setup_in_hsi_out_48mhz();
    //rcc_clock_setup_in_hse_8mhz_out_24mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART3);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO4|GPIO5);
    gpio_clear(GPIOA, GPIO4);
    gpio_set(GPIOA, GPIO5);

    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, 0, GPIO15);

    usart_setup();
    tim2_setup();

    cm_mask_interrupts(true);
    systick_set_frequency(SYSTEM_TICKS_PER_SEC, 48000000);
    systick_interrupt_enable();
    systick_counter_enable();

    nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

    /*
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO6|GPIO7|GPIO8|GPIO9);
    gpio_set(GPIOC, GPIO6);
    gpio_set(GPIOC, GPIO7);
    gpio_clear(GPIOC, GPIO8);
    gpio_toggle(GPIOC, GPIO9);
    */

     

    gpio_set(GPIOA, GPIO15);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO8|GPIO9);

    /*
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_clear(GPIOA, GPIO8);
    */

    /* Button pin */

    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO1);
    gpio_set(GPIOC, GPIO1);


    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART3_TX);

    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');
    usart_send_blocking(USART2, 'U');
    usart_send_blocking(USART2, '2');
    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);

    gpio_set(GPIOA, GPIO8);

    if(atomOSInit(idle_stack, sizeof(idle_stack), FALSE) != ATOM_OK) 
        fault(1);

    if (atomQueueCreate (&uart2_rx, uart2_rx_storage, sizeof(uint8_t), sizeof(uart2_rx_storage)) != ATOM_OK) 
        fault(2);
    if (atomQueueCreate (&uart2_tx, uart2_tx_storage, sizeof(uint8_t), sizeof(uart2_tx_storage)) != ATOM_OK) 
        fault(3);
    if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), sizeof(uart1_rx_storage)) != ATOM_OK) 
        fault(4);
    if (atomQueueCreate (&uart3_tx, uart3_tx_storage, sizeof(uint8_t), sizeof(uart3_tx_storage)) != ATOM_OK) 
        fault(5);
    u_write(1,(char *)"USART2 ready\r\n",14);

    atomThreadCreate(&master_thread_tcb, 10, master_thread, 0,
            master_thread_stack, sizeof(master_thread_stack), TRUE);

    atomOSStart();
    fault(255);
}

int u_write(int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        switch(file){
            case 3: //to dr40
                atomQueuePut(&uart3_tx,0, (uint8_t*) &ptr[i]);
                break;
            case 1: //DEBUG
                atomQueuePut(&uart2_tx,0, (uint8_t*) &ptr[i]);
                break;
         }
    }
    switch(file){
        case 3:
            USART_CR1(USART3) |= USART_CR1_TXEIE;
            break;
        case 1:
            USART_CR1(USART2) |= USART_CR1_TXEIE;
            break;
    }
    return i;
}
