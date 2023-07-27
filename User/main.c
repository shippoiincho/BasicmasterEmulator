/*

 Hitachi BasicMaster Jr. (MB6885) Emulator for CH32V203
 You must use HSE 96MHz clock.
 (May be work on 48MHz by change SPI/Timer prescaler value)

 TIM1CH4:  (PA11) Sync signal
 TIM1CH3:  (NC)  Video out timing interrupt
 SPI1MOSI: (PA7) Video signal

 PA11 -- R1 --+
 PA7 -- R2 --+---> Video

 R1: 560 ohm
 R2: 240 ohm

 TIM4CH4: (PB9)  Sound
 Key input: USART2_RX £¨PA3£© or PS/2 PA8(Clock)/PA9(DATA)

 */

#include "debug.h"
#include "cpuintrf.h"
#include "m6800.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include "bmrom.h"
#include "PS2Keyboard.h"

/* Global define */

#define NTSC_COUNT 3050 // = 63.56 us / 48MHz
#define NTSC_HSYNC 225  // =  4.7  us / 48MHz
#define NTSC_VSYNC 2825 // = NTSC_COUNT - NTSC_HSYNC
#define NTSC_SCAN_DELAY 0 // Delay for video signal generation
#define NTSC_SCAN_START 40 // Display start line

#define NTSC_X_PIXELS 256
#define NTSC_Y_PIXELS 192

#define NTSC_PRESCALER SPI_BaudRatePrescaler_16

#define NTSC_X_CHARS (NTSC_X_PIXELS/8)
#define NTSC_Y_CHARS (NTSC_Y_PIXELS/8)

// Choose Keyboard

//#define USE_PS2_KEYBOARD
#define USE_USB_KEYBOARD


#ifdef USE_PS2_KEYBOARD
#include "PS2Keyboard.h"
#endif

#ifdef USE_USB_KEYBOARD
#include "usb_host_config.h"
#ifdef USE_PS2_KEYBOARD
#error "You can not enable both PS/2 and USB keyboard."
#endif
#endif

#define RX_BUFFER_LEN 64

/* Global Variable */

volatile uint16_t ntsc_line;
volatile uint8_t ntsc_blank = 0;
volatile uint8_t run_emulation = 0;
volatile uint8_t key_break=0;

uint8_t *scandata[2];
uint8_t keymatrix[16];   // 15 = shift keys
uint8_t iomem[16];
volatile uint8_t cmt_buff;
volatile uint8_t cmt_bit = 0;
volatile uint8_t cmt_running=0;

uint64_t last_usart_tx_systick = 0;
uint32_t last_cmt_cycles=0;
uint32_t total_cycles=0;

volatile uint8_t rxbuff[RX_BUFFER_LEN];
uint8_t rxptr = 0;
uint32_t lastptr = RX_BUFFER_LEN;

#define VRAM_START_ADDRESS 0x100
#define GVRAM_START_ADDRESS 0x900

//16KB  (maybe 2KB free)
#define MEM_SIZE 16384
const uint8_t memmap[] = { //
        1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,   // Memory map
        0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 4, 5, 5 }; // 1=RAM,3=BASIC ROM,4=IO,5=MonitorROM

#if !defined (USE_PS2_KEYBOARD) && !defined (USE_USB_KEYBOARD)
// keymap
const uint8_t keymap[] = { 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0,
        0, // scanline , key ,modifyer(shift/ctrl)
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 8, 16, 2, 0xff, 0, 0,
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 8, 8, 0, 0xff, 0, 0, 0xff, 0,
        0, // 0x0D = CR
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0,
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0, 16, 2,
        0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 0xff, 0, 0, 8, 2, 0, 3, 1, 1, 3,
        2, // 0x1B (ESC = break (Ctrl+C)
        1, 3, 4,
        1, // 0x20
        3, 8, 1, 3, 16, 1, 4, 1, 1, 4, 2, 1, 4, 4, 1, 4, 8, 1, 8, 4, 1, 6, 16,
        1, 7, 16, 0, 8, 16, 0, 8, 1, 0, 6, 8, 1, 4, 16, 0, 3, 1, 0, 3, 2, 0, 3,
        4,
        0, // 0x30
        3, 8, 0, 3, 16, 0, 4, 1, 0, 4, 2, 0, 4, 4, 0, 4, 8, 0, 8, 4, 0, 6, 16,
        0, 7, 16, 1, 8, 16, 1, 8, 1, 1, 6, 4, 1, 5, 1, 1, 1, 1, 0, 7, 2, 0, 0,
        16,
        0, // 0x40
        1, 4, 0, 2, 4, 0, 1, 8, 0, 1, 16, 0, 6, 1, 0, 5, 4, 0, 6, 2, 0, 6, 4, 0,
        6, 8, 0, 7, 8, 0, 7, 4, 0, 5, 8, 0, 5, 16, 0, 2, 1, 0, 2, 8, 0, 1, 2,
        0, // 0x50
        2, 16, 0, 5, 2, 0, 7, 1, 0, 2, 2, 0, 0, 8, 0, 5, 1, 0, 0, 4, 0, 5, 8, 1,
        5, 4, 1, 5, 16, 1, 4, 16, 1, 7, 8, 1 };
#endif

#ifdef USE_PS2_KEYBOARD

// PS/2 Keymap
const uint8_t ps2_keymap[] =   {  //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0x00
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00,15, 0x20, 0, 0x00,15, 0x10, 0, 0x04, 0, 0x08, 0, 0x00,   //0x10
        0, 0x00, 0, 0x00, 0, 0x01, 1, 0x02, 0, 0x02, 1, 0x04, 1, 0x08, 0, 0x00,   //
        0, 0x00, 2, 0x01, 1, 0x01, 2, 0x02, 2, 0x04, 3, 0x08, 2, 0x08, 0, 0x00,   //0x20
        0, 0x00,11, 0x01, 3, 0x01, 3, 0x02, 4, 0x04, 3, 0x04, 4, 0x08, 0, 0x00,   //
        0, 0x00, 5, 0x01, 4, 0x01, 5, 0x02, 4, 0x02, 5, 0x04, 5, 0x08, 0, 0x00,   //0x30
        0, 0x00, 0, 0x00, 6, 0x01, 6, 0x02, 6, 0x04, 6, 0x08, 7, 0x08, 0, 0x00,   //
        0, 0x00, 7, 0x01, 7, 0x02, 7, 0x04, 8, 0x04, 9, 0x08, 8, 0x08, 0, 0x00,   //0x40
        0, 0x00, 8, 0x01, 9, 0x01, 8, 0x02, 9, 0x02, 9, 0x04,10, 0x08, 0, 0x00,   //
        0, 0x00,10, 0x01,10, 0x02, 0, 0x00,10, 0x04,11, 0x08, 0, 0x00, 0, 0x00,   //0x50
        0, 0x00,15, 0x40,12, 0x02,11, 0x04, 0, 0x00,11, 0x02, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,12, 0x04, 0, 0x00,   //0x60
        0, 0x00, 0, 0x00,12, 0x08, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0x70
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00 };
#endif

#ifdef USE_USB_KEYBOARD

// USB Keymap
const uint8_t usb_keymap[] = { //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x02, 4, 0x01, 2, 0x01, 2, 0x02,   //0x00  A=0x04
        2, 0x04, 3, 0x02, 4, 0x02, 5, 0x02, 7, 0x04, 6, 0x02, 7, 0x02, 8, 0x02,   //
        6, 0x01, 5, 0x01, 8, 0x04, 9, 0x04, 0, 0x04, 3, 0x04, 1, 0x02, 4, 0x04,   //0x10
        6, 0x04, 3, 0x01, 1, 0x04, 1, 0x01, 5, 0x04, 0, 0x01, 0, 0x08, 1, 0x08,   //
        2, 0x08, 3, 0x08, 4, 0x08, 5, 0x08, 6, 0x08, 7, 0x08, 8, 0x08, 9, 0x08,   //0x20
       12, 0x02, 0, 0x00,12, 0x04, 0, 0x00,11, 0x01,10, 0x08,11, 0x08,10, 0x04,   //
       11, 0x04,11, 0x02, 0, 0x00, 9, 0x02,10, 0x02, 0, 0x00, 7, 0x01, 8, 0x01,   //0x30
        9, 0x01, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0x40
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0x50
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0x60
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0x70
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,10, 0x01,   //0x80
        0, 0x00,12, 0x08, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0x90
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0xa0
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0xb0
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0xc0
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0xd0
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0xe0
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00,   //0xf0
        0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00, 0, 0x00   //
};

#endif

// general global variables
//extern uint8_t RAM[];            // any reads or writes to ports or vectors are trapped in SW
uint8_t *RAM; // any reads or writes to ports or vectors are trapped in SW

static uint8_t usart_getch();


// TVout for CH32V203

void video_init() {

    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    SPI_InitTypeDef SPI_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd(
    RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1, ENABLE);

    // PC4:Sync

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure);

    // PC6: Video (SPI1)

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure);

    // Initalize TIM1

    TIM_TimeBaseInitStructure.TIM_Period = NTSC_COUNT;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1;                // Presclaer = 0
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = NTSC_HSYNC;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM1, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = NTSC_HSYNC * 2 - NTSC_SCAN_DELAY; // 9.4usec - delay
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init( TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC3PreloadConfig( TIM1, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig( TIM1, ENABLE);
    TIM_Cmd( TIM1, ENABLE);

    // Initialize SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = NTSC_PRESCALER; // 6MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure);

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_Cmd(SPI1, ENABLE);

    // NVIC

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);

    // Init VRAM

    scandata[0] = malloc(NTSC_X_CHARS + 1);
    scandata[1] = malloc(NTSC_X_CHARS + 1);

    scandata[0][NTSC_X_CHARS] = 0;
    scandata[1][NTSC_X_CHARS] = 0;

    //

}

static inline void video_cls() {
    memset(RAM + VRAM_START_ADDRESS, 0, (NTSC_X_CHARS * NTSC_Y_CHARS));
}

void video_wait_vsync() {

    while(ntsc_blank==1);
    while(ntsc_blank==0);

}

/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr,
        u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

}

void TIM4_PWMOut_Init(u16 arr, u16 psc, u16 ccp) {
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init( TIM4, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_OC4PreloadConfig( TIM4, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig( TIM4, ENABLE);
    TIM_Cmd( TIM4, ENABLE);
}

// Emulator related

void cpu_writemem16(unsigned short addr, unsigned char bdat) { // RAM access is managed here to allow memory-mapped I/O access via traps

    uint32_t cmt_diff;
    static uint8_t cmt_tone=0;
    static uint8_t cmt_bitcount=0;
    static uint16_t cmt_totalbits=0;
    static uint16_t cmt_ch=0;

    switch (memmap[addr >> 11]) {

    case 1: // RAM
        RAM[addr & 0x3fff] = bdat;
        break;
    case 4: // IO
        if(addr>=0xee00) {
            iomem[(addr&0x1ff)>>5]=bdat;
            switch (addr & 0x1f0) {
            case 0x80: // $ee80 Tape & DAC
                TIM4->CH4CVR= (bdat&0x3e)>>1;
                if(cmt_running==1) {
//                    printf("%d\n\r",total_cycles-last_cmt_cycles);
                    cmt_diff=total_cycles-last_cmt_cycles;
                    last_cmt_cycles=total_cycles;
                    if((bdat&1)==1) {
                        if((cmt_diff>180)&&(cmt_diff<220)) {  // 208
                            cmt_tone=1;
                        } else if ((cmt_diff>400)&&(cmt_diff<440)) {  // 415
                            cmt_tone=0;
                        } else {
                            cmt_buff = 0;
                            cmt_bit = 0;
                            cmt_bitcount=0;
                            cmt_totalbits=0;
                            cmt_ch=0;
                            break;
                        }
                    } else {
                        break;
                    }

                    if ((SysTick->CNT - last_usart_tx_systick)
                            > SystemCoreClock / 50) {
                        cmt_buff = 0;
                        cmt_bit = 0;
                        cmt_bitcount=0;
                        cmt_totalbits=0;
                        cmt_ch=0;
                        printf("\n\r");
                    }
                    last_usart_tx_systick = SysTick->CNT;

                    cmt_bitcount++;
//                    printf("%d%d",cmt_tone,cmt_bitcount);
                    if((cmt_tone==1)&&(cmt_bitcount!=8)) break;
                    if((cmt_tone==0)&&(cmt_bitcount!=4)) break;

                    cmt_bitcount=0;

                    cmt_ch >>= 1;
                    if (cmt_tone==1) {
                        cmt_ch|=0x400;
                    } else {
                    }

                    cmt_totalbits++;

                    if(cmt_totalbits%11==0) {
                        printf("%02x",((cmt_ch)>>1)&0xff);
//                        if((cmt_ch&0x600)!=0x600) {
//                            printf("!%03x\n\r",cmt_ch);
//                        }
                    }

               }
                break;
            default:
                iomem[(addr&0x1ff)>>5]=bdat;
                break;
            }
        }
    }

}
//
unsigned char cpu_readmem16(unsigned short addr) { // to allow for memory-mapped I/O access

    uint32_t cmt_diff,cmt_toggle_diff;
    static uint8_t cmt_tone=0;
    static uint8_t cmt_bitcount=0;
    static uint16_t cmt_ch;
    static uint16_t cmt_total_bits=0;
    static uint16_t cmt_last_bit=0;
    uint8_t cmt_ch1,cmt_ch2;
    static uint32_t last_cmt_toggle_cycle=0;
    uint64_t oldtick;

    switch (memmap[addr >> 11]) {

    case 1: // RAM
        return (RAM[addr & 0x3fff]);
    case 3: // BASIC ROM
        return (basicrom[addr - 0xb000]);
    case 4: // IO
        if(addr>=0xee00) {
            switch (addr & 0x1f0) {
            case 0: // $ee00 cmt stop
//                printf("CMT STOP\n\r");
                cmt_running=0;
                return 0x01;
            case 0x20: //$ee20 cmt start
//                printf("CMT START\n\r");
                cmt_running=1;
                return 0x01;
            case 0x80:  // $ee80 Tape input
                if(cmt_running==1) {
                    cmt_diff=total_cycles-last_cmt_cycles;
                    cmt_toggle_diff=total_cycles-last_cmt_toggle_cycle;
                    last_cmt_cycles=total_cycles;
                    if(cmt_diff>5000) {  // reset new data
                        //                                                printf("!%d\n\r",cmt_diff);
                        cmt_bitcount=11;
                        cmt_tone=0;
                        last_cmt_toggle_cycle=total_cycles;
                        cmt_total_bits=0;
                        cmt_last_bit=0;
                    } else {
                        cmt_total_bits=cmt_toggle_diff/(415*8);

                        if(cmt_total_bits==cmt_last_bit) { //
                            if(cmt_tone==1) {
//                                printf("h");
                                return 0x80;

                            } else {
//                                printf("l");
                                return 0x0;
                            }
                        }
                    }

                    cmt_last_bit=cmt_total_bits;
                    cmt_bitcount++;

                    oldtick=SysTick->CNT;

                    if(cmt_bitcount>=11) { // load next bit
                        cmt_ch=0;
                        cmt_ch1=cmt_ch2=0;
                        while(cmt_ch1==0) {
                            cmt_ch1=toupper(usart_getch());
                            if((SysTick->CNT-oldtick)>SystemCoreClock/50) {   //Timeout
//                                printf("!");
                                cmt_tone=0;
                                cmt_ch=0;
                                last_cmt_toggle_cycle=total_cycles;
                                cmt_total_bits=0;
                                cmt_last_bit=0;
                                return 0;
                            }
                        }
                        if(cmt_ch1<0x21) {  // data buffer is empty
                            cmt_tone=0;
                            cmt_ch=0;
                            last_cmt_toggle_cycle=total_cycles;
                            cmt_total_bits=0;
                            cmt_last_bit=0;
                            return 0;
                        }
                        cmt_ch1-='0';
                        if(cmt_ch1>10) cmt_ch1-=7;

                        while(cmt_ch2==0) {
                            cmt_ch2=toupper(usart_getch());
                            if((SysTick->CNT-oldtick)>SystemCoreClock/50) {   //Timeout
 //                               printf("!");
                                cmt_tone=0;
                                cmt_ch=0;
                                last_cmt_toggle_cycle=total_cycles;
                                cmt_total_bits=0;
                                cmt_last_bit=0;
                                return 0;
                            }
                        }
                        if(cmt_ch2<0x21) {  // data buffer is empty
                            cmt_tone=0;
                            cmt_ch=0;
                            last_cmt_toggle_cycle=total_cycles;
                            cmt_total_bits=0;
                            cmt_last_bit=0;
                            return 0;
                        }
                        cmt_ch2-='0';
                        if(cmt_ch2>10) cmt_ch2-=7;
                        //                        printf(" %d %d ",cmt_ch1,cmt_ch2);

                        cmt_ch=(cmt_ch1<<4)+cmt_ch2;

                        cmt_bitcount=0;
//                        printf("%02x",cmt_ch);

                    }

                    if(cmt_bitcount==0) { cmt_tone=0; return 0;}
                    if(cmt_bitcount==9) { cmt_tone=1; return 0x80;}
                    if(cmt_bitcount==10) { cmt_tone=1; return 0x80;}

                    cmt_tone=((cmt_ch&(1<<(cmt_bitcount-1)))!=0)?1:0;

                    if(cmt_tone==1) {
//                        printf("H");
                        return 0x80;

                    } else {
//                        printf("L");
                        return 0;
                    }

                } else {
                    return 0;
                }
            case 0xc0:  // $eec0 Keyboard

 //              printf("Keycode: %x,%x\n\r",iomem[6],keymatrix[iomem[6]&0xf]);
//                return keymatrix[iomem[6]&0xf];
                return (keymatrix[iomem[6]&0xf]&0x0f)+(keymatrix[15]&0xf0);
            case 0x100:  // $ef00 IRQ
                if((iomem[8]&0x80)!=0) {
                    iomem[8]&= ~(0x80);
                    return 0x80;
                }
                return 0;
            case 0x180:  // $ef80 NMI
                if((iomem[12]&0x80)!=0) {
                    iomem[12]&= ~(0x80);
                    return 0x80;
                }
                return 0;
            default:
                return iomem[(addr&0x1ff)>>5];
            }
        }
        return 0;
        break;
    case 5: // Monitor ROM
        return (monrom[addr & 0xfff]);

    }

    return 0xff;         // traps go above here

}

void beep_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    /*
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_Init(GPIOA, &GPIO_InitStructure);
     */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM4_PWMOut_Init(32, 63, 32);

}

//unsigned char cpu_readop(uint16_t addr) {return cpu_readmem16(addr); };
//unsigned char cpu_readop_arg(uint16_t addr) {return cpu_readmem16(addr); };

// init UART2 for key input

void USART_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    USART_InitTypeDef USART_InitStructure = { 0 };

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART2 TX-->A.2   RX-->A.3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART2, ENABLE);

}

void DMA_Rx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr,
        u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

    DMA_Cmd(DMA_CHx, ENABLE);

}

static inline uint8_t usart_getch() {

    uint8_t ch;
    uint32_t currptr;

    currptr = DMA_GetCurrDataCounter(DMA1_Channel6);

    if (currptr == lastptr) {
        return 0;
    }

    ch = rxbuff[rxptr];
    lastptr--;
    if (lastptr == 0) {
        lastptr = RX_BUFFER_LEN;
    }

    rxptr++;
    if (rxptr >= RX_BUFFER_LEN)
        rxptr = 0;

    return ch;

}

#if !defined (USE_PS2_KEYBOARD) && !defined (USE_USB_KEYBOARD)
void USART_Getkey() {

    static uint8_t ch, pressed;
    static uint8_t col, row;

    if (pressed == 0) { // no modifier keys pressed
        for (int i = 0; i < 9; i++) {
            keymatrix[i] = 0xff;
        }

        ch = toupper(usart_getch());

        if (ch != 0) {

//            if (ch == '|') { // load test game to memory
//                memcpy(RAM + 0x1000, jr100guldus, 0x1600);
//            }

            if (keymap[ch * 3] < 9) {
                if (keymap[ch * 3 + 2] == 1) { // use modifier (Shift)
                    keymatrix[0] &= ~(2);
                    col = keymap[ch * 3];
                    row = keymap[ch * 3 + 1];
                    pressed = 1;
                } else if (keymap[ch * 3 + 2] == 2) { // Control
                    keymatrix[0] &= ~(1);
                    col = keymap[ch * 3];
                    row = keymap[ch * 3 + 1];
                    pressed = 1;
                } else {
                    keymatrix[keymap[ch * 3]] &= ~(keymap[ch * 3 + 1]);
                }
            }
        }
    } else {
        pressed++;
        if (pressed > 2) {
            keymatrix[col] &= ~(row);
            pressed = 0;
        }
    }

}
#endif

//

void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM1_CC_IRQHandler(void) {

    TIM_ClearFlag(TIM1, TIM_FLAG_CC3);
    uint8_t char_x, char_y, slice_y, ch, ddat;
    uint8_t clocks, clocks2;
#ifdef USE_USB_KEYBOARD
    uint8_t index;
    uint8_t hub_port;
    uint8_t intf_num, in_num;
#endif

    ntsc_line++;

    // VSYNC/HSYNC slection for next scanline

    if ((ntsc_line == 3) || (ntsc_line == 4) || (ntsc_line == 5)) { // VSYNC : ntsc_line : 4-6
        TIM_SetCompare4(TIM1, NTSC_VSYNC);
        //    TIM1->CH4CVR = NTSC_VSYNC;
    } else {
        TIM_SetCompare4(TIM1, NTSC_HSYNC);
        //    TIM1->CH4CVR = NTSC_HSYNC;
    }

    // Video Out

    if ((ntsc_line >= NTSC_SCAN_START)
            && (ntsc_line < (NTSC_SCAN_START + NTSC_Y_PIXELS))) { // video out
        ntsc_blank = 0;
        DMA_Tx_Init(DMA1_Channel3, (u32) (&SPI1->DATAR + 1),
                (u32) scandata[ntsc_line % 2], NTSC_X_CHARS + 1);
        DMA_Cmd(DMA1_Channel3, ENABLE);
    } else {
        ntsc_blank = 1;
    }

    // Redner fonts for next scanline

    if ((ntsc_line >= NTSC_SCAN_START - 1)
            && (ntsc_line < (NTSC_SCAN_START + NTSC_Y_PIXELS - 1))) {

        char_y = (ntsc_line + 1 - NTSC_SCAN_START) / 8;
        slice_y = (ntsc_line + 1 - NTSC_SCAN_START) % 8;

        for (char_x = 0; char_x < NTSC_X_CHARS; char_x++) {
            ch = RAM[char_x + char_y * NTSC_X_CHARS + VRAM_START_ADDRESS];

            ddat = ((iomem[15] & 0x80) == 0) ? fontrom[ch * 8 + slice_y] : 0; // $efe0 bit 7 off
            ddat |= ((iomem[15] & 0x40) != 0) ?
                            RAM[char_x + (char_y*8 + slice_y) * NTSC_X_CHARS + GVRAM_START_ADDRESS + (iomem[15] & 0x0f) * 0x200] : 0;       // $efe0 bit 6 on
            if ((iomem[2] & 0x80) != 0) {        // $ee40 bit 7 = reverse
                scandata[(ntsc_line + 1) % 2][char_x] = ~ddat;

            } else {
                scandata[(ntsc_line + 1) % 2][char_x] = ddat;

            }
        }

    }

    if (ntsc_line > 262) {
        ntsc_line = 0;
    }

    // Execute m6800
    // 63.5us/750KHz~48clocks

    if (run_emulation != 0) {

        clocks2 = 0;
        while(clocks2<48) {
            clocks=m6800_execute(1);
            clocks2+=clocks;
            total_cycles+=clocks;
            //           exec6522(clocks);
        }
    }

    // check NMI

    if((key_break==1)&&((iomem[6]&0x80)!=0)) {
//        printf("NMI:\n\r");
        key_break=0;
        iomem[12]|=0x80; // $ef80
        ENTER_INTERRUPT("", 0xfffc);
    }

    // check IRQ

    if(ntsc_line==0) {
        iomem[8]|=0x80;  // $ef00
        if((iomem[14]&0x10)!=0) {
            ENTER_INTERRUPT2("", 0xfff8);
        }
    }



#ifdef USE_USB_KEYBOARD
    /* USB HID Device Input Endpoint Timing */
    if( RootHubDev.bStatus >= ROOT_DEV_SUCCESS )
    {
        index = RootHubDev.DeviceIndex;
        if( RootHubDev.bType == USB_DEV_CLASS_HID )
        {
            for( intf_num = 0; intf_num < HostCtl[ index ].InterfaceNum; intf_num++ )
            {
                for( in_num = 0; in_num < HostCtl[ index ].Interface[ intf_num ].InEndpNum; in_num++ )
                {
                    HostCtl[ index ].Interface[ intf_num ].InEndpTimeCount[ in_num ]++;
                }
            }
        }
        else if( RootHubDev.bType == USB_DEV_CLASS_HUB )
        {
            HostCtl[ index ].Interface[ 0 ].InEndpTimeCount[ 0 ]++;
            for( hub_port = 0; hub_port < RootHubDev.bPortNum; hub_port++ )
            {
                if( RootHubDev.Device[ hub_port ].bStatus >= ROOT_DEV_SUCCESS )
                {
                    index = RootHubDev.Device[ hub_port ].DeviceIndex;

                    if( RootHubDev.Device[ hub_port ].bType == USB_DEV_CLASS_HID )
                    {
                        for( intf_num = 0; intf_num < HostCtl[ index ].InterfaceNum; intf_num++ )
                        {
                            for( in_num = 0; in_num < HostCtl[ index ].Interface[ intf_num ].InEndpNum; in_num++ )
                            {
                                HostCtl[ index ].Interface[ intf_num ].InEndpTimeCount[ in_num ]++;
                            }
                        }
                    }
                }
            }
        }
    }

#endif

}

#ifdef USE_PS2_KEYBOARD

void ps2_getkey() {
    uint8_t ps2_keycode, col, row;
    static uint8_t ps2_extra = 0;
    static uint8_t ps2_depressed = 0;

    ps2_keycode = get_scan_code();

//   if(ps2_keycode!=0) printf("%x ",ps2_keycode);

    if (ps2_keycode == 0xe0) {
        ps2_extra = 1;
    } else if (ps2_keycode == 0xf0) {
        ps2_depressed = 1;
    } else if ((ps2_keycode != 0)&&(ps2_keycode!=0xff)) {
        if (ps2_extra == 0) {      // skip extra modifier keys (eg. Right Ctrl)
            col = ps2_keymap[ps2_keycode * 2];
            row = ps2_keymap[ps2_keycode * 2 + 1];
            if (row != 0) {
                if (ps2_depressed == 0) {
                    keymatrix[col] &= ~row;
                } else {
                    keymatrix[col] |= row;
                }
            } else if (ps2_keycode == 0x76) {
                if(ps2_depressed==0) {
                    // ESC: make NMI or RESET
                    if((keymatrix[15]&0x40)==0) { // if KanaKigou is pressed
                        m6800_reset();
                    } else {
                        key_break=1;
                        //                printf("NMI:\n\r");
//                        ENTER_INTERRUPT("", 0xfffc);
                    }
                } else {
                    key_break=0;
                }
            } else if (ps2_keycode == 0x11) {
                // ALT: reset keymatrix
                for (int i = 0; i < 16; i++) {
                    keymatrix[i] = 0xff;
                }
            } else if ((ps2_keycode == 0x05) && (ps2_depressed == 0)) {
                // F1: CMT LOAD
//                cmt_load();
            } else if ((ps2_keycode == 0x07) && (ps2_depressed == 0)) {
                // F12: load test game to memory
//                memcpy(RAM + 0x1000, jr100guldus, 0x1600);
            }
        } else {   // extra modifier keys
            if(ps2_keycode==0x14) {      // Right Ctrl = Kana
                if(ps2_depressed == 0) {
                    keymatrix[15] &= 0x7f;
                } else {
                    keymatrix[15] |= 0x80;
                }
            }
        }
        ps2_depressed = 0;
        ps2_extra = 0;
    }

}

#endif

#ifdef USE_USB_KEYBOARD

void usb_getkey( uint8_t index, uint8_t intf_num, uint8_t *pbuf, uint16_t len ) {

        uint8_t  i;
        uint8_t  value;
        uint8_t  bit_pos = 0x00;
        uint8_t  modifyer=0;
        uint8_t  keycode;
        uint8_t  key_break_check;

//        for(i=0;i<len;i++) {
//            printf("%x ",pbuf[i]);
//        }
//        printf("\n\r");
        key_break_check=0;

        if((len==8)&&(pbuf[1]!=0x1)) {

            // reset all keys
            for(i=0;i<16;i++) {
                keymatrix[i]=0xff;
            }


            modifyer=pbuf[0];
            for(int bit=0;bit<8;bit++) {
                if((modifyer&(1<<bit))!=0) {
                    switch(bit) {
                    case 0:  // left CTRL = EISUU
                        keymatrix[15]&= ~(0x10);
                        break;
                    case 1:  // left SHIFT = EIKIGOU
                        keymatrix[15]&= ~(0x20);
                        break;
                    case 4:  // right CTRL = KANA
                        keymatrix[15]&= ~(0x80);
                        break;
                    case 5:  // right SHIFT = KANAKIGOU
                        keymatrix[15]&= ~(0x40);
                        break;
                    }
                }
            }

            for(i=2;i<8;i++) {
                keycode=pbuf[i];
                if(keycode!=0) {
                    if(usb_keymap[keycode*2+1]!=0) {
                        keymatrix[usb_keymap[keycode*2]] &= ~usb_keymap[keycode*2+1];
                    }
                    if(keycode==0x29) { // ESCAPE
                        if((modifyer&0x20)!=0) {  // if KANAKIGOU is pressed
                            m6800_reset();
                        } else {
//                            ENTER_INTERRUPT("", 0xfffc);
                            key_break_check=1;
                        }
                    }
                }
            }
        }

        if((key_break==0)&&(key_break_check==1)) {
            key_break=1;
        } else if((key_break==1)&&(key_break_check==0)) {
            key_break=0;
        }

        value = HostCtl[ index ].Interface[ intf_num ].SetReport_Value;

        for( i = HostCtl[ index ].Interface[ intf_num ].LED_Usage_Min; i <= HostCtl[ index ].Interface[ intf_num ].LED_Usage_Max; i++ )
        {
            if( i == 0x01 )
            {
                if( memchr( pbuf, DEF_KEY_NUM, len ) )
                {
                    HostCtl[ index ].Interface[ intf_num ].SetReport_Value ^= ( 1 << bit_pos );
                }
            }
            else if( i == 0x02 )
            {
                if( memchr( pbuf, DEF_KEY_CAPS, len ) )
                {
                    HostCtl[ index ].Interface[ intf_num ].SetReport_Value ^= ( 1 << bit_pos );
                }
            }
            else if( i == 0x03 )
            {
                if( memchr( pbuf, DEF_KEY_SCROLL, len ) )
                {
                    HostCtl[ index ].Interface[ intf_num ].SetReport_Value ^= ( 1 << bit_pos );
                }
            }

            bit_pos++;
        }

        if( value != HostCtl[ index ].Interface[ intf_num ].SetReport_Value )
        {
            HostCtl[ index ].Interface[ intf_num ].SetReport_Flag = 1;
        }
        else
        {
            HostCtl[ index ].Interface[ intf_num ].SetReport_Flag = 0;
        }

}

/*********************************************************************
 * @fn      USBH_MainDeal
 *
 * @brief   Provide a simple enumeration process for USB devices and
 *          obtain keyboard and mouse data at regular intervals.
 *
 * @return  none
 */
void usb_keyboard( void )
{
    uint8_t  s;
    uint8_t  index;
    uint8_t  hub_port;
    uint8_t  hub_dat;
    uint8_t  intf_num, in_num;
    uint16_t len;
#if DEF_DEBUG_PRINTF
    uint16_t i;
#endif

    s = USBFSH_CheckRootHubPortStatus( RootHubDev.bStatus ); // Check USB device connection or disconnection
    if( s == ROOT_DEV_CONNECTED )
    {
        DUG_PRINTF( "USB Port Dev In.\r\n" );

        /* Set root device state parameters */
        RootHubDev.bStatus = ROOT_DEV_CONNECTED;
        RootHubDev.DeviceIndex = DEF_USBFS_PORT_INDEX * DEF_ONE_USB_SUP_DEV_TOTAL;

        s = USBH_EnumRootDevice( ); // Simply enumerate root device
        if( s == ERR_SUCCESS )
        {
            if( RootHubDev.bType == USB_DEV_CLASS_HID ) // Further enumerate it if this device is a HID device
            {
                DUG_PRINTF("Root Device Is HID. ");

                s = USBH_EnumHidDevice( RootHubDev.DeviceIndex, RootHubDev.bEp0MaxPks );
                DUG_PRINTF( "Further Enum Result: " );
                if( s == ERR_SUCCESS )
                {
                    DUG_PRINTF( "OK\r\n" );

                    /* Set the connection status of the device  */
                    RootHubDev.bStatus = ROOT_DEV_SUCCESS;
                }
                else if( s != ERR_USB_DISCON )
                {
                    DUG_PRINTF( "Err(%02x)\r\n", s );

                    RootHubDev.bStatus = ROOT_DEV_FAILED;
                }
            }
            else if( RootHubDev.bType == USB_DEV_CLASS_HUB )
            {
                DUG_PRINTF("Root Device Is HUB. ");

                s = USBH_EnumHubDevice( );
                DUG_PRINTF( "Further Enum Result: " );
                if( s == ERR_SUCCESS )
                {
                    DUG_PRINTF( "OK\r\n" );

                    /* Set the connection status of the device  */
                    RootHubDev.bStatus = ROOT_DEV_SUCCESS;
                }
                else if( s != ERR_USB_DISCON )
                {
                    DUG_PRINTF( "Err(%02x)\r\n", s );

                    RootHubDev.bStatus = ROOT_DEV_FAILED;
                }
            }
            else // Detect that this device is a NON-HID device
            {
                DUG_PRINTF( "Root Device Is " );
                switch( RootHubDev.bType )
                {
                    case USB_DEV_CLASS_STORAGE:
                        DUG_PRINTF("Storage. ");
                        break;
                    case USB_DEV_CLASS_PRINTER:
                        DUG_PRINTF("Printer. ");
                        break;
                    case DEF_DEV_TYPE_UNKNOWN:
                        DUG_PRINTF("Unknown. ");
                        break;
                }
                DUG_PRINTF( "End Enum.\r\n" );

                RootHubDev.bStatus = ROOT_DEV_SUCCESS;
            }
        }
        else if( s != ERR_USB_DISCON )
        {
            /* Enumeration failed */
            DUG_PRINTF( "Enum Fail with Error Code:%x\r\n",s );
            RootHubDev.bStatus = ROOT_DEV_FAILED;
        }
    }
    else if( s == ROOT_DEV_DISCONNECT )
    {
        DUG_PRINTF( "USB Port Dev Out.\r\n" );

        /* Clear parameters */
        index = RootHubDev.DeviceIndex;
        memset( &RootHubDev.bStatus, 0, sizeof( ROOT_HUB_DEVICE ) );
        memset( &HostCtl[ index ].InterfaceNum, 0, sizeof( HOST_CTL ) );
    }

    /* Get the data of the HID device connected to the USB host port */
    if( RootHubDev.bStatus >= ROOT_DEV_SUCCESS )
    {
        index = RootHubDev.DeviceIndex;

        if( RootHubDev.bType == USB_DEV_CLASS_HID )
        {
            for( intf_num = 0; intf_num < HostCtl[ index ].InterfaceNum; intf_num++ )
            {
                for( in_num = 0; in_num < HostCtl[ index ].Interface[ intf_num ].InEndpNum; in_num++ )
                {
                    /* Get endpoint data based on the interval time of the device */
                    if( HostCtl[ index ].Interface[ intf_num ].InEndpTimeCount[ in_num ] >= HostCtl[ index ].Interface[ intf_num ].InEndpInterval[ in_num ] )
                    {
                        HostCtl[ index ].Interface[ intf_num ].InEndpTimeCount[ in_num ] %= HostCtl[ index ].Interface[ intf_num ].InEndpInterval[ in_num ];

                        /* Get endpoint data */
                        s = USBFSH_GetEndpData( HostCtl[ index ].Interface[ intf_num ].InEndpAddr[ in_num ],
                                                &HostCtl[ index ].Interface[ intf_num ].InEndpTog[ in_num ], Com_Buf, &len );
                        if( s == ERR_SUCCESS )
                        {
#if DEF_DEBUG_PRINTF
                            for( i = 0; i < len; i++ )
                            {
                                DUG_PRINTF( "%02x ", Com_Buf[ i ] );
                            }
                            DUG_PRINTF( "\r\n" );
#endif

                            /* Handle keyboard lighting */
                            if( HostCtl[ index ].Interface[ intf_num ].Type == DEC_KEY )
                            {
//                                KB_AnalyzeKeyValue( index, intf_num, Com_Buf, len );
                                usb_getkey( index, intf_num, Com_Buf, len );

                                if( HostCtl[ index ].Interface[ intf_num ].SetReport_Flag )
                                {
                                    KB_SetReport( index, RootHubDev.bEp0MaxPks, intf_num );
                                }
                            }
                        }
                        else if( s == ERR_USB_DISCON )
                        {
                            break;
                        }
                        else if( s == ( USB_PID_STALL | ERR_USB_TRANSFER ) )
                        {
                            /* USB device abnormal event */
                            DUG_PRINTF("Abnormal\r\n");

                            /* Clear endpoint */
                            USBFSH_ClearEndpStall( RootHubDev.bEp0MaxPks, HostCtl[ index ].Interface[ intf_num ].InEndpAddr[ in_num ] | 0x80 );
                            HostCtl[ index ].Interface[ intf_num ].InEndpTog[ in_num ] = 0x00;

                            /* Judge the number of error */
                            HostCtl[ index ].ErrorCount++;
                            if( HostCtl[ index ].ErrorCount >= 10 )
                            {
                                /* Re-enumerate the device and clear the endpoint again */
                                memset( &RootHubDev.bStatus, 0, sizeof( struct _ROOT_HUB_DEVICE ) );
                                s = USBH_EnumRootDevice( );
                                if( s == ERR_SUCCESS )
                                {
                                    USBFSH_ClearEndpStall( RootHubDev.bEp0MaxPks, HostCtl[ index ].Interface[ intf_num ].InEndpAddr[ in_num ] | 0x80 );
                                    HostCtl[ index ].ErrorCount = 0x00;

                                    RootHubDev.bStatus = ROOT_DEV_CONNECTED;
                                    RootHubDev.DeviceIndex = DEF_USBFS_PORT_INDEX * DEF_ONE_USB_SUP_DEV_TOTAL;

                                    memset( &HostCtl[ index ].InterfaceNum, 0, sizeof( struct __HOST_CTL ) );
                                    s = USBH_EnumHidDevice( index, RootHubDev.bEp0MaxPks );
                                    if( s == ERR_SUCCESS )
                                    {
                                        RootHubDev.bStatus = ROOT_DEV_SUCCESS;
                                    }
                                    else if( s != ERR_USB_DISCON )
                                    {
                                        RootHubDev.bStatus = ROOT_DEV_FAILED;
                                    }
                                }
                                else if( s != ERR_USB_DISCON )
                                {
                                    RootHubDev.bStatus = ROOT_DEV_FAILED;
                                }
                            }
                        }
                    }
                }

                if( s == ERR_USB_DISCON )
                {
                    break;
                }
            }
        }
        else if( RootHubDev.bType == USB_DEV_CLASS_HUB )
        {
           /* Query port status change */
           if( HostCtl[ index ].Interface[ 0 ].InEndpTimeCount[ 0 ] >= HostCtl[ index ].Interface[ 0 ].InEndpInterval[ 0 ] )
           {
               HostCtl[ index ].Interface[ 0 ].InEndpTimeCount[ 0 ] %= HostCtl[ index ].Interface[ 0 ].InEndpInterval[ 0 ];

               /* Select HUB port */
               USBFSH_SetSelfAddr( RootHubDev.bAddress );
               USBFSH_SetSelfSpeed( RootHubDev.bSpeed );

               /* Get HUB interrupt endpoint data */
               s = USBFSH_GetEndpData( HostCtl[ index ].Interface[ 0 ].InEndpAddr[ 0 ], &HostCtl[ index ].Interface[ 0 ].InEndpTog[ 0 ], Com_Buf, &len );
               if( s == ERR_SUCCESS )
               {
                   hub_dat = Com_Buf[ 0 ];
                   DUG_PRINTF( "Hub Int Data:%02x\r\n", hub_dat );

                   for( hub_port = 0; hub_port < RootHubDev.bPortNum; hub_port++ )
                   {
                       /* HUB Port PreEnumate Step 1: C_PORT_CONNECTION */
                       s = HUB_Port_PreEnum1( ( hub_port + 1 ), &hub_dat );
                       if( s == ERR_USB_DISCON )
                       {
                           hub_dat &= ~( 1 << ( hub_port + 1 ) );

                           /* Clear parameters */
                           memset( &HostCtl[ RootHubDev.Device[ hub_port ].DeviceIndex ], 0, sizeof( HOST_CTL ) );
                           memset( &RootHubDev.Device[ hub_port ].bStatus, 0, sizeof( HUB_DEVICE ) );
                           continue;
                       }

                       /* HUB Port PreEnumate Step 2: Set/Clear PORT_RESET */
                       Delay_Ms( 100 );  //
                       s = HUB_Port_PreEnum2( ( hub_port + 1 ), &hub_dat );
                       if( s == ERR_USB_CONNECT )
                       {
                           /* Set parameters */
                           RootHubDev.Device[ hub_port ].bStatus = ROOT_DEV_CONNECTED;
                           RootHubDev.Device[ hub_port ].bEp0MaxPks = DEFAULT_ENDP0_SIZE;
                           RootHubDev.Device[ hub_port ].DeviceIndex = DEF_USBFS_PORT_INDEX * DEF_ONE_USB_SUP_DEV_TOTAL + hub_port + 1;
                       }
                       else
                       {
                           hub_dat &= ~( 1 << ( hub_port + 1 ) );
                       }

                       /* Enumerate HUB Device */
                       if( RootHubDev.Device[ hub_port ].bStatus == ROOT_DEV_CONNECTED )
                       {
                           /* Check device speed */
                           RootHubDev.Device[ hub_port ].bSpeed = HUB_CheckPortSpeed( ( hub_port + 1 ), Com_Buf );
                           DUG_PRINTF( "Dev Speed:%x\r\n", RootHubDev.Device[ hub_port ].bSpeed );

                           /* Select the specified port */
                           USBFSH_SetSelfAddr( RootHubDev.Device[ hub_port ].bAddress );
                           USBFSH_SetSelfSpeed( RootHubDev.Device[ hub_port ].bSpeed );
                           if( RootHubDev.bSpeed != USB_LOW_SPEED )
                           {
                               USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
                           }

                           /* Enumerate the USB device of the current HUB port */
                           DUG_PRINTF("Enum_HubDevice\r\n");
                           s = USBH_EnumHubPortDevice( hub_port, &RootHubDev.Device[ hub_port ].bAddress, \
                                                       &RootHubDev.Device[ hub_port ].bType );
                           if( s == ERR_SUCCESS )
                           {
                               if( RootHubDev.Device[ hub_port ].bType == USB_DEV_CLASS_HID )
                               {
                                   DUG_PRINTF( "HUB port%x device is HID! Further Enum:\r\n", hub_port );

                                   /* Perform HID class enumeration on the current device */
                                   s = USBH_EnumHidDevice( RootHubDev.Device[ hub_port ].DeviceIndex, \
                                                           RootHubDev.Device[ hub_port ].bEp0MaxPks );
                                   if( s == ERR_SUCCESS )
                                   {
                                       RootHubDev.Device[ hub_port ].bStatus = ROOT_DEV_SUCCESS;
                                       DUG_PRINTF( "OK!\r\n" );
                                   }
                               }
                               else // Detect that this device is a Non-HID device
                               {
                                   DUG_PRINTF( "HUB port%x device is ", hub_port );
                                   switch( RootHubDev.Device[ hub_port ].bType )
                                   {
                                       case USB_DEV_CLASS_STORAGE:
                                           DUG_PRINTF("storage!\r\n");
                                           break;
                                       case USB_DEV_CLASS_PRINTER:
                                           DUG_PRINTF("printer!\r\n");
                                           break;
                                       case USB_DEV_CLASS_HUB:
                                           DUG_PRINTF("printer!\r\n");
                                           break;
                                       case DEF_DEV_TYPE_UNKNOWN:
                                           DUG_PRINTF("unknown!\r\n");
                                           break;
                                   }
                                   RootHubDev.Device[ hub_port ].bStatus = ROOT_DEV_SUCCESS;
                               }
                           }
                           else
                           {
                               RootHubDev.Device[ hub_port ].bStatus = ROOT_DEV_FAILED;
                               DUG_PRINTF( "HUB Port%x Enum Err!\r\n", hub_port );
                           }
                       }
                   }
               }
           }

           /* Get HUB port HID device data */
           for( hub_port = 0; hub_port < RootHubDev.bPortNum; hub_port++ )
           {
               if( RootHubDev.Device[ hub_port ].bStatus == ROOT_DEV_SUCCESS )
               {
                   index = RootHubDev.Device[ hub_port ].DeviceIndex;

                   if( RootHubDev.Device[ hub_port ].bType == USB_DEV_CLASS_HID )
                   {
                       for( intf_num = 0; intf_num < HostCtl[ index ].InterfaceNum; intf_num++ )
                       {
                           for( in_num = 0; in_num < HostCtl[ index ].Interface[ intf_num ].InEndpNum; in_num++ )
                           {
                               /* Get endpoint data based on the interval time of the device */
                               if( HostCtl[ index ].Interface[ intf_num ].InEndpTimeCount[ in_num ] >= HostCtl[ index ].Interface[ intf_num ].InEndpInterval[ in_num ] )
                               {
                                   HostCtl[ index ].Interface[ intf_num ].InEndpTimeCount[ in_num ] %= HostCtl[ index ].Interface[ intf_num ].InEndpInterval[ in_num ];

                                   /* Select HUB device port */
                                   USBFSH_SetSelfAddr( RootHubDev.Device[ hub_port ].bAddress );
                                   USBFSH_SetSelfSpeed( RootHubDev.Device[ hub_port ].bSpeed );
                                   if( RootHubDev.bSpeed != USB_LOW_SPEED )
                                   {
                                       USBOTG_H_FS->HOST_CTRL &= ~USBFS_UH_LOW_SPEED;
                                   }

                                   /* Get endpoint data */
                                   s = USBFSH_GetEndpData( HostCtl[ index ].Interface[ intf_num ].InEndpAddr[ in_num ], \
                                                           &HostCtl[ index ].Interface[ intf_num ].InEndpTog[ in_num ], Com_Buf, &len );
                                   if( s == ERR_SUCCESS )
                                   {
#if DEF_DEBUG_PRINTF
                                       for( i = 0; i < len; i++ )
                                       {
                                           DUG_PRINTF( "%02x ", Com_Buf[ i ] );
                                       }
                                       DUG_PRINTF( "\r\n" );
#endif

                                       if( HostCtl[ index ].Interface[ intf_num ].Type == DEC_KEY )
                                       {
//                                           KB_AnalyzeKeyValue( index, intf_num, Com_Buf, len );
                                           usb_getkey( index, intf_num, Com_Buf, len );

                                           if( HostCtl[ index ].Interface[ intf_num ].SetReport_Flag )
                                           {
                                               KB_SetReport( index, RootHubDev.Device[ hub_port ].bEp0MaxPks, intf_num );
                                           }
                                       }
                                   }
                                   else if( s == ERR_USB_DISCON )
                                   {
                                       break;
                                   }
                               }
                           }

                           if( s == ERR_USB_DISCON )
                           {
                               break;
                           }
                       }
                   }
               }
           }
        }
    }
}
#endif

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {

    // run Systick timer

    SysTick->CNT = 0;
    SysTick->CTLR |= (1 << 0);

    Delay_Init();

//  Peripheral setup

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    USART_CFG();

    DMA_Rx_Init( DMA1_Channel6, (u32) &USART2->DATAR, (u32) &rxbuff,
    RX_BUFFER_LEN);

#ifdef USE_USB_KEYBOARD
    USBFS_RCC_Init( );
    USBFS_Host_Init( ENABLE );
    memset( &RootHubDev.bStatus, 0, sizeof( ROOT_HUB_DEVICE ) );
    memset( &HostCtl[ DEF_USBFS_PORT_INDEX * DEF_ONE_USB_SUP_DEV_TOTAL ].InterfaceNum, 0, DEF_ONE_USB_SUP_DEV_TOTAL * sizeof( HOST_CTL ) );
#endif

    beep_init();

    video_init();
    video_cls();

#ifdef USE_PS2_KEYBOARD

    kbd_init();

#endif



//  Emulator setup

    RAM = malloc(MEM_SIZE);

    if(RAM==NULL) printf("memory allocation error\n\r");

    memset(RAM, 0, MEM_SIZE); // zeroes execute as NOP (as do all undefined instructions)

    for (int i = 0; i < 16; i++) {
        keymatrix[i] = 0xff;
    }

    memset(rxbuff, 0, RX_BUFFER_LEN);

    m6800_init();
    m6800_reset();

    iomem[14]=0x10; // IRQ enabled

    //   Delay_Init();
//
//     for (int xx = 0; xx < NTSC_X_CHARS; xx++) {
//     cpu_writemem16(xx, 0xaa);
//     if (cpu_readmem16(xx) != 0xaa)
//     vram[xx + 0x100] = 0x20;
//     Delay_Ms(10);
//     }

    run_emulation = 1;

    while(1)
    {

        video_wait_vsync();
#ifdef USE_PS2_KEYBOARD
        ps2_getkey();
#endif

#ifdef USE_USB_KEYBOARD
        usb_keyboard();
#endif

               if((cmt_bit!=0)&&(SysTick->CNT-last_usart_tx_systick>SystemCoreClock/100)) {
                   printf("\n\r");
                  cmt_bit=0;
                  cmt_buff=0;
              }

    }
}
