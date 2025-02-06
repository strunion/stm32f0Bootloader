#include "stm32f0xx.h"

#ifndef PAGES
    #define PAGES 16
#endif

#ifndef CRYPTO
    #define CRYPTO 1
#endif

#ifndef KEY1
    #define KEY1 0x1b1a1918
#endif

#ifndef KEY2
    #define KEY2 0x13121110
#endif

#ifndef KEY3
    #define KEY3 0x0b0a0908
#endif

#ifndef KEY4
    #define KEY4 0x03020100
#endif

#ifndef BAUDRATE
    #define BAUDRATE 115200
#endif

#ifndef USART
    #define USART 1
#endif

#if USART == 1
    #define UART USART1
    #define UART_RCC_EN RCC->APB2ENR = RCC_APB2ENR_USART1EN
    #define UART_RCC_RES RCC->APB2RSTR = RCC_APB2RSTR_USART1RST;
#endif

#if USART == 2
    #define UART USART2
    #define UART_RCC_EN RCC->APB1ENR = RCC_APB1ENR_USART2EN
    #define UART_RCC_RES RCC->APB1RSTR = RCC_APB1RSTR_USART2RST;
#endif

#ifndef RS485
    #define RS485 0
#endif

#ifndef IWDG_ENABLE
    #define IWDG_ENABLE 1
#endif

#if IWDG_ENABLE == 1
    #ifndef IWDG_RELOAD_SEC
        #define IWDG_RELOAD_SEC 2
    #endif
    #define IWDG_RELOAD (156.25 * IWDG_RELOAD_SEC)
    #define IWDG_START          0xCCCC
    #define IWDG_WRITE_ACCESS   0x5555
    #define IWDG_REFRESH        0xAAAA

    __STATIC_FORCEINLINE
    void iwdgInti(void){
        IWDG->KR = IWDG_START;
        IWDG->KR = IWDG_WRITE_ACCESS;
        IWDG->PR = IWDG_PR_PR_Msk;
        IWDG->RLR = IWDG_RELOAD;
    }

    __attribute__((noinline))
    void iwdgRefresh(void) {
        IWDG->KR = IWDG_REFRESH;
    }
#endif

#if CRYPTO == 1
    uint32_t l[29];
    uint32_t key[27];

    void speckInit_64_128(void){
        l[2] = KEY1;
        l[1] = KEY2;
        l[0] = KEY3;
        key[0] = KEY4;
        for (int i = 0; i < 26; i++) {
            l[i+3] = (key[i] + (l[i]>>8 | l[i]<<24)) ^ i;
            key[i+1] = (key[i]<<3 | key[i]>>29) ^ l[i+3];
        }
    }

    void speckDecrypt_64_128(uint32_t p[2]){
        uint32_t x = p[0];
        uint32_t y = p[1];
        for (int i = 26; i >= 0; i--) {
            y ^= x;
            y = y>>3 | y<<29;
            x ^= key[i];
            x -= y;
            x = x<<8 | x>>24;
        }
        p[0] = x;
        p[1] = y;
    }
#endif

#define APPLICATION_ADDRESS 0x08000400
#define app ((volatile uint32_t *)(APPLICATION_ADDRESS))
#define vec ((volatile uint32_t *)(SRAM_BASE))

union{
    uint8_t p[1024];
    uint16_t p16[512];
    uint32_t p32[256];
} page;

__STATIC_FORCEINLINE
void flashWritePage(uint32_t adr, const uint16_t* data) {
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = adr;
    FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT;
    while ((FLASH->SR & FLASH_SR_BSY) != 0);

    for(uint32_t i = 0; i < 512; i++) {
        FLASH->CR = FLASH_CR_PG;
        *(__IO uint16_t*)(adr + i * 2) = data[i];
        while ((FLASH->SR & FLASH_SR_BSY) != 0);
    }
}

// Самообновление загрузчика
__attribute__ ((section(".RamFunc"))) __NO_RETURN
void bootloaderSelfUpdate(void){
    UART->TDR = 0xAA;
    flashWritePage(FLASH_BASE, page.p16);
    SCB->AIRCR  = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
    __builtin_unreachable();
}

// Чтение данных из UART
int uartRead(void){
    int i = 0;
    while(!(UART->ISR & USART_ISR_RXNE)){
        if(i++ > 0x20000) return -1;    // ~212ms
    }
    #if IWDG_ENABLE == 1
        iwdgRefresh();
    #endif
    return UART->RDR;
}

// Запись данных в UART
void uartWrite(uint8_t d){
    while(!(UART->ISR & USART_ISR_TXE));
    UART->TDR = d;
}

// Чтение страницы из UART
__STATIC_FORCEINLINE
int uartPageRead(void){
    uint16_t crc = 211;
    for(int i = 0; i < 1024; i++){
        int c = uartRead();
        if(c == -1) return -1;
        page.p[i] = c;
        crc += c * 211;
        crc ^= crc>>8;
    }
    return crc & 0xFF;
}

#if IWDG_ENABLE == 0
    __NO_RETURN
    void hardFault(void){
        SCB->AIRCR  = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
        __builtin_unreachable();
    }
#endif

// Переход в приложение
__STATIC_FORCEINLINE __NO_RETURN
void goApp(){
    FLASH->SR = FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_LOCK;

    UART_RCC_RES;
    RCC->AHBRSTR = RCC_AHBRSTR_GPIOARST;
    RCC->APB1RSTR = 0;
    RCC->APB2RSTR = 0;
    RCC->AHBRSTR = 0;
    RCC->AHBENR = 0x00000014;
    RCC->APB2ENR = RCC_APB2ENR_SYSCFGEN;

    for (int i = 0; i < 48; i++)
        vec[i] = app[i];

    #if IWDG_ENABLE == 0
        vec[3] = (uint32_t)hardFault;
    #endif

    SYSCFG->CFGR1 = SYSCFG_CFGR1_MEM_MODE;

    #if IWDG_ENABLE == 1
        iwdgRefresh();
    #endif

    asm volatile(
        "msr msp, %[sp]   \n\t"
        "bx	%[pc]         \n\t"
        :: [sp] "r" (app[0]), [pc] "r" (app[1])
    );

    __builtin_unreachable();
}

__STATIC_FORCEINLINE
void rccInit(void){
    RCC->AHBENR = RCC_AHBENR_GPIOAEN;
    UART_RCC_EN;
}

__STATIC_FORCEINLINE
void perefInit(void){
    #if RS485 == 1
        GPIOA->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER9_1;
        GPIOA->AFR[1] = 0x00010110;

        UART->BRR = F_CPU / BAUDRATE;
        UART->CR3 = USART_CR3_DEM;
        UART->CR1 = USART_CR1_DEAT_Msk | USART_CR1_DEDT_Msk | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    #else
        // GPIOA->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER9_1;
        // GPIOA->AFR[1] = 0x00000110;

        GPIOA->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1;
        GPIOA->AFR[0] = 0x00001100;

        UART->BRR = F_CPU / BAUDRATE;
        UART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    #endif
}

__STATIC_FORCEINLINE
void sendHello(void){
    uartWrite(0xFE);
    uartWrite(0xE1);
    uartWrite(0xDE);
    uartWrite(0xAD);
}

__STATIC_FORCEINLINE
void getFirmware(void){
    while(1){
        int c = uartRead();
        if(c != 0xDE) return;

        c = uartRead();
        if(c != 0xAD) return;

        c = uartRead();
        if(c == 0x00){
            const char* str = "Zerg17 Bootloader v1.0\r\n";
            while(*str) uartWrite(*str++);
            return;
        }
        if(c != 0xBE) return;

        c = uartRead();
        if(c != 0xEF) return;

        uint32_t pageCtx = uartRead();
        if(pageCtx == -1) return;

        int crc = uartRead();
        if(crc == -1) return;

        if(uartPageRead() != crc) return;

        #if CRYPTO == 1
            for(int i = 0; i < 256; i+=2)
                speckDecrypt_64_128(&(page.p32[i]));
        #endif

        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        if(pageCtx == 127 && page.p32[0] == ((volatile uint32_t *)(FLASH_BASE))[0]){
            bootloaderSelfUpdate();
        }else if(pageCtx && pageCtx < PAGES){
            flashWritePage(FLASH_BASE + pageCtx * 1024, page.p16);
        }else return;

        uartWrite(0xAA); // Отправка ACK
    }
}

int main(){
    rccInit();
    perefInit();

    #if IWDG_ENABLE == 1
        iwdgInti();
    #endif

    #if CRYPTO == 1
        speckInit_64_128();
    #endif

    sendHello();
    getFirmware();
    goApp();
}
