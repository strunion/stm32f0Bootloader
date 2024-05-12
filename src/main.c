#include "stm32f0xx.h"

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

#ifndef CRYPTO
    #define CRYPTO 1
#endif

#ifndef BAUDRATE
    #define BAUDRATE 115200
#endif

#ifndef RS485
    #define RS485 1
#endif

#ifndef IWDG_RELOAD_SEC
    #define IWDG_RELOAD_SEC 1
#endif

#define IWDG_RELOAD (156.25 * IWDG_RELOAD_SEC)
#define IWDG_START          0xCCCC
#define IWDG_WRITE_ACCESS   0x5555
#define IWDG_REFRESH        0xAAAA

#define APPLICATION_ADDRESS 0x08000400

#define app ((volatile uint32_t *)(APPLICATION_ADDRESS))
#define vec ((volatile uint32_t *)(SRAM_BASE))

union{
    uint8_t p[1024];
    uint16_t p16[512];
    uint32_t p32[256];
} page;

#if CRYPTO == 1
    uint32_t l[29];
    uint32_t key[27];
#endif

// Очистка сектора(1КБ)
__STATIC_FORCEINLINE
void flashSectorClear(uint32_t adr){
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = adr;
    FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT;
    while ((FLASH->SR & FLASH_SR_BSY) != 0);
}

// Запись 2 байт данных(связанно с организацией flash) по адресу
__STATIC_FORCEINLINE
void flashWrite(uint32_t adr, uint16_t data){
    FLASH->CR = FLASH_CR_PG;
    *(__IO uint16_t*)(adr) = data;
    while ((FLASH->SR & FLASH_SR_BSY) != 0);
}

// Чтение данных из UART1
uint16_t uartRead(void){
    for(int i = 0; i < 0x8000; i++){
        if(USART1->ISR & USART_ISR_RXNE){
            IWDG->KR = IWDG_REFRESH;
            return USART1->RDR;
        }
    }
    return 0x100;
}

// Запись данных в UART1
void uartWrite(uint8_t d){
    while(!(USART1->ISR & USART_ISR_TXE));
    USART1->TDR = d;
}

#if CRYPTO == 1
    // Расшифровка алгоритмом Speck_64_128
    void speck_decrypt(uint32_t k[], uint32_t p[2]){
        uint32_t x = p[0];
        uint32_t y = p[1];
        for (int i = 26; i >= 0; i--) {
            y ^= x;
            y = y>>3 | y<<29;
            x ^= k[i];
            x -= y;
            x = x<<8 | x>>24;
        }
        p[0] = x;
        p[1] = y;
    }
#endif

// Чтение страницы из UART1
__STATIC_FORCEINLINE
uint16_t uartPageRead(void){
    uint16_t crc = 211;
    for(int i = 0; i < 1024; i++){
        uint16_t c = uartRead();
        if(c == 0x100) return 0x100;
        page.p[i] = c;
        crc += c * 211;
        crc ^= crc>>8;
    }
    return crc & 0xFF;
}

// Самообновление загрузчика
__attribute__ ((section(".RamFunc"))) __NO_RETURN
void bootloaderSelfUpdate(void){
    USART1->TDR = 0xAA;
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = FLASH_BASE;
    FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT;
    while ((FLASH->SR & FLASH_SR_BSY) != 0);
    for(int i = 0; i < 512; i++){
        FLASH->CR = FLASH_CR_PG;
        *(__IO uint16_t*)(FLASH_BASE + i * 2) = page.p16[i];
        while ((FLASH->SR & FLASH_SR_BSY) != 0);
    }
    SCB->AIRCR  = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
    __builtin_unreachable();
}

// Переход в приложение
__STATIC_FORCEINLINE __NO_RETURN
void goApp(){
    FLASH->SR = FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_LOCK;

    RCC->APB2RSTR = RCC_APB2RSTR_USART1RST;
    RCC->AHBRSTR = RCC_AHBRSTR_GPIOARST;
    RCC->APB2RSTR = 0;
    RCC->AHBRSTR = 0;
    RCC->AHBENR = 0x00000014;
    RCC->APB2ENR = RCC_APB2ENR_SYSCFGEN;

    for (int i = 0; i < 48; i++)
        vec[i] = app[i];

    SYSCFG->CFGR1 = SYSCFG_CFGR1_MEM_MODE;

    IWDG->KR = IWDG_REFRESH;

    asm volatile(
        "msr msp, %[sp]   \n\t"
        "bx	%[pc]         \n\t"
        :: [sp] "r" (app[0]), [pc] "r" (app[1])
    );

    __builtin_unreachable();
}

int main(){
    RCC->AHBENR = RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR = RCC_APB2ENR_USART1EN;

    #if RS485 == 1
        GPIOA->MODER |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER9_1;
        GPIOA->AFR[1] = 0x00010110;

        USART1->BRR = F_CPU / BAUDRATE;
        USART1->CR3 = USART_CR3_DEM;
        USART1->CR1 = USART_CR1_DEAT_Msk | USART_CR1_DEDT_Msk | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    #else
        GPIOA->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER9_1;
        GPIOA->AFR[1] = 0x00000110;

        USART1->BRR = F_CPU / BAUDRATE;
        USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    #endif

    IWDG->KR = IWDG_START;
    IWDG->KR = IWDG_WRITE_ACCESS;
    IWDG->PR = IWDG_PR_PR_Msk;
    IWDG->RLR = IWDG_RELOAD;

    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    #if CRYPTO == 1
        l[2] = KEY1;
        l[1] = KEY2;
        l[0] = KEY3;
        key[0] = KEY4;
        for (int i = 0; i < 26; i++) {
            l[i+3] = (key[i] + (l[i]>>8 | l[i]<<24)) ^ i;
            key[i+1] = (key[i]<<3 | key[i]>>29) ^ l[i+3];
        }
    #endif

    uartWrite(0xFE);
    uartWrite(0xE1);
    uartWrite(0xDE);
    uartWrite(0xAD);

    for(int count = 0; count < 3; count++){
        uint16_t c = uartRead();
        if(c != 0xDE) continue;

        c = uartRead();
        if(c != 0xAD) continue;

        c = uartRead();
        if(c == 0x00){
            const char* str = "Zerg17 Bootloader v1.0\r\n";
            while(*str) uartWrite(*str++);
            continue;
        }
        if(c != 0xBE) continue;

        c = uartRead();
        if(c != 0xEF) continue;

        uint32_t pageCtx = uartRead();
        if((pageCtx == 0x100 || !(pageCtx) || (pageCtx) >= PAGES) && (pageCtx != 127)) continue;

        uint16_t crc = uartRead();
        if(crc == 0x100) continue;

        if(uartPageRead() != crc) continue;

        #if CRYPTO == 1
            for(int i = 0; i < 256; i+=2)
                speck_decrypt(key, &(page.p32[i]));
        #endif

        if(pageCtx == 127){
            bootloaderSelfUpdate();
        }else{
            flashSectorClear(FLASH_BASE + pageCtx * 1024);
            for(uint32_t i = 0; i < 512; i++)
                flashWrite(FLASH_BASE + pageCtx * 1024 + i * 2, page.p16[i]);
        }

        count = 0;
        uartWrite(0xAA); // Отправка ACK
    }

    goApp();
}
