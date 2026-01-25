#include "zf_common_typedef.h"
#include "intrins.h"


void (*uart1_irq_handler)(uint8 dat) = NULL;
void (*uart2_irq_handler)(uint8 dat) = NULL;
void (*uart3_irq_handler)(uint8 dat) = NULL;
void (*uart4_irq_handler)(uint8 dat) = NULL;

void (*tim0_irq_handler)(void) = NULL;
void (*tim1_irq_handler)(void) = NULL;
void (*tim2_irq_handler)(void) = NULL;
void (*tim3_irq_handler)(void) = NULL;
void (*tim4_irq_handler)(void) = NULL;
void (*tim11_irq_handler)(void) = NULL;

void (*int0_irq_handler)(void) = NULL;
void (*int1_irq_handler)(void) = NULL;
void (*int2_irq_handler)(void) = NULL;
void (*int3_irq_handler)(void) = NULL;
void (*int4_irq_handler)(void) = NULL;

