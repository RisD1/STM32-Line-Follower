#include "switch_input.h"
volatile uint8_t switch_bits[4] = {0};
volatile uint8_t switch_value = 0;
void SwitchInput_Init(void) { SwitchInput_Update(); }
void SwitchInput_Update(void) {
    /* Order: SEL0=PA12, SEL1=PB3, SEL2=PB8, SEL3=PB9. */
    switch_bits[0] = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET) ? 1U : 0U;
    switch_bits[1] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)  == GPIO_PIN_RESET) ? 1U : 0U;
    switch_bits[2] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)  == GPIO_PIN_RESET) ? 1U : 0U;
    switch_bits[3] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)  == GPIO_PIN_RESET) ? 1U : 0U;
    switch_value = (uint8_t)(switch_bits[0] | (switch_bits[1] << 1) | (switch_bits[2] << 2) | (switch_bits[3] << 3));
}
