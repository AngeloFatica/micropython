#include STM32_HAL_H
#include <stdio.h>
#include <stdint.h>

void customPins(void)
{
	__GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitOutput;
    GPIO_InitOutput.Speed = GPIO_SPEED_HIGH;
    GPIO_InitOutput.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitOutput.Pull = GPIO_PULLUP;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
   	GPIO_InitOutput.Pin = GPIO_PIN_15;
   	HAL_GPIO_Init(GPIOA, &GPIO_InitOutput);

   	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
   	GPIO_InitOutput.Pin = GPIO_PIN_9;
   	HAL_GPIO_Init(GPIOA, &GPIO_InitOutput);

   	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
   	GPIO_InitOutput.Pin = GPIO_PIN_3;
   	HAL_GPIO_Init(GPIOA, &GPIO_InitOutput);

   	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
   	GPIO_InitOutput.Pin = GPIO_PIN_2;
   	HAL_GPIO_Init(GPIOA, &GPIO_InitOutput);

   	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
   	GPIO_InitOutput.Pin = GPIO_PIN_1;
   	HAL_GPIO_Init(GPIOA, &GPIO_InitOutput);
}