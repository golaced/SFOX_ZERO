#include "app_led.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern osSemaphoreId myBinarySem02LED1ONHandle;
extern osSemaphoreId myBinarySem03LED2ONHandle;
extern osSemaphoreId myBinarySem05LED1PulsateHandle;
extern osSemaphoreId myBinarySem06LED2PulsateHandle;

void app_led_thread(void)
{
  if(osOK == osSemaphoreWait(myBinarySem02LED1ONHandle,0))
  {
    led_one_on();
    osDelay(3000);
    led_one_off();
    //return;
  }
  if(osOK == osSemaphoreWait(myBinarySem03LED2ONHandle,0))
  {
    while(1)
    {
      led_all_on();
      osDelay(100);
      if(osOK == osSemaphoreWait(myBinarySem03LED2ONHandle,0))
      {
        led_all_off();
        return;
      }
        
    }
    //led_two_on();
    //osDelay(3000);
    //led_two_off();
    //return;
  }
  if(osOK == osSemaphoreWait(myBinarySem05LED1PulsateHandle,0))
  {
    led_one_toggle();
  }
  if(osOK == osSemaphoreWait(myBinarySem06LED2PulsateHandle,0))
  {
    
  }
  led_one_toggle();
  osDelay(700);
}

void led_init(void)
{
  led_all_off();
}

void led_all_on(void)
{
  led_one_on();
  led_two_on();
}

void led_all_off(void)
{
  led_one_off();
  led_two_off();
}

void led_all_toggle(void)
{
  led_one_toggle();
  led_two_toggle();
}

void led_one_on(void)
{
  HAL_GPIO_WritePin(GPIOB, LED_PB8_Pin, GPIO_PIN_RESET);
}

void led_one_off(void)
{
  HAL_GPIO_WritePin(GPIOB, LED_PB8_Pin, GPIO_PIN_SET);
}

void led_one_toggle(void)
{
  HAL_GPIO_TogglePin(GPIOB, LED_PB8_Pin);
}

void led_two_on(void)
{
  HAL_GPIO_WritePin(GPIOB, LED_PB9_Pin, GPIO_PIN_RESET);
}

void led_two_off(void)
{
  HAL_GPIO_WritePin(GPIOB, LED_PB9_Pin, GPIO_PIN_SET);
}

void led_two_toggle(void)
{
  HAL_GPIO_TogglePin(GPIOB, LED_PB9_Pin);
}


