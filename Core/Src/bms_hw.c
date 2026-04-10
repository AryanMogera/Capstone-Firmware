
// bms_hw.c
#include "bms_hw.h"
#include "stm32f4xx.h"

#define GPIOCEN (1U << 2)

// Charge / Discharge enable pins (PC6/PC7)
#define CHG_PIN  6U
#define DSG_PIN  7U

// Nucleo USER button (B1)
#define USER_BTN_PIN 13U

static inline void gpio_pin_output(GPIO_TypeDef *GPIOx, uint32_t pin) {
  GPIOx->MODER &= ~(3U << (pin * 2U));
  GPIOx->MODER |=  (1U << (pin * 2U));   // output

  GPIOx->OTYPER &= ~(1U << pin);         // push-pull
  GPIOx->PUPDR  &= ~(3U << (pin * 2U));  // no pull

  GPIOx->OSPEEDR &= ~(3U << (pin * 2U));
  GPIOx->OSPEEDR |=  (1U << (pin * 2U)); // medium speed
}

static inline void gpio_pin_input_pullup(GPIO_TypeDef *GPIOx, uint32_t pin) {
  GPIOx->MODER &= ~(3U << (pin * 2U));   // input
  GPIOx->PUPDR &= ~(3U << (pin * 2U));
  GPIOx->PUPDR |=  (1U << (pin * 2U));   // pull-up
}

static inline void pin_write(GPIO_TypeDef *GPIOx, uint32_t pin, bool on) {
  if (on) GPIOx->BSRR = (1U << pin);
  else    GPIOx->BSRR = (1U << (pin + 16U));
}

void BMS_HW_Init(void) {
  RCC->AHB1ENR |= GPIOCEN;

  gpio_pin_output(GPIOC, CHG_PIN);
  gpio_pin_output(GPIOC, DSG_PIN);
  gpio_pin_input_pullup(GPIOC, USER_BTN_PIN);

  BMS_HW_SetChargeEnable(false);
  BMS_HW_SetDischargeEnable(false);
}

void BMS_HW_SetChargeEnable(bool on) {
  pin_write(GPIOC, CHG_PIN, on);
}

void BMS_HW_SetDischargeEnable(bool on) {
  pin_write(GPIOC, DSG_PIN, on);
}

bool BMS_HW_UserButtonPressed(void) {
  // Nucleo USER button on PC13: pressed = LOW, released = HIGH
  return ((GPIOC->IDR & (1U << USER_BTN_PIN)) == 0U);
}

