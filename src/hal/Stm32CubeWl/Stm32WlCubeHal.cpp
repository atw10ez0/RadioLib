

#include "Stm32WlCubeHal.h"


Stm32WlCubeHal::Stm32WlCubeHal()
  : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {
}

void Stm32WlCubeHal::setPinMap(const StmPinConfig_t* pinMap, size_t size) {
  _pinMap = pinMap;
  _pinMapSize = size;
}

void Stm32WlCubeHal::setRfSwitchCallback(RfSwitchCallback_t cb) {
  _rfSwitchCb = cb;
}

void Stm32WlCubeHal::pinMode(uint32_t pin, uint32_t mode) {
  // Virtual pins are configured automatically by the hardware
  switch(pin) {
    case RADIOLIB_STM32WL_VIRTUAL_PIN_NSS:
    case RADIOLIB_STM32WL_VIRTUAL_PIN_BUSY:
    case RADIOLIB_STM32WL_VIRTUAL_PIN_IRQ:
    case RADIOLIB_STM32WL_VIRTUAL_PIN_RESET:
      return;
    default:
      break;
  }

  // Handle physical pins via the pin map
  if (_pinMap != nullptr && pin < _pinMapSize) {
    // Removed "{0}" initialization to resolve -Wmissing-field-initializers warning in C++20
    GPIO_InitTypeDef cfg;
    cfg.Pin = _pinMap[pin].pin;
    cfg.Mode = (mode == OUTPUT) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    cfg.Pull = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;
    cfg.Alternate = 0;
    HAL_GPIO_Init(_pinMap[pin].port, &cfg);
  }
}

void Stm32WlCubeHal::digitalWrite(uint32_t pin, uint32_t value) {
  switch (pin) {
    case RADIOLIB_STM32WL_VIRTUAL_PIN_NSS:
      // The SubGHz NSS is active LOW. Managed via the Power Control Register (PWR)
      if (value == LOW) {
        PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS;
      } else {
        PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;
      }
      return;

    case RADIOLIB_STM32WL_VIRTUAL_PIN_RESET:
      // SubGHz Radio Reset is managed via the RCC Control/Status Register
      if (value == LOW) {
        RCC->CSR |= RCC_CSR_RFRST;
      } else {
        RCC->CSR &= ~RCC_CSR_RFRST;
      }
      return;

    case RADIOLIB_STM32WL_VIRTUAL_PIN_BUSY:
    case RADIOLIB_STM32WL_VIRTUAL_PIN_IRQ:
      // Read-only internal signals; ignore writes
      return;

    default:
      break;
  }

  // Handle physical pins via the pin map
  if (_pinMap != nullptr && pin < _pinMapSize) {
    HAL_GPIO_WritePin(_pinMap[pin].port, _pinMap[pin].pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

uint32_t Stm32WlCubeHal::digitalRead(uint32_t pin) {
  switch (pin) {
    case RADIOLIB_STM32WL_VIRTUAL_PIN_BUSY:
      // The RFBUSYS flag in the Power Control Register 2 indicates the radio state
      return (PWR->SR2 & PWR_SR2_RFBUSYS) ? HIGH : LOW;

    case RADIOLIB_STM32WL_VIRTUAL_PIN_IRQ: {
      // The internal Radio IRQ is mapped directly to the NVIC (Interrupt 50)
      // Polling the pending state effectively reads the hardware IRQ line
      bool pending = HAL_NVIC_GetPendingIRQ(SUBGHZ_Radio_IRQn);
      #if RADIOLIB_DEBUG
      if(pending) {
        // RADIOLIB_DEBUG_BASIC_PRINTLN("IRQ Pending detected");
      }
      #endif
      return pending ? HIGH : LOW;
    }

    case RADIOLIB_STM32WL_VIRTUAL_PIN_NSS:
      return (PWR->SUBGHZSPICR & PWR_SUBGHZSPICR_NSS) ? HIGH : LOW;

    case RADIOLIB_STM32WL_VIRTUAL_PIN_RESET:
      // If the hardware reset bit is currently active, the logic state is LOW
      return (RCC->CSR & RCC_CSR_RFRST) ? LOW : HIGH;

    default:
      break;
  }

  // Handle physical pins via the pin map
  if (_pinMap != nullptr && pin < _pinMapSize) {
    return HAL_GPIO_ReadPin(_pinMap[pin].port, _pinMap[pin].pin) == GPIO_PIN_SET ? HIGH : LOW;
  }

  return LOW;
}

void Stm32WlCubeHal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) {
  // RadioLib handles internal polling via digitalRead() on the VIRTUAL_PIN_IRQ.
  // Standard GPIO EXTI attachments are not used for the internal radio.
  (void)interruptNum;
  (void)interruptCb;
  (void)mode;
}

void Stm32WlCubeHal::detachInterrupt(uint32_t interruptNum) {
  (void)interruptNum;
}

void Stm32WlCubeHal::delay(RadioLibTime_t ms) {
  HAL_Delay(ms);
}

void Stm32WlCubeHal::delayMicroseconds(RadioLibTime_t us) {
  uint32_t start = this->micros();
  while (this->micros() - start < us) {
    this->yield();
  }
}

RadioLibTime_t Stm32WlCubeHal::millis() {
  return HAL_GetTick();
}

RadioLibTime_t Stm32WlCubeHal::micros() {
  uint32_t ms = HAL_GetTick();
  uint32_t load = SysTick->LOAD;
  uint32_t val = SysTick->VAL;

  // Check for overflow
  if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) && (val > (load / 2))) {
    ms = HAL_GetTick();
    val = SysTick->VAL;
  }

  return (RadioLibTime_t)ms * 1000 + (load - val) / (SystemCoreClock / 1000000);
}

void Stm32WlCubeHal::spiBegin() {
  // Initialization is provided by MX_SUBGHZ_Init() in CubeMX
}

void Stm32WlCubeHal::spiBeginTransaction() {
}

void Stm32WlCubeHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
  // Direct register access for optimal performance on the internal SPI bus
  for (size_t i = 0; i < len; i++) {
    uint8_t dataOut = out ? out[i] : 0x00;

    // Wait until TX buffer is empty
    while ((SUBGHZSPI->SR & SPI_SR_TXE) == 0);
    *((__IO uint8_t *)&SUBGHZSPI->DR) = dataOut;

    // Wait until RX buffer has data
    while ((SUBGHZSPI->SR & SPI_SR_RXNE) == 0);
    uint8_t dataIn = *((__IO uint8_t *)&SUBGHZSPI->DR);

    if (in) {
      in[i] = dataIn;
    }
  }
}

void Stm32WlCubeHal::spiEndTransaction() {
}

void Stm32WlCubeHal::spiEnd() {
}

void Stm32WlCubeHal::tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration) {
  (void)pin;
  (void)frequency;
  (void)duration;
}

void Stm32WlCubeHal::noTone(uint32_t pin) {
  (void)pin;
}

void Stm32WlCubeHal::yield() {
}

uint32_t Stm32WlCubeHal::pinToInterrupt(uint32_t pin) {
  return pin;
}


long Stm32WlCubeHal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) {
  (void)pin;
  (void)state;
  (void)timeout;
  return 0;
}