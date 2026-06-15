#if !defined(_RADIOLIB_STM32WLCUBEHAL_H)
#define _RADIOLIB_STM32WLCUBEHAL_H

#include <RadioLib.h>
#include "stm32wlxx_hal.h"


// Define standard logic states if they are missing in the HAL environment
#ifndef INPUT
#define INPUT 0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef RISING
#define RISING 0
#endif
#ifndef FALLING
#define FALLING 1
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif


/*!
  \brief Virtual pins used by RadioLib to control the internal SubGHz radio.
  These do not correspond to physical GPIOs, but are intercepted by the HAL
  to trigger internal PWR and RCC register operations.
*/
enum {
  RADIOLIB_STM32WL_VIRTUAL_PIN_NSS = 0x200,
  RADIOLIB_STM32WL_VIRTUAL_PIN_BUSY,
  RADIOLIB_STM32WL_VIRTUAL_PIN_IRQ,
  RADIOLIB_STM32WL_VIRTUAL_PIN_RESET
};

/*!
  \struct StmPinConfig_t
  \brief Structure to hold standard STM32 hardware pin configurations
  if physical peripherals need to be controlled via RadioLib.
*/
struct StmPinConfig_t {
  GPIO_TypeDef* port;
  uint16_t pin;
};

/*!
  \brief Callback function type for RF switch control.
*/
typedef void (*RfSwitchCallback_t)(bool txEnable, bool rxEnable);

/*!
  \class Stm32WlCubeHal
  \brief Hardware Abstraction Layer implementation for STM32WL using the STM32Cube HAL.
*/
class Stm32WlCubeHal : public RadioLibHal {
  public:
    /*!
      \brief Default constructor.
    */
    Stm32WlCubeHal();

    /*!
      \brief Set the hardware pin mapping array.
      \param pinMap Pointer to the array of STM32 pin configurations.
      \param size Number of elements in the array.
    */
    void setPinMap(const StmPinConfig_t* pinMap, size_t size);

    /*!
      \brief Set the callback for module-specific RF switch control.
      \param cb Function pointer to the RF switch handler.
    */
    void setRfSwitchCallback(RfSwitchCallback_t cb);

    // GPIO related methods (overridden)
    void pinMode(uint32_t pin, uint32_t mode) override;
    void digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;

    // Interrupt related methods (overridden)
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override;
    void detachInterrupt(uint32_t interruptNum) override;

    // Delay related methods (overridden)
    void delay(RadioLibTime_t ms) override;
    void delayMicroseconds(RadioLibTime_t us) override;
    RadioLibTime_t millis() override;
    RadioLibTime_t micros() override;

    // SPI related methods (overridden)
    void spiBegin() override;
    void spiBeginTransaction() override;
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
    void spiEndTransaction() override;
    void spiEnd() override;

    // Tone and generic methods (overridden, mostly unused)
    void tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration) override;
    void noTone(uint32_t pin) override;
    void yield() override;
    uint32_t pinToInterrupt(uint32_t pin) override;
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override;

  private:
    const StmPinConfig_t* _pinMap = nullptr;
    size_t _pinMapSize = 0;
    RfSwitchCallback_t _rfSwitchCb = nullptr;
};

#endif