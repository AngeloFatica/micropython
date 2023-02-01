#define MICROPY_HW_BOARD_NAME       "V3-MOBO"
#define MICROPY_HW_MCU_NAME         "STM32F405RG"

#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (0)
#define MICROPY_HW_HAS_MMA7660      (0)
#define MICROPY_HW_HAS_LCD          (0)
#define MICROPY_HW_ENABLE_RNG       (0)
#define MICROPY_HW_ENABLE_RTC       (0)
#define MICROPY_HW_ENABLE_SERVO     (0)
#define MICROPY_HW_ENABLE_DAC       (0)
#define MICROPY_HW_ENABLE_USB       (1)
#define MICROPY_HW_ENABLE_SDCARD    (0)

// HSE is 12MHz
#define MICROPY_HW_CLK_PLLM (6)
#define MICROPY_HW_CLK_PLLN (72)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ (3)
#define MICROPY_HW_CLK_LAST_FREQ (1)

// The pyboard has a 32kHz crystal for the RTC
#define MICROPY_HW_RTC_USE_LSE      (0)
#define MICROPY_HW_RTC_USE_US       (0)
#define MICROPY_HW_RTC_USE_CALOUT   (0)

// UART config
#define MICROPY_HW_UART1_TX     (pin_A9)
#define MICROPY_HW_UART1_RX     (pin_A10)
#define MICROPY_HW_UART2_TX     (pin_A2)
#define MICROPY_HW_UART2_RX     (pin_A3)
#define MICROPY_HW_UART2_RTS    (pin_A1) //maybe comment this out because it's POWERBUTTON
#define MICROPY_HW_UART2_CTS    (pin_A0)
#define MICROPY_HW_UART3_TX     (pin_C10)
#define MICROPY_HW_UART3_RX     (pin_C11)
#define MICROPY_HW_UART3_RTS    (pin_B14) //commented out because it's green led
#define MICROPY_HW_UART3_CTS    (pin_B13) //commented out because it's yellow led
#define MICROPY_HW_UART5_TX     (pin_C12)
#define MICROPY_HW_UART5_RX     (pin_D2)
#define MICROPY_HW_UART6_TX     (pin_C6)
#define MICROPY_HW_UART6_RX     (pin_C7)

// SPI buses
#define MICROPY_HW_SPI1_NAME "X"
#define MICROPY_HW_SPI1_NSS  (pin_A4) // X5
#define MICROPY_HW_SPI1_SCK  (pin_A5) // X6
#define MICROPY_HW_SPI1_MISO (pin_A6) // X7
#define MICROPY_HW_SPI1_MOSI (pin_A7) // X8
#define MICROPY_HW_SPI2_NAME "Y"
#define MICROPY_HW_SPI2_NSS  (pin_B12) // Y5 //commented out because it's blue led
#define MICROPY_HW_SPI2_SCK  (pin_B13) // Y6 //commented out because it's yellow led
#define MICROPY_HW_SPI2_MISO (pin_B14) // Y7 //commented out because it's green led
#define MICROPY_HW_SPI2_MOSI (pin_B15) // Y8

// CAN buses
#define MICROPY_HW_CAN1_NAME "YA"
#define MICROPY_HW_CAN1_TX   (pin_B9) // Y4
#define MICROPY_HW_CAN1_RX   (pin_B8) // Y3
#define MICROPY_HW_CAN2_NAME "YB"
#define MICROPY_HW_CAN2_TX   (pin_B13) // Y6
#define MICROPY_HW_CAN2_RX   (pin_B12) // Y5

// I2C buses
#define MICROPY_HW_I2C1_SCL (pin_B6)
#define MICROPY_HW_I2C1_SDA (pin_B7)
#define MICROPY_HW_I2C2_SCL (pin_B10)
#define MICROPY_HW_I2C2_SDA (pin_B11)

// USRSW has no pullup or pulldown, and pressing the switch makes the input go low
#define MICROPY_HW_USRSW_PIN        (pin_B3)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)


// The pyboard has 4 LEDs
#define MICROPY_HW_LED1             (pin_C8) // red
#define MICROPY_HW_LED2             (pin_B14) // green
#define MICROPY_HW_LED3             (pin_B13) // yellow
#define MICROPY_HW_LED4             (pin_B12)  // blue
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_high(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_low(pin))


// USB config
#define MICROPY_HW_USB_FS              (1)

void customPins(void);
#define MICROPY_BOARD_EARLY_INIT customPins
