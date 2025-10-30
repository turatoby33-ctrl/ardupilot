#pragma once

#include <cstdint>
#include <string>

namespace CustomCopter {
namespace HAL {

// ============================================================================
// GPIO Interface
// ============================================================================
class GPIO {
public:
    enum PinMode {
        INPUT,
        OUTPUT,
        INPUT_PULLUP,
        INPUT_PULLDOWN
    };

    virtual ~GPIO() = default;

    virtual void pinMode(uint8_t pin, PinMode mode) = 0;
    virtual void digitalWrite(uint8_t pin, bool value) = 0;
    virtual bool digitalRead(uint8_t pin) = 0;
};

// ============================================================================
// PWM Interface
// ============================================================================
class PWM {
public:
    virtual ~PWM() = default;

    // Initialize PWM channel
    virtual bool init(uint8_t channel, uint32_t frequency_hz) = 0;

    // Set PWM output (microseconds)
    virtual void write(uint8_t channel, uint16_t pulse_us) = 0;

    // Enable/disable PWM output
    virtual void enable(uint8_t channel, bool enabled) = 0;
};

// ============================================================================
// UART Interface
// ============================================================================
class UART {
public:
    virtual ~UART() = default;

    // Initialize UART
    virtual bool init(uint32_t baud_rate) = 0;

    // Read/write operations
    virtual int available() = 0;
    virtual uint8_t read() = 0;
    virtual void write(uint8_t byte) = 0;
    virtual void write(const uint8_t* buffer, size_t length) = 0;

    // Flush buffers
    virtual void flush() = 0;
};

// ============================================================================
// I2C Interface
// ============================================================================
class I2C {
public:
    virtual ~I2C() = default;

    // Initialize I2C bus
    virtual bool init(uint32_t frequency_hz) = 0;

    // Read/write operations
    virtual bool read(uint8_t address, uint8_t reg, uint8_t* data, size_t length) = 0;
    virtual bool write(uint8_t address, uint8_t reg, const uint8_t* data, size_t length) = 0;

    // Register operations
    virtual uint8_t readRegister(uint8_t address, uint8_t reg) = 0;
    virtual bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) = 0;
};

// ============================================================================
// SPI Interface
// ============================================================================
class SPI {
public:
    virtual ~SPI() = default;

    // Initialize SPI bus
    virtual bool init(uint32_t frequency_hz) = 0;

    // Transfer data
    virtual void transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length) = 0;

    // Chip select control
    virtual void setCS(uint8_t pin, bool active) = 0;
};

// ============================================================================
// Scheduler Interface
// ============================================================================
class Scheduler {
public:
    virtual ~Scheduler() = default;

    // Time functions
    virtual uint64_t micros() = 0;
    virtual uint64_t millis() = 0;
    virtual void delay_microseconds(uint32_t us) = 0;
    virtual void delay_milliseconds(uint32_t ms) = 0;

    // Thread creation (optional)
    virtual bool create_thread(void (*func)(void*), void* arg, const char* name,
                               size_t stack_size, uint8_t priority) = 0;
};

// ============================================================================
// Util Interface
// ============================================================================
class Util {
public:
    virtual ~Util() = default;

    // System information
    virtual uint32_t available_memory() = 0;
    virtual void panic(const char* message) = 0;

    // CRC calculations
    virtual uint16_t crc16_ccitt(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF) = 0;
};

// ============================================================================
// HAL Interface - Main HAL class
// ============================================================================
class HAL_Interface {
public:
    virtual ~HAL_Interface() = default;

    // Hardware interfaces
    virtual GPIO* get_gpio() = 0;
    virtual PWM* get_pwm() = 0;
    virtual UART* get_uart(uint8_t instance) = 0;
    virtual I2C* get_i2c(uint8_t instance) = 0;
    virtual SPI* get_spi(uint8_t instance) = 0;
    virtual Scheduler* get_scheduler() = 0;
    virtual Util* get_util() = 0;

    // System control
    virtual void init() = 0;
    virtual void reboot() = 0;
};

// ============================================================================
// Global HAL instance
// ============================================================================
extern HAL_Interface* hal;

// Initialize HAL (platform specific)
HAL_Interface* create_hal();

} // namespace HAL
} // namespace CustomCopter
