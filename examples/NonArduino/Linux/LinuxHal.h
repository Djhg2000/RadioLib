#ifndef LINUX_HAL_H
#define LINUX_HAL_H

#include <RadioLib.h>

// Linux limits C library
extern "C"
{
	#include <linux/limits.h>
}

// Standard Linux GPIO C++ library
#include <gpiod.hpp>
// Define libgpiod consumer name
#define GPIOD_CONSUMER "RadioLib"
// Define libgpiod flags
#define GPIOD_FLAGS 0
// Define inactive and active values
#define GPIO_LINE_VALUE_INACTIVE 0
#define GPIO_LINE_VALUE_ACTIVE   1
// Define direction values
#define GPIO_LINE_DIRECTION_OUTPUT 0
#define GPIO_LINE_DIRECTION_INPUT  1
// Define rising and falling edge values
#define GPIO_LINE_EDGE_FALLING 0
#define GPIO_LINE_EDGE_RISING  1

// Standard Linux SPI C library, with C dependencies
extern "C"
{
	#include <fcntl.h>
	#include <unistd.h>
	#include <sys/ioctl.h>
	#include <linux/spi/spidev.h>
}
// CPOL = 1, CPHA = 0, CS active high
#define SPI_CONF_MODE SPI_MODE_0 | SPI_CS_HIGH
// 8 bits per character
#define SPI_CONF_BITS_PER_WORD 8
// 1.5 MHz SPI bus speed
#define SPI_CONF_MAX_SPEED_HZ 1500000

// Delay related functions
#include <chrono>
#include <thread>

// Debugging
#include <iostream>

struct gpioConfig_t {
	::std::string dev;
	unsigned int lineReset;
	unsigned int lineBusy;
};

struct spiConfig_t {
	::std::string dev;
	uint32_t speed = SPI_CONF_MAX_SPEED_HZ;
};

// create a new Linux hardware abstraction layer
// using the libgpiod (C++) and spidev (C) libraries
// the HAL must inherit from the base RadioLibHal class
// and implement all of its virtual methods
class LinuxHal : public RadioLibHal {
	public:
		// default constructor
		// initializes the base HAL and any needed private members
		LinuxHal(gpioConfig_t gpioConfig, spiConfig_t spiConfig)
			: RadioLibHal(GPIO_LINE_DIRECTION_INPUT,
				GPIO_LINE_DIRECTION_OUTPUT,
				GPIO_LINE_VALUE_INACTIVE,
				GPIO_LINE_VALUE_ACTIVE,
				GPIO_LINE_EDGE_RISING,
				GPIO_LINE_EDGE_FALLING
			),
			_gpioConfig(gpioConfig),
			_spiConfig(spiConfig) {
		}

		void interruptEmulationThread() {}

		void init() override {
			// GPIO Linux interface
			// GPIO chip
			::gpiod::chip _gpioHandle = ::gpiod::chip(_gpioConfig.dev);
			// This trades some wasted memory for much faster pin access
			_gpioLines.resize(_gpioHandle.num_lines());

			// SPI Linux interface
			spiBegin();
		}

		void term() override {
			// stop the SPI
			spiEnd();

			// Iterate over the vector of line objects and release them all
			for (uint64_t i = _gpioLines.size()-1; i <= 0; i--)
			{
				if(_gpioLines[i].is_requested() == true)
					_gpioLines[i].release();
			}
			// finally, stop the pigpio library
			_gpioHandle.~chip();
		}

		// GPIO-related methods (pinMode, digitalWrite etc.) should check
		// RADIOLIB_NC as an alias for non-connected pins
		void pinMode(uint32_t pin, uint32_t mode) override {
			if(pin == RADIOLIB_NC) {
				return;
			}

			// libgpiod expects the mode to be a signed int
			int _gpioMode;
			if(mode == GPIO_LINE_DIRECTION_INPUT)
				_gpioMode = ::gpiod::line::DIRECTION_INPUT;
			else if(mode == GPIO_LINE_DIRECTION_OUTPUT)
				_gpioMode = ::gpiod::line::DIRECTION_OUTPUT;
			// libgpiod expects the pin to be an unsigned int
			_gpioLines[pin] = _gpioHandle.get_line(static_cast<unsigned int>(pin));
			if(_gpioLines[pin].is_requested() == false)
				_gpioLines[pin].request({GPIOD_CONSUMER,
					_gpioMode,
					GPIOD_FLAGS
				},
				GPIO_LINE_VALUE_INACTIVE);
			else
				_gpioLines[pin].set_config(_gpioMode,
					GPIOD_FLAGS,
					GPIO_LINE_VALUE_INACTIVE
				);
		}

		void digitalWrite(uint32_t pin, uint32_t value) override {
			if(pin == RADIOLIB_NC) {
				return;
			}

			// libgpiod expects the value to be a signed int
			_gpioLines[pin].set_value(static_cast<int>(value));
		}

		uint32_t digitalRead(uint32_t pin) override {
			if(pin == RADIOLIB_NC) {
				return(0);
			}

			// libgpiod returns the value as a signed int
			return static_cast<uint32_t>(_gpioLines[pin].get_value());
		}

		void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
			if(interruptNum == RADIOLIB_NC) {
				return;
			}

			//#FIXME sort out threading
			//std::thread iet (interruptEmulationThread, interruptNum, mode, 0, interruptCb);
			//gpioSetISRFunc(interruptNum, mode, 0, (gpioISRFunc_t)interruptCb);
		}

		void detachInterrupt(uint32_t interruptNum) override {
			if(interruptNum == RADIOLIB_NC) {
				return;
			}

			//#FIXME sort out threading
			//gpioSetISRFunc(interruptNum, 0, 0, NULL);
		}

		void delay(unsigned long ms) override {
			::std::this_thread::sleep_for(::std::chrono::milliseconds(ms));
		}

		void delayMicroseconds(unsigned long us) override {
			::std::this_thread::sleep_for(::std::chrono::microseconds(us));
		}

		unsigned long millis() override {
			return (
				std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count()
				);
		}

		unsigned long micros() override {
			return (
				std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count()
				);
		}

		long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
			if(pin == RADIOLIB_NC) {
				return(0);
			}

			this->pinMode(pin, GPIO_LINE_DIRECTION_INPUT);
			uint32_t start = this->micros();
			uint32_t curtick = this->micros();

			while(this->digitalRead(pin) == state) {
				if((this->micros() - curtick) > timeout) {
					return(0);
				}
			}

			return(this->micros() - start);
		}

		void spiBegin() {
			if(_spiBegun == true) return;

			int _spiHandle = open(_spiConfig.dev.c_str(), O_RDWR);

			if (_spiHandle < 0) {
				// If we get here, something went wrong. Abort.
				close(_spiHandle);
				return;
			}

			// SPI device setup
			const uint8_t _spiConfMode = SPI_CONF_MODE;
			const uint32_t _spiConfBitsPerWord = SPI_CONF_BITS_PER_WORD;
			ioctl(_spiHandle, SPI_IOC_WR_MODE, &_spiConfMode);
			ioctl(_spiHandle, SPI_IOC_WR_BITS_PER_WORD, &_spiConfBitsPerWord);
			ioctl(_spiHandle, SPI_IOC_WR_MAX_SPEED_HZ, &_spiConfig.speed);

			// SPI up and running
			_spiBegun = true;
		}

		void spiBeginTransaction() {}

		void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
			struct spi_ioc_transfer ioc_tr[1] =
			{
				{
					.tx_buf = (unsigned long)out,
					.rx_buf = (unsigned long)in,
					.len = (uint32_t)len,
					.speed_hz = SPI_CONF_MAX_SPEED_HZ,
				},
			};
			int err = ioctl(_spiHandle, SPI_IOC_MESSAGE(1), &(ioc_tr));

			if (err < 1) ::std::cerr << "SPI transfer failed" << ::std::endl;
		}

		void spiEndTransaction() {}

		void spiEnd() {
			if(_spiBegun == true) {
				close(_spiHandle);
				_spiBegun == false;
			}
		}

	private:
		// the HAL can contain any additional private members
		const gpioConfig_t _gpioConfig;
		::gpiod::chip _gpioHandle;
		::std::vector<::gpiod::line> _gpioLines;
		const spiConfig_t _spiConfig;
		int _spiHandle = -1;
		bool _spiBegun = false;
};

#endif
