#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "dht11.h"

#define LCD_RS_PORT PORTB
#define LCD_RS_PIN 0
#define LCD_E_PORT PORTB
#define LCD_E_PIN 3
#define LCD_DATA_PORT PORTC
#define LCD_D4_PIN 1
#define LCD_D5_PIN 2
#define LCD_D6_PIN 3
#define LCD_D7_PIN 4
#define LED_PIN PB5
#define SOIL_SENSOR_CHANNEL 0
#define RELAY_PIN PD1
#define DHT11_PIN 4
#define MOISTURE_THRESHOLD 550 // Calibrate: >550 = DRY, <550 = OK
#define WATERING_DURATION 10000 // 10s
#define UPDATE_INTERVAL 2000 // 2s for testing, revert to 10000 for final

void lcd_write_nibble(uint8_t data) {
	LCD_DATA_PORT &= ~((1 << LCD_D4_PIN) | (1 << LCD_D5_PIN) |
	(1 << LCD_D6_PIN) | (1 << LCD_D7_PIN));
	if (data & 0x01) LCD_DATA_PORT |= (1 << LCD_D4_PIN);
	if (data & 0x02) LCD_DATA_PORT |= (1 << LCD_D5_PIN);
	if (data & 0x04) LCD_DATA_PORT |= (1 << LCD_D6_PIN);
	if (data & 0x08) LCD_DATA_PORT |= (1 << LCD_D7_PIN);
	LCD_E_PORT |= (1 << LCD_E_PIN);
	_delay_us(1000);
	LCD_E_PORT &= ~(1 << LCD_E_PIN);
	_delay_us(1000);
}

void lcd_write_cmd(uint8_t cmd) {
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	lcd_write_nibble(cmd >> 4);
	lcd_write_nibble(cmd & 0x0F);
	_delay_ms(2);
}

void lcd_write_data(uint8_t data) {
	LCD_RS_PORT |= (1 << LCD_RS_PIN);
	lcd_write_nibble(data >> 4);
	lcd_write_nibble(data & 0x0F);
	_delay_ms(2);
}

void lcd_init(void) {
	DDRC |= (1 << LCD_D4_PIN) | (1 << LCD_D5_PIN) |
	(1 << LCD_D6_PIN) | (1 << LCD_D7_PIN);
	DDRB |= (1 << LCD_RS_PIN) | (1 << LCD_E_PIN);
	_delay_ms(1000);
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
	lcd_write_nibble(0x03);
	_delay_ms(100);
	lcd_write_nibble(0x03);
	_delay_ms(100);
	lcd_write_nibble(0x03);
	_delay_ms(100);
	lcd_write_nibble(0x02);
	_delay_ms(100);
	lcd_write_cmd(0x28);
	_delay_ms(100);
	lcd_write_cmd(0x0C);
	_delay_ms(100);
	lcd_write_cmd(0x01);
	_delay_ms(100);
	lcd_write_cmd(0x06);
	_delay_ms(100);
}

void lcd_puts(const char *str) {
	while (*str) {
		lcd_write_data(*str++);
	}
}

void lcd_gotoxy(uint8_t x, uint8_t y) {
	uint8_t addr = (y == 0) ? 0x80 : 0xC0;
	addr += x;
	lcd_write_cmd(addr);
}

void lcd_reset(void) {
	lcd_write_cmd(0x01); // Clear display
	_delay_ms(2);
	lcd_write_cmd(0x0C); // Display on, cursor off
	_delay_ms(2);
	lcd_write_cmd(0x06);
	_delay_ms(2);
}

void adc_init(void) {
	ADMUX = (1 << REFS0); // AVcc as reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler 128
}

uint16_t adc_read(uint8_t ch) {
	ch &= 0b00000111;
	ADMUX = (ADMUX & 0xF8) | ch;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void init_io(void) {
	DDRD |= (1 << RELAY_PIN);
	DDRB |= (1 << LED_PIN);
	PORTD |= (1 << RELAY_PIN); // Relay off (active low)
	PORTB &= ~(1 << LED_PIN);  // LED off
	PORTD |= (1 << PD4);       // DHT11 internal pull-up
}

int main(void) {
	DDRC |= (1 << LCD_D4_PIN) | (1 << LCD_D5_PIN) |
	(1 << LCD_D6_PIN) | (1 << LCD_D7_PIN);
	DDRB |= (1 << LCD_RS_PIN) | (1 << LCD_E_PIN);
	DDRD |= (1 << RELAY_PIN);
	PORTD |= (1 << RELAY_PIN);

	lcd_init();
	adc_init();
	init_io();

	// Initial delay for DHT11 stability
	_delay_ms(3000);

	uint16_t last_temperature = 0, last_humidity = 0;
	uint8_t last_moisture_state = 0; // 0 = OK, 1 = DRY
	uint8_t dht_error_count = 0; // Track consecutive DHT11 errors
	while (1) {
		char buffer[16];
		uint16_t temperature = last_temperature;
		uint16_t humidity = last_humidity;
		uint16_t moisture = 0;

		// Read DHT11 with retries
		_delay_ms(2000); // Ensure 2s between reads
		dht11_device dev = dht11_get_device(DHT11_PIN);
		uint8_t dht_result = DHT11_READ_TIMEOUT;
		for (uint8_t i = 0; i < 3 && dht_result != DHT11_READ_SUCCESS; i++) {
			dht_result = dht11_read_data(dev, &humidity, &temperature);
			if (dht_result != DHT11_READ_SUCCESS) {
				_delay_ms(1000); // Wait before retry
			}
		}
		if (dht_result != DHT11_READ_SUCCESS) {
			dht_error_count++;
			if (dht_error_count >= 3) { // Show error only after 3 consecutive failures
				lcd_reset();
				lcd_gotoxy(0, 0);
				lcd_puts("DHT11 Error     ");
				lcd_gotoxy(0, 1);
				snprintf(buffer, 16, "Code: %u        ", dht_result);
				lcd_puts(buffer);
				_delay_ms(2000); // Show error briefly
			}
			temperature = last_temperature;
			humidity = last_humidity;
			} else {
			dht_error_count = 0; // Reset error count
			last_temperature = temperature;
			last_humidity = humidity;
		}

		// Read soil moisture
		moisture = adc_read(SOIL_SENSOR_CHANNEL);
		uint8_t current_moisture_state = (moisture > MOISTURE_THRESHOLD) ? 1 : 0;

		// Clear LCD before update if state changes
		if (current_moisture_state != last_moisture_state) {
			lcd_reset(); // Clear display for clean transition
			last_moisture_state = current_moisture_state;
		}

		// Update LCD
		lcd_reset(); // Clear display for smooth update
		lcd_gotoxy(0, 0);
		snprintf(buffer, 16, "Temp:%uC Hum:%u", temperature, humidity);
		lcd_puts(buffer);
		lcd_write_data('%'); // Explicitly write %

		lcd_gotoxy(0, 1);
		if (moisture > MOISTURE_THRESHOLD) {
			snprintf(buffer, 16, "Moisture: DRY   ");
			lcd_puts(buffer);
			PORTB |= (1 << LED_PIN); // LED on for low moisture
			} else {
			snprintf(buffer, 16, "Moisture: OK    ");
			lcd_puts(buffer);
			PORTB &= ~(1 << LED_PIN); // LED off
			PORTD |= (1 << RELAY_PIN); // Relay off
		}

		// Watering logic
		if (moisture > MOISTURE_THRESHOLD) {
			// Activate relay and blink LED
			PORTD &= ~(1 << RELAY_PIN); // Relay on (active low)
			for (uint16_t i = 0; i < WATERING_DURATION / 1000; i++) {
				PORTB |= (1 << LED_PIN);
				_delay_ms(500);
				PORTB &= ~(1 << LED_PIN);
				_delay_ms(500);
			}
			PORTD |= (1 << RELAY_PIN); // Relay off
		}

		_delay_ms(UPDATE_INTERVAL);
	}
}

