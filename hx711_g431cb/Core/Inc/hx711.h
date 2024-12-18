#ifndef APPLICATION_CORE_HX711_H_
#define APPLICATION_CORE_HX711_H_

#include "stdint.h"
#include "stdbool.h"
#include "main.h"

#define CHANNEL_A 0
#define CHANNEL_B 1
#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

typedef struct
{
  GPIO_TypeDef  *clk_gpio;
  GPIO_TypeDef  *dat_gpio;
  uint16_t      clk_pin;
  uint16_t      dat_pin;
  int16_t       	Aoffset;
  float         Ascale;
  uint8_t		Again;
  int16_t       	Boffset;
  float         Bscale;
  uint8_t		Bgain;

}hx711_t;

/* Setup functions */
void hx711_init(hx711_t *hx711, GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin);
void set_scale(hx711_t *hx711, float Ascale, float Bscale);
void set_gain(hx711_t *hx711, uint8_t Again, uint8_t Bgain);

/* Load cell reading functions */
int16_t tare(hx711_t *hx711, uint8_t times, uint8_t channel);
void tare_all(hx711_t *hx711, uint8_t times);
float get_weight(hx711_t *hx711, int8_t times, uint8_t channel);

/* Others */
void set_offset(hx711_t *hx711, int16_t offset, uint8_t channel);
uint8_t shiftIn(hx711_t *hx711, uint8_t bit_count);
bool is_ready(hx711_t *hx711);
void wait_ready(hx711_t *hx711);
int16_t read(hx711_t *hx711, uint8_t channel);
int16_t read_average(hx711_t *hx711, int8_t times, uint8_t channel);
int16_t get_value(hx711_t *hx711, int8_t times, uint8_t channel);


#endif /* APPLICATION_CORE_HX711_H_ */
