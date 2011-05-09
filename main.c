#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "spinuptimings.h"

#define GET(port, pin) \
  (((port)>>(pin))&0x01)

#define HIGH 0b00000000
#define LOW  0b00000011
#define OFF  0b00000010

#define STATE(a, b, c) \
  (((a) << 4) | ((b) << 2) | (c))

#define DRV_PORT  PORTA
#define DRV_MASK  0b11111100
#define DRV_SHIFT 2

#define BEMF_APORT PINA
#define BEMF_BPORT PINB
#define BEMF_CPORT PINA
#define BEMF_APIN 1
#define BEMF_BPIN 6
#define BEMF_CPIN 0

/* time limit before stall-handler is activated */
#define COM_PERIOD_MAX 1000 /* 6 comm/cycle * 1000 ticks/comm * 100us/tick = 300ms/cycle, or ~1.66Hz = 100RPM */

/* Driver output format:
 * 7   6   5   4   3   2   1   0
 * 0   0   AH  AL  BH  BL  CH  CL */
static const uint8_t wavetable[7] = {
		STATE(OFF,  LOW,  LOW),  /* phase 0 */
		STATE(HIGH, OFF,  LOW),  /* phase 1 */
		STATE(HIGH, HIGH, OFF),  /* phase 2 */
		STATE(OFF,  HIGH, HIGH), /* phase 3 */
		STATE(LOW,  OFF,  HIGH), /* phase 4 */
		STATE(LOW,  LOW,  OFF),  /* phase 5 */
		STATE(OFF,  OFF,  OFF)   /* disabled/error */
};

/* Back EMF input format:
 * 7   6   5   4   3   2   1   0
 * 0   0   0   0   0   A   B   C */
static inline uint8_t statemap(uint8_t state) {
    switch (state) {
        case 0b00000000:
            return 0x00;
        case 0b00000100:
            return 0x01;
        case 0b00000110:
            return 0x02;
        case 0b00000111:
            return 0x03;
        case 0b00000011:
            return 0x04;
        case 0b00000001:
            return 0x05;
        default:
            return 0x06;
    }
}

static inline void bldc_waveout(uint8_t index) {
	uint8_t bits = DRV_PORT;
	uint8_t wave = wavetable[index];
	DRV_PORT = ((wave << DRV_SHIFT) & DRV_MASK) | (bits & ~DRV_MASK);
}

static inline uint8_t bldc_sense() {
	uint8_t state;
	state  = GET(BEMF_APORT, BEMF_APIN) << 2;
	state |= GET(BEMF_BPORT, BEMF_BPIN) << 1;
	state |= GET(BEMF_CPORT, BEMF_CPIN);
	return statemap(state);
}

volatile uint8_t  waveindex = 0x06;
volatile uint8_t  stalled   = 1;
/* timers are in units of 100us */
volatile uint16_t commtimer = 0;
volatile uint16_t spintimer = 0;

ISR(PCINT_vect) {
    if (!stalled) {
        commtimer   = 0;
        waveindex   = bldc_sense();
        bldc_waveout(waveindex);
    }
}

/* 100us interrupt */
ISR(TIMER0_COMPA_vect) {
    commtimer++;
    spintimer++;
}

void init_ports() {
	DDRA  = 0b11111100;
	PORTA = 0b10101000;
	DDRB  = 0b00001000;
	PORTB = 0b00001000;

	GIMSK  = _BV(PCIE0)  | _BV(PCIE1);  /* enable pin-change interrupts */
	PCMSK0 = _BV(PCINT0) | _BV(PCINT1);
	PCMSK1 = _BV(PCINT14);
}

void init_timers() {
    /* main timer, increments software counters at 100us intervals */
    TCCR0A = _BV(WGM00);  /* Clear timer on compare match mode */
    TCCR0B = _BV(CS01);   /* clk/8 prescale factor */
    OCR0A  = 250;         /* 250 / (20MHz / 8) = 100us */
    TIMSK  = _BV(OCIE0A); /* Enable compare match A interrupt */
}

void spinup() {
    uint16_t time;

    spintimer = 0;
    for (uint16_t i=0; i<SPINUP_TIMINGS_SIZE; i++) {
        time = spinup_timings[i];
        while (spintimer < time) { wdt_reset(); }

        waveindex = (waveindex + 1) % 6;
        bldc_waveout(waveindex);
    }
}

int main(void) {
    init_ports();
    init_timers();

    sei();

    for (;;) {
        if (commtimer > COM_PERIOD_MAX) {
            stalled = 1;
        }
        if (stalled) {
            spinup();
            stalled = 0;
        }
        wdt_reset();
    }
}
