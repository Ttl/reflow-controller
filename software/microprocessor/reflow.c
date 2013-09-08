#include "reflow.h"
#define SS 0
#define SCK 1
#define MOSI 2
#define MISO 3

static volatile bool tx_flag = 0;
static volatile bool usb_connected = 0;
static volatile uint16_t temp = 0;
static volatile uint16_t room_temp = 0;
static volatile uint16_t target = 0;
static volatile uint16_t prev_target = 0;
static volatile uint16_t timer = 0;/* Timer for various reflow stages */
static profile_t profile;
static profile_t EEMEM eeprom_profile = {
        .start_rate = 1*4,
        .soak_temp1 = 100*4,
        .soak_temp2 = 110*4,
        .soak_length = 100,
        .peak_temp = 200*4,
        .time_to_peak = 80,
        .cool_rate = 2*4,
        .pid_p = 800,
        .pid_i = 400,
        .pid_d = 0
};

static state_enum reflow_state;
static int16_t integral = 0;
static int16_t last_error = 0;
static int8_t PID_debug = 0;
static volatile uint8_t target_update = 0;

static FILE USBSerialStream;

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = 0,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

void set_profile(void) {
    /* Read profile from EEPROM to RAM */
    eeprom_read_block(&profile, &eeprom_profile, sizeof(profile));
}

void write_profile(void) {
    /* Read profile from PC and replace current profile in RAM and EEPROM */
    uint8_t settings[20];
    uint8_t i=0;
    int ReceivedChar;
    for(;;) {
        if ((ReceivedChar = fgetc(&USBSerialStream)) != EOF) {
            settings[i++] = (uint8_t)ReceivedChar;
            if (i==sizeof(profile))
                break;
        }
    }

#define TOU16(x,k) ( (((uint16_t)x[k])<<8)|((uint16_t)x[k+1]) )

    /* Change the current settings in RAM */
    profile.start_rate = TOU16(settings,0);
    profile.soak_temp1 = TOU16(settings,2);
    profile.soak_temp2 = TOU16(settings,4);
    profile.soak_length = TOU16(settings,6);
    profile.peak_temp = TOU16(settings,8);
    profile.time_to_peak = TOU16(settings,10);
    profile.cool_rate = TOU16(settings,12);
    profile.pid_p = TOU16(settings,14);
    profile.pid_i = TOU16(settings,16);
    profile.pid_d = TOU16(settings,18);
    /* Write all settings as one block */
    eeprom_update_block(&profile, &eeprom_profile ,sizeof(profile));
}

void output_profile(void) {
    /* Print current profile through USB */
    fprintf(&USBSerialStream, "!%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
            profile.start_rate,
            profile.soak_temp1,
            profile.soak_temp2,
            profile.soak_length,
            profile.peak_temp,
            profile.time_to_peak,
            profile.cool_rate,
            profile.pid_p,
            profile.pid_i,
            profile.pid_d);
}

/* Get the target temperature */
uint16_t target_temp(void) {
    uint16_t target = 0;
    switch(reflow_state) {
        case(T_STOP):
            target = 0;
            break;
        case(T_START):
            target = prev_target + profile.start_rate;
            /* Clamp the target value and avoid it getting too big, if the oven
             * isn't yet on. */
            target = CLAMP(target,room_temp,MIN(profile.soak_temp1,temp+profile.start_rate*5));
            if (temp > profile.soak_temp1 - 4*5) {
                timer = 0;
                reflow_state = T_SOAK;
            }
            break;
        case(T_SOAK):
            /* Linear interpolation from soak_temp1 to soak_temp2 in soak_length
             * seconds */
            if (timer < profile.soak_length) {
                target = profile.soak_temp1 +
                   (timer*(profile.soak_temp2-profile.soak_temp1))/profile.soak_length;
            } else {
                target = profile.soak_temp2;
                if (temp > profile.soak_temp2 - 4*10) {
                    timer = 0;
                    reflow_state = T_PEAK;
                }
            }
            break;
        case(T_PEAK):
            if (timer < profile.time_to_peak) {
                target = profile.soak_temp2 +
                   (timer*(profile.peak_temp-profile.soak_temp2))/profile.time_to_peak;
            } else {
                target = profile.peak_temp;
                if (temp > target) {
                    timer = 0;
                    integral = 0; /* Zero integral term of PID for faster response */
                    reflow_state = T_COOL;
                }
            }
            break;
        case(T_COOL):
            target = prev_target-profile.cool_rate;
            if (target < room_temp) {
                reflow_state = T_STOP;
            }
            break;
        default:
            /* Invalid state */
            reflow_state = T_STOP;
            break;
    }
    prev_target = target;
    return target;
}



void usb_rx(void) {
    /*  Handle messages from host */
    char ReceivedChar;
    int ReceivedByte;
    /* Start commands with '!' */
    if ( (ReceivedChar = fgetc(&USBSerialStream)) != '!') {
        return;
    }
    /* Get the real command */
    while((ReceivedByte = fgetc(&USBSerialStream)) == EOF);
    ReceivedChar = (char)ReceivedByte;
    /* PID debugging, prints PID term values */
    if (ReceivedChar == 'D') {
        PID_debug = 1;
    }
    if (ReceivedChar == 'd') {
        PID_debug = 0;
    }
    /* Write temperature profile and PID settings */
    if (ReceivedChar == 'W') {
        write_profile();
    }
    /* Start reflow */
    if (ReceivedChar == 'S') {
        reflow_state = T_START;
    }
    /* Stop reflow */
    if (ReceivedChar == 'H') {
        reflow_state = T_STOP;
    }
    /* Output current profile */
    if (ReceivedChar == 'O') {
        output_profile();
    }
    return;
}

void read_sensor(void) {
/* Bits:
 * 31 : sign,
 * 30 - 18 : thermocouple temperature,
 * 17 : reserved(0),
 * 16 : 1 if fault,
 * 15 - 4 : cold junction temperature,
 * 3 : reserved(0),
 * 2 : 1 if thermocouple is shorted to Vcc,
 * 1 : 1 if thermocouple is shorted to ground,
 * 0 : 1 if thermocouple is open circuit */

    /* Enable slave */
    uint8_t sensor[4];
    int8_t i;
    /* SS = 0 */
    PORTB = (0<<SS);

    /* Wait for the device */
    _NOP();
    _NOP();
    /* Transmit nothing */
    for(i=0;i<4;i++) {
        SPDR = 0x00;
        /* Wait for transmission to complete */
        while (!(SPSR & _BV(SPIF)));
        sensor[i] = SPDR;
    }

    /* Thermocouple temperature */
    if (sensor[0]&(1<<7)) {
        /* Negative temperature, clamp it to zero */
        temp = 0;
    } else {
        temp = (((uint16_t)sensor[0])<<6)+(sensor[1]>>2);
    }

    /* Room temperature */
    if (sensor[2]&(1<<7)) {
        /* Negative temperature, clamp it to zero */
        room_temp = 0;
    } else {
        room_temp = (((uint16_t)sensor[2])<<4)+(sensor[3]>>4);
        /* Sensor gives room temp as sixteenths of celsius,
         * divide it by four to get quarters of celsius. */
        room_temp = room_temp / 4;
    }

    if (sensor[1]&0x01) {
        /* Fault */
        fprintf(&USBSerialStream,"Fault:%u\n",sensor[3]&0b00000111);
    }

    /* Disable slave */
    PORTB = (1<<SS);
}

void setupHardware(void) {

    /* Disable wtachdog */
    MCUSR &= ~(1 << WDRF);
	wdt_disable();
    /* Disable prescaler */
    clock_prescale_set(clock_div_1);

    /* Set !SS and SCK output, all others input */
    DDRB = (1<<SS)|(1<<SCK);
    //bit_set(DDRB,SS);
    //bit_set(DDRB,SCK);
    bit_set(PORTB, SS);/* Set !SS high (slave not enabled) */
    bit_clear(PORTB, SCK);

    /* Set timer1 to count 1 second */
    TCNT1 = 0x00;
    /* PWM output to channel A, pin PC6 */
    TCCR1A = 0b10000010;
    /*  Set prescaler to divide by 256 for TMR1 */
    TCCR1B = 0b11011100;
    TIMSK1 = (1<<2);
    ICR1 = _ICR1;
    OCR1B = 1;
    OCR1A = _ICR1;

    /* PC6 = Relay */
    DDRC   = 0b01000000;
    PORTC  = 0x00;
    DDRD   = 0x00;

    /* Enable SPI, Master, set clock rate fck/2 */
    SPCR = (1<<SPE) | (1<<MSTR);
    SPSR = (1<<SPI2X);

    /* Initialize USB */
    USB_Init();
    CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    return;
}

/* Get PWM frequency from target temperature */
uint16_t approx_pwm(uint16_t target)
{
    int32_t t;
	t = ((_ICR1*(target-room_temp)) / (MAXTEMP*4));
    return (uint16_t)CLAMP(t,0,_ICR1);
}

uint16_t pid(void) {
	int16_t error = target - temp;
	if (target == 0) {
		integral = 0;
		last_error = error;
		return 0;
	} else {

		int16_t new_integral = integral + error;
        /* Clamp integral to a reasonable value */
        new_integral = CLAMP(new_integral,-_ICR1,_ICR1);

		int32_t p_term = profile.pid_p * error;
		int32_t i_term = new_integral * profile.pid_i;
		int32_t d_term = (last_error - error) * profile.pid_d;

		last_error = error;

        /* Detect overflow */
        if ((error != 0) && (SIGN(p_term) != SIGN(profile.pid_p)*SIGN(error))) {
            p_term = -SIGN(p_term)*65535;
        }
        if ((integral != 0) && (SIGN(i_term) != SIGN(profile.pid_i)*SIGN(integral))) {
            i_term = -SIGN(i_term)*65535;
        }
        if (((last_error-error) != 0) && (SIGN(d_term) != SIGN(profile.pid_d)*SIGN(last_error-error))) {
            d_term = -SIGN(d_term)*65535;
        }
        /* Clamp terms */
        p_term = CLAMP(p_term,-65536,65535);
        i_term = CLAMP(i_term,-65536,65535);
        d_term = CLAMP(d_term,-65536,65535);
		int32_t result = approx_pwm(target) + p_term + i_term + d_term;

        /* Avoid integral buildup */
		if ((result >= _ICR1 && new_integral < integral) || (result < 0 && new_integral > integral) || (result <= _ICR1 && result >= 0)) {
            integral = new_integral;
		}

        /* Clamp the output value */
        return (uint16_t)(CLAMP(result,0,_ICR1));
	}
}

int main(void) {

    setupHardware();
    set_profile();
	GlobalInterruptEnable();

    reflow_state = T_START;

    while(1)
    {
        if (usb_connected) {
            /*  Check mail */
            usb_rx();
            CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
            USB_USBTask();
        }

        if (usb_connected && tx_flag) {
            tx_flag = 0;
            /* Send temp temperature */
            fprintf(&USBSerialStream, "temp:%u,room:%u,target:%u,PWM:%u,state:%d", temp, room_temp, target, OCR1A, reflow_state);
            if (PID_debug)
                fprintf(&USBSerialStream, ",I:%d", integral);
            fprintf(&USBSerialStream, "\n");
        }
    }
}


ISR(TIMER1_COMPB_vect) {
    /* Update target once per second */
    if (target_update++ == 5) {
        target_update = 0;
        target = target_temp();
        timer++;
        tx_flag = 1;
    }
    /* Read the current temperature, updates temp and room_temp */
    read_sensor();
    /* Set PWM */
    OCR1A = pid();
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
    usb_connected = 1;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
    usb_connected = 0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

