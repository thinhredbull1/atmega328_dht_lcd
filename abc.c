/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
� Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project :
Version :
Date    : 31/10/2024
Author  :
Company :
Comments:


Chip type               : ATmega328P
Program type            : Application
AVR Core Clock frequency: 16,000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/
#define F_CPU 16000000UL
#include <mega328p.h>
#include <io.h>
#include <interrupt.h>
#include <string.h>
#include <delay.h>
#include <i2c.h>
#include <stdio.h>
#include <stdint.h>
#include <util/atomic.h>
#asm
.equ __sda_bit = 4
                     .equ __scl_bit = 5
                                          //.equ __i2c_port=0x1b ;PORTA
                                          //.equ __i2c_port=0x18 ;PORTB
                                          .equ __i2c_port = 0x15;
PORTC
//.equ __i2c_port=0x12 ;PORT
#endasm
// Declare your global variables here

// TWI functions
void _i2c_init()
{
    TWBR = 0x62;        //	Baud rate is set by calculating
    TWCR = (1 << TWEN); // Enable I2C
    TWSR = 0x00;        // Prescaler set to 1
}
// Start condition
void _i2c_start()
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA); // start condition
    while (!(TWCR & (1 << TWINT)))
        ; // check for start condition
}
// I2C stop condition
void _i2c_write(char x)
{                                      // Cpn esta funcion se escribe en el bus de TWDR
    TWDR = x;                          // Move value to I2C
    TWCR = (1 << TWINT) | (1 << TWEN); // Enable I2C and clear interrupt
    while (!(TWCR & (1 << TWINT)))
        ;
}

char _i2c_read()
{
    TWCR = (1 << TWEN) | (1 << TWINT); // Enable I2C and clear interrupt
    while (!(TWCR & (1 << TWINT)))
        ; // Read successful with all data received in TWDR
    return TWDR;
}
void _i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // G?i t�n hi?u d?ng
}

bit lbl = 1;
#define LCD_ADDR 0x4E
#define LRS 0
#define LRW 1
#define LE 2
#define LBL 3
#define LD4 4
#define LD5 5
#define LD6 6
#define LD7 7
#define DHT11_PORT PORTB
#define DHT11_PORTPIN PORTB2
#define DHT11_DDR DDRB
#define DHT11_DDRPIN DDB2
#define DHT11_PINPORT PINB
#define DHT11_PIN PINB2
#define BUT_PIN PINB
#define FAN_PIN PORTD5
#define COIL_PIN PORTD7
#define PHUN_SUONG_PIN PORTD6
#define BUT_UP_PIN PINB4
#define BUT_DOWN_PIN PINB3
#define BUT_SETUP_PIN PINB5
#define DHT11_INPUT_MODE() DHT11_DDR &= ~(1 << DHT11_DDRPIN)
#define DHT11_OUTPUT_MODE() DHT11_DDR |= (1 << DHT11_DDRPIN)
#define DHT11_LOW() DHT11_PORT &= ~(1 << DHT11_PORTPIN)
#define DHT11_HIGH() DHT11_PORT |= (1 << DHT11_PORTPIN)
#define ON 1
#define OFF 0
#define DHTLIB_OK 0
#define DHTLIB_ERROR_CHECKSUM -1
#define DHTLIB_ERROR_TIMEOUT -2
#define TOLERANCE_TEMP 0.6
#define TOLERANCE_HUM 0.8
enum stateMain
{
    HIGH_MODE = 0,
    LOW_MODE = 1,
    NORMAL = 2
};
int stateTemp = NORMAL;
int stateHum = NORMAL;
float setupTemp;
float setupHum;
float RealSetupT;
float RealSetupH;
uint8_t error = 0;
uint8_t hum_now;
int count_state_setup = 0;

unsigned int base_y[5] = {0x00, 0x80, 0xC0, 0x94, 0xD4};
volatile unsigned long timer1_millis = 0;
unsigned long millis()
{
    unsigned long millis_return;

    // Ensure this cannot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        millis_return = timer1_millis;
    }
    return millis_return;
}
ISR(TIMER1_COMPA_vect)
{
    timer1_millis++;
}

void init_millis(unsigned long f_cpu)
{
    unsigned long ctc_match_overflow;

    ctc_match_overflow = ((f_cpu / 1000) / 8); // when timer1 is this value, 1ms has passed

    // (Set timer to clear when matching ctc_match_overflow) | (Set clock divisor to 8)
    TCCR1B |= (1 << WGM12) | (1 << CS11);

    // high byte first, then low byte
    OCR1AH = (ctc_match_overflow >> 8);
    OCR1AL = ctc_match_overflow;

    // Enable the compare match interrupt
    TIMSK1 |= (1 << OCIE1A);

    // REMEMBER TO ENABLE GLOBAL INTERRUPTS AFTER THIS WITH sei(); !!!
}
void ProcessBut()
{
    if (count_state_setup != 0)
    {
        if (!(BUT_PIN & (1 << BUT_UP_PIN)))
        { // Ki?m tra n?u n�t b?m UP
            if (hum_now)
                setupTemp += 1;
            else
                setupHum += 1;
        }
        else if (!(BUT_PIN & (1 << BUT_DOWN_PIN)))
        { // Ki?m tra n?u n�t b?m DOWN
            if (hum_now)
                setupTemp -= 1;
            else
                setupHum -= 1;
        }
    }
}
void FAN_SET(uint8_t on_off)
{
    if(on_off==ON)
    {
        PORTD|=(1<<FAN_PIN);
    }
    else{
        PORTD&=~(1<<FAN_PIN);
    }
}
void TEMP_COIL_SET(uint8_t on_off)
{
    if(on_off==ON)
    {
        PORTD|=(1<<COIL_PIN);
    }
    else{
        PORTD&=~(1<<COIL_PIN);
    }
}
void PHUN_SUONG(uint8_t on_off)
{
    if(on_off==ON)
    {
        PORTD|=(1<<PHUN_SUONG_PIN);
    }
    else{
        PORTD&=~(1<<PHUN_SUONG_PIN);
    }
}
int dht11_read(uint8_t *humidity, float *temperature)
{
    uint8_t DHT11Data[5];
    uint8_t sensor_bytes, bits, buffer = 0, timeout = 0, checksum;

    //
    DHT11_OUTPUT_MODE(); //
    DHT11_LOW();         //
    delay_ms(19);        //
    DHT11_HIGH();
    delay_us(20);

    DHT11_INPUT_MODE();
    delay_us(30); // wait 20-40us

    // check dht
    if (DHT11_PINPORT & (1 << DHT11_PIN))
    {
        return 0; //// no response
    }

    // Ch? t�n hi?u HIGH t? c?m bi?n
    delay_us(80);
    if (!(DHT11_PINPORT & (1 << DHT11_PIN)))
    {
        return 0; // L?i kh�ng nh?n du?c ph?n h?i
    }

    delay_us(80); // wait high
    for (sensor_bytes = 0; sensor_bytes < 5; sensor_bytes++)
    {
        buffer = 0;
        for (bits = 0; bits < 8; bits++)
        {
            while (!(DHT11_PINPORT & (1 << DHT11_PIN)))
            {
                timeout++;
                if (timeout > 100)
                    break; // timeout
                delay_us(1);
            }
            timeout = 0;

            // �?c bit
            delay_us(40); // Th?i gian d?c bit
            if (DHT11_PINPORT & (1 << DHT11_PIN))
            {
                buffer |= (1 << (7 - bits)); // save to buffer
            }

            // Ch? t�n hi?u HIGH k?t th�c
            while (DHT11_PINPORT & (1 << DHT11_PIN))
            {
                timeout++;
                if (timeout > 100)
                    break; // timeout
                delay_us(1);
            }
            timeout = 0;
        }
        DHT11Data[sensor_bytes] = buffer; // save
    }
    checksum = DHT11Data[0] + DHT11Data[1] + DHT11Data[2] + DHT11Data[3];
    if (checksum != DHT11Data[4])
    {
        return -1; // checksum
    }
    *humidity = DHT11Data[0];
    float temp = DHT11Data[2];
    if (DHT11Data[3] & 0x80)
    { // if sign bit is set (negative temperature)
        temp = -1 - temp;
    }
    temp += (DHT11Data[3] & 0x0F) * 0.1; // add decimal part
    *temperature = temp;
    return 1;
}
void lcd_write(unsigned char data)
{

    _i2c_start();

    _i2c_write(LCD_ADDR);

    if (lbl == 1)
    {
        data |= (1 << LBL);
    }
    else
    {
        data &= ~(1 << LBL);
    }
    _i2c_write(data);

    _i2c_stop();
}
void lcd_reset(void)
{
    unsigned char data;
    delay_ms(20);
    data = 0x30 | (1 << LE);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    delay_ms(10);
    data |= (1 << LE);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    delay_ms(1);
    data |= (1 << LE);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    delay_ms(1);
    data = 0x20 | (1 << LE);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    delay_ms(1);
}
void lcd_cmd(char cmd)
{
    unsigned char data;
    data = (cmd & 0xF0) | (1 << LE);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    data = ((cmd << 4) & 0xF0) | (1 << LE);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    delay_ms(2);
    delay_ms(2);
}
void lcd_init(void)
{
    _i2c_init();
    // lcd_reset();         // Call LCD reset lcd_cmd(0�28);
    lcd_cmd(0x02); /* send for 4 bit initialization of LCD  */
    lcd_cmd(0x28);
    lcd_cmd(0x0C); // Display no cursor � no blink.
    lcd_cmd(0x06); // Automatic Increment � No Display shift.
    lcd_cmd(0x01);
    lcd_cmd(0x80); // Address DDRAM with 0 offset 80h.
}
void lcd_data(unsigned char dat)
{
    unsigned char data;
    data = (dat & 0xF0) | (1 << LE) | (1 << LRS);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    data = ((dat << 4) & 0xF0) | (1 << LE) | (1 << LRS);
    lcd_write(data);
    data &= ~(1 << LE);
    lcd_write(data);
    delay_ms(2);
    delay_ms(2);
}
void lcd_putchar(char c)
{
    lcd_data(c);
}
void lcd_puts(char *str)
{
    unsigned int i;
    for (i = 0; str[i] != 0; i++)
        lcd_data(str[i]);
}
void lcd_clear(void)
{
    lcd_cmd(0x01);
}
void lcd_gotoxy(unsigned char x, unsigned char y)
{
    if (y == 0 && x < 16)
        lcd_cmd((x & 0x0F) | 0x80); /* Command of first row and required position<16 */
    else if (y == 1 && x < 16)
        lcd_cmd((x & 0x0F) | 0xC0); /* Command of first row and required position<16 */
}
void lcd_backlightoff(void)
{
    lbl = 0;
}
void lcd_backlighton(void)
{
    lbl = 1;
}
void lcd_backlighttoggle(void)
{
    lbl++;
}
void lcd_cursor(void)
{
    lcd_cmd(0x0E); // Lệnh hiện con trỏ
}

void lcd_nocursor(void)
{
    lcd_cmd(0x0C); // Lệnh ẩn con trỏ
}
void LCD_String_xy(char pos, char row, char *str) /* Send string to LCD with xy position */
{
    if (row == 0 && pos < 16)
        lcd_cmd((pos & 0x0F) | 0x80); /* Command of first row and required position<16 */
    else if (row == 1 && pos < 16)
        lcd_cmd((pos & 0x0F) | 0xC0); /* Command of first row and required position<16 */
    lcd_puts(str);                    /* Call LCD string function */
}
void main(void)
{
// Declare your local variables here

// Crystal Oscillator division factor: 1
#pragma optsize -
    CLKPR = (1 << CLKPCE);
    CLKPR = (0 << CLKPCE) | (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize +
#endif

    // Input/Output Ports initialization
    // Port B initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRB = (0 << DDB7) | (0 << DDB6) | (0 << DDB5) | (0 << DDB4) | (0 << DDB3) | (0 << DDB2) | (0 << DDB1) | (0 << DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB = (0 << PORTB7) | (0 << PORTB6) | (1 << PORTB5) | (1 << PORTB4) | (1 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

    // Port C initialization
    // Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRC = (0 << DDC6) | (0 << DDC5) | (0 << DDC4) | (0 << DDC3) | (0 << DDC2) | (0 << DDC1) | (0 << DDC0);
    // State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTC = (0 << PORTC6) | (0 << PORTC5) | (0 << PORTC4) | (0 << PORTC3) | (0 << PORTC2) | (0 << PORTC1) | (0 << PORTC0);

    // Port D initialization
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRD = (1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (0 << DDD4) | (0 << DDD3) | (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTD = (0 << PORTD7) | (0 << PORTD6) | (0 << PORTD5) | (0 << PORTD4) | (0 << PORTD3) | (0 << PORTD2) | (0 << PORTD1) | (0 << PORTD0);

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: Timer 0 Stopped
    // Mode: Normal top=0xFF
    // OC0A output: Disconnected
    // OC0B output: Disconnected
    TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (0 << WGM01) | (0 << WGM00);
    TCCR0B = (0 << WGM02) | (0 << CS02) | (0 << CS01) | (0 << CS00);
    TCNT0 = 0x00;
    OCR0A = 0x00;
    OCR0B = 0x00;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: Timer1 Stopped
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: Timer2 Stopped
    // Mode: Normal top=0xFF
    // OC2A output: Disconnected
    // OC2B output: Disconnected
    ASSR = (0 << EXCLK) | (0 << AS2);
    TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
    TCCR2B = (0 << WGM22) | (0 << CS22) | (0 << CS21) | (0 << CS20);
    TCNT2 = 0x00;
    OCR2A = 0x00;
    OCR2B = 0x00;

    // Timer/Counter 0 Interrupt(s) initialization
    TIMSK0 = (0 << OCIE0B) | (0 << OCIE0A) | (0 << TOIE0);

    // Timer/Counter 1 Interrupt(s) initialization
    TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

    // Timer/Counter 2 Interrupt(s) initialization
    TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (0 << TOIE2);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // Interrupt on any change on pins PCINT0-7: Off
    // Interrupt on any change on pins PCINT8-14: Off
    // Interrupt on any change on pins PCINT16-23: Off
    EICRA = (0 << ISC11) | (0 << ISC10) | (0 << ISC01) | (0 << ISC00);
    EIMSK = (0 << INT1) | (0 << INT0);
    PCICR = (0 << PCIE2) | (0 << PCIE1) | (0 << PCIE0);

    // USART initialization
    // USART disabled
    UCSR0B = (0 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (0 << RXEN0) | (0 << TXEN0) | (0 << UCSZ02) | (0 << RXB80) | (0 << TXB80);

    // Analog Comparator initialization
    // Analog Comparator: Off
    // The Analog Comparator's positive input is
    // connected to the AIN0 pin
    // The Analog Comparator's negative input is
    // connected to the AIN1 pin
    ACSR = (1 << ACD) | (0 << ACBG) | (0 << ACO) | (0 << ACI) | (0 << ACIE) | (0 << ACIC) | (0 << ACIS1) | (0 << ACIS0);
    ADCSRB = (0 << ACME);
    // Digital input buffer on AIN0: On
    // Digital input buffer on AIN1: On
    DIDR1 = (0 << AIN0D) | (0 << AIN1D);

    // ADC initialization
    // ADC disabled
    ADCSRA = (0 << ADEN) | (0 << ADSC) | (0 << ADATE) | (0 << ADIF) | (0 << ADIE) | (0 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);

    // SPI initialization
    // SPI disabled
    SPCR = (0 << SPIE) | (0 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR1) | (0 << SPR0);

    // TWI initialization
    // TWI disabled

    lcd_init();
    LCD_String_xy(0, 0, "Start Sensor");

    delay_ms(1000);

    delay_ms(500);
    lcd_clear();
    while (1)

    {
        uint8_t humidity;
        float temperature;
        char buffer[16];
        int status = 0;
        static uint8_t count_delay = 0;
        if (!(BUT_PIN & (1 << BUT_SETUP_PIN)))
        {

            count_state_setup += 1;
            if (count_state_setup >= 3)
                count_state_setup = 0;
        }
        ProcessBut();
        hum_now = count_state_setup == 1 ? 1 : 0;

        status = dht11_read(&humidity, &temperature);
        if (status == 1)
        {
            float diff_hum = RealSetupH - humidity;
            float diff_temp = RealSetupT - temperature;
            sprintf(buffer, "Tem:%f %f", temperature, setupTemp);
            lcd.noCursor();
            LCD_String_xy(0, 0, buffer);
            sprintf(buffer, "Hum:%d%% %f", humidity, setupHum);
            LCD_String_xy(0, 1, buffer);

            if (fabs(diff_hum) < tolerance_hum)
                stateHum = NORMAL;
            else if (diff_hum > tolerance_hum)
                stateHum = LOW_MODE; // dif hum >0 va
            else if (diff_hum < -tolerance_hum)
                stateHum = HIGH_MODE;
            if (fabs(diff_temp) < tolerance_temp)
                stateTemp = NORMAL;
            else if (diff_temp > tolerance_temp)
                stateTemp = LOW_MODE;
            else if (diff_temp < -tolerance_temp)
                stateTemp = HIGH_MODE;
            if (stateTemp == HIGH_MODE)
            {

                FAN_SET(ON);
                TEMP_COIL_SET(OFF);
            }
            else if (stateTemp == NORMAL)
            {
                FAN_SET(OFF);
                TEMP_COIL_SET(OFF);
            }
            else if (stateTemp == LOW_MODE)
            {

                FAN_SET(OFF);
                TEMP_COIL_SET(ON);
            }

            if (stateHum == LOW_MODE)
                PHUN_SUONG(ON);
            else
                PHUN_SUONG(OFF);
            if (count_state_setup != 0)
            {
                lcd.lcd_gotoxy(11, hum_now);
                lcd.cursor();
            }
            else
            {
                lcd.noCursor();
            }

            RealSetupT = setupTemp;
            RealSetupH = setupHum;
        }
        else
        {
            lcd_clear();
            sprintf(buffer, "Error reading:%d", status);
            LCD_String_xy(0, 0, buffer);
        }

        delay_ms(200);
    }
}
