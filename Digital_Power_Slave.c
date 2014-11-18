//
//  RC_PPM.c
//
//
//  Created by Sysadmin on 20.07.13
//  Copyright Ruedi Heimlicher 2013. All rights reserved.
//


#include <string.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <math.h>

#include "lcd.c"
#include "adc.c"
#include "version.c"
#include "usb_rawhid.c"
#include "def.h"

//#include "spi.c"
#include "spi_adc.c"

#include "spi_slave.c"



// USB
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LOOPDELAY 5

#define SERVOMAX  4400
#define SERVOMIN  1400


#define USB_DATENBREITE 32
#define EE_PARTBREITE 32

#define CODE_OFFSET  8
#define ROTARY_OFFSET  10

/*
const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};
*/

 uint16_t key_state;				// debounced key state:
// bit = 1: key pressed
uint16_t key_press;				// key press detect
volatile uint16_t tscounter =0;



volatile uint8_t do_output=0;
static volatile uint8_t testbuffer[USB_DATENBREITE]={};


static volatile uint8_t buffer[USB_DATENBREITE]={};
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

static volatile uint8_t outbuffer[USB_DATENBREITE]={};
static volatile uint8_t inbuffer[USB_DATENBREITE]={};

static volatile uint8_t kontrollbuffer[USB_DATENBREITE]={};

static volatile uint8_t eeprombuffer[USB_DATENBREITE]={};

#define TIMER0_STARTWERT	0x40

#define EEPROM_STARTADRESSE   0x7FF

volatile uint8_t timer0startwert=TIMER0_STARTWERT;

//volatile uint8_t rxbuffer[USB_DATENBREITE];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[USB_DATENBREITE];

//uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit

void delay_ms(unsigned int ms);

static volatile uint8_t             displaystatus=0x00; // Tasks fuer Display
 volatile uint8_t                   eepromsavestatus = 0;


static volatile uint16_t            displaycounter=0;

volatile uint8_t                    in_taskcounter=0;
volatile uint8_t                    out_taskcounter=0;



static volatile uint8_t             substatus=0x00; // Tasks fuer Sub

static volatile uint8_t             usbstatus=0x00;

static volatile uint8_t             usbtask=0x00; // was ist zu tun

static volatile uint8_t             eepromstatus=0x00;
static volatile uint8_t             potstatus=0x00; // Bit 7 gesetzt, Mittelwerte setzen
static volatile uint8_t             impulscounter=0x00;

static volatile uint8_t             masterstatus = 0;

static volatile uint8_t             tastaturstatus = 0;

volatile uint8_t status=0;

volatile uint8_t                    PWM=0;
static volatile uint8_t             pwmposition=0;
static volatile uint8_t             pwmdivider=0;


volatile char SPI_data='0';
volatile char SPI_dataArray[SPI_BUFSIZE];
volatile uint16_t Pot_Array[SPI_BUFSIZE];

volatile uint16_t Mitte_Array[8];

//volatile uint8_t Level_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal
//volatile uint8_t Expo_Array[8]; // Levels fuer Kanaele, 1 byte pro kanal

volatile uint16_t Mix_Array[8];// Mixings, 2 8-bit-bytes pro Mixing


volatile uint16_t RAM_Array[SPI_BUFSIZE];

volatile uint8_t testdataarray[8]={};
volatile uint16_t teststartadresse=0xA0;


volatile uint16_t Batteriespannung =0;

volatile uint16_t adc_counter =0; // zaehlt Impulspakete bis wieder die Batteriespannung gelesen werden soll

volatile short int received=0;

volatile uint16_t abschnittnummer=0;

volatile uint16_t usbcount=0;

volatile uint16_t minwert=0xFFFF;
volatile uint16_t maxwert=0;

volatile uint16_t eepromstartadresse=0;

volatile uint16_t inchecksumme=0;

volatile uint16_t bytechecksumme=0;
volatile uint16_t outchecksumme=0;

volatile uint8_t eeprom_databyte=0;
volatile uint8_t anzahlpakete=0;
//volatile uint8_t usb_readcount = 0;

volatile uint8_t  eeprom_indata=0;

volatile    uint8_t task_in=0;      // Task von RC_PPM
volatile    uint8_t task_indata=0;  // Taskdata von RC_PPM

volatile    uint8_t task_out=0;     // Task an RC_PPM
volatile    uint8_t task_outdata=0; // Taskdata an RC_PPM

// Mark Screen

//#define CLOCK_DIV 15 // timer0 1 Hz bei Teilung /4 in ISR 16 MHz
#define CLOCK_DIV 8 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz
#define BLINK_DIV 4 // timer0 1 Hz bei Teilung /4 in ISR 8 MHz



volatile uint16_t                TastaturCount=0;
volatile uint16_t                manuellcounter=0; // Counter fuer Timeout
volatile uint8_t                 startcounter=0; // timeout-counter beim Start von Settings, schneller als manuellcounter. Ermoeglicht Dreifachklick auf Taste 5
volatile uint8_t                 settingstartcounter=0; // Counter fuer Klicks auf Taste 5
volatile uint16_t                mscounter=0; // Counter fuer ms in timer-ISR
volatile uint8_t                 blinkcounter=0;

volatile uint16_t                TastenStatus=0;
volatile uint16_t                Tastencount=0;
volatile uint16_t                Tastenprellen=0x01F;
volatile uint8_t                 Taste=0;

volatile uint16_t                tastentransfer=0;

volatile uint8_t                 Tastenindex=0;
volatile uint8_t                 lastTastenindex=0;
volatile uint16_t                prellcounter=0;

volatile uint8_t                 trimmstatus=0;
volatile uint8_t                 Trimmtaste=0;
volatile uint8_t                 Trimmtastenindex=0;
volatile uint8_t                 lastTrimmtastenindex=0;
volatile uint16_t                trimmprellcounter=0;

volatile uint16_t                manuelltrimmcounter=0; // Counter fuer Timeout der Trimmtsastatur


volatile uint8_t                  vertikaltrimm_L=0;
volatile uint8_t                  vertikaltrimm_R=0;
volatile uint8_t                  horizontaltrimm_L=0;
volatile uint8_t                  horizontaltrimm_R=0;





//volatile uint16_t                SPI_Data_counter; // Zaehler fuer Update des screens



volatile uint16_t Tastenwert=0;
volatile uint16_t Trimmtastenwert=0;
volatile uint8_t adcswitch=0;


/*
 #define TASTE1		15
 #define TASTE2		23
 #define TASTE3		34
 #define TASTE4		51
 #define TASTE5		72
 #define TASTE6		94
 #define TASTE7		120
 #define TASTE8		141
 #define TASTE9		155
 #define TASTE_L	168
 #define TASTE0		178
 #define TASTE_R	194
 
 */
//const char wertearray[] PROGMEM = {TASTE1,TASTE2,TASTE3,TASTE4,TASTE5,TASTE6,TASTE7,TASTE8,TASTE9,TASTE_L,TASTE0,TASTE_R};

/*

static inline
uint16_t key_no( uint8_t adcval )
{
   uint16_t num = 0x1000;
   PGM_P pointer = wertearray;
   
   
   while( adcval < pgm_read_byte(pointer))
   {
      pointer++;
      num >>= 1;
   }
   return num & ~0x1000;
}

uint16_t get_key_press( uint16_t key_mask )
{
   cli();
   key_mask &= key_press;		// read key(s)
   key_press ^= key_mask;		// clear key(s)
   sei();
   return key_mask;
}

*/






void startTimer2(void)
{
   //timer2
   TCNT2   = 0;
   //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
   //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}

void Master_Init(void)
{
	LOOPLEDDDR |=(1<<LOOPLED);
	LOOPLEDPORT |= (1<<LOOPLED);	// HI
   
	//Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
   
   
	
   /*
    TASTENDDR &= ~(1<<TASTE0);	//Bit 0 von PORT B als Eingang fŸr Taste 0
    TASTENPORT |= (1<<TASTE0);	//Pull-up
    */
   
   
   
   // ---------------------------------------------------
   // Pin Change Interrupt enable on PCINT0 (PD7)
   // ---------------------------------------------------
   
   // PCIFR |= (1<<PCIF0);
   // PCICR |= (1<<PCIE0);
   //PCMSK0 |= (1<<PCINT7);
  
   // ---------------------------------------------------
   // USB_Attach
   // ---------------------------------------------------
   
   EICRA |= (1<<ISC01); // falling edge
   EIMSK=0;
   EIMSK |= (1<<INT0); // Interrupt en

   
   // ---------------------------------------------------
	//LCD
   // ---------------------------------------------------
	LCD_DDR |= (1<<LCD_RSDS_PIN);		// PIN als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT D als Ausgang fuer LCD
   
   
}


void SPI_PORT_Init(void) // SPI-Pins aktivieren
{
   
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   
   /*
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // HI   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
    */
   
   // Slave init
   SPI_DDR |= (1<<SPI_MISO_PIN); // Output
   //SPI_PORT &= ~(1<<SPI_MISO_PIN); // HI
   SPI_DDR &= ~(1<<SPI_MOSI_PIN); // Input
   SPI_DDR &= ~(1<<SPI_SCK_PIN); // Input
   //SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   SPI_DDR &= ~(1<<SPI_SS_PIN); // Input
   SPI_PORT |= (1<<SPI_SS_PIN); // HI

   
   
   
}

void SPI_ADC_init(void) // SS-Pin fuer EE aktivieren
{
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
}



void spi_start(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // LO
   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   SPI_PORT &= ~(1<<SPI_MOSI_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
  }

void spi_end(void) // SPI-Pins deaktivieren
{
   SPCR=0;
   
   SPI_DDR &= ~(1<<SPI_MOSI_PIN); // MOSI off
   SPI_DDR &= ~(1<<SPI_SCK_PIN); // SCK off
   SPI_DDR &= ~(1<<SPI_SS_PIN); // SS off
   
   //SPI_RAM_DDR &= ~(1<<SPI_RAM_CS_PIN); // RAM-CS-PIN off
   //SPI_EE_DDR &= ~(1<<SPI_EE_CS_PIN); // EE-CS-PIN off
}

/*
void spi_slave_init()
{
   SPCR=0;
   SPCR = (1<<SPE)|(1<<SPR1)|(0<<SPR0)|(1<<CPOL)|(1<<CPHA);
   
}
*/

void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

// http://www.co-pylit.org/courses/COSC2425/lectures/AVRNetworks/index.html

/* nicht verwendet
void timer1_init(void)
{
    // Quelle http://www.mikrocontroller.net/topic/103629
    
    OSZI_A_HI ; // Test: data fuer SR
    _delay_us(5);
    //#define FRAME_TIME 20 // msec
    KANAL_DDR |= (1<<KANAL_PIN); // Kanal Ausgang
    
    DDRD |= (1<<PORTD5); //  Ausgang
    PORTD |= (1<<PORTD5); //  Ausgang
    
    //TCCR1A = (1<<COM1A0) | (1<<COM1A1);// | (1<<WGM11);	// OC1B set on match, set on TOP
    //TCCR1B = (1<<WGM13) | (1<<WGM12) ;		// TOP = ICR1, clk = sysclk/8 (->1us)
    TCCR1B |= (1<<CS11);
    TCNT1  = 0;														// reset Timer
    
    // Impulsdauer
    OCR1B  = 0x80;				// Impulsdauer des Kanalimpulses
    
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
} // end timer1
*/
/*
void timer1_stop(void)
{
   // TCCR1A = 0;
   
}
 */
/*
 ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
 {
 
 KANAL_HI;
 impulscounter++;
 
 if (impulscounter < ANZ_POT)
 {
 // Start Impuls
 
 TCNT1  = 0;
 //KANAL_HI;
 
 // Laenge des naechsten Impuls setzen
 
 //OCR1A  = POT_FAKTOR*Pot_Array[1]; // 18 us
 //OCR1A  = POT_FAKTOR*Pot_Array[impulscounter]; // 18 us
 
 }
 }
 */

/*
 ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 ms
 {
 //OSZI_A_LO ;
 //PORTB &= ~(1<<PORTB5); // OC1A Ausgang
 //OSZI_A_HI ;
 KANAL_LO;
 
 if (impulscounter < ANZ_POT)
 {
 }
 else
 {
 timer1_stop();
 
 }
 }
 */


void timer0 (void) // Grundtakt fuer Stoppuhren usw.
{
   // Timer fuer Exp
	//TCCR0 |= (1<<CS01);						// clock	/8
	//TCCR0 |= (1<<CS01)|(1<<CS02);			// clock	/64
	//TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
	//TCCR0 |= (1<<CS00)|(1<<CS02);			// clock /1024
	
   TCCR0B |= (1 << CS02);//
   //TCCR0B |= (1 << CS00);
   
   TCCR0B |= (1 << CS10); // Set up timer
	
   OCR0A = 0x02;
	
	//TIFR |= (1<<TOV0);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);							//Overflow Interrupt aktivieren
	TCNT0 = 0;					//RŸcksetzen des Timers
}

/*
 void timer2 (uint8_t wert)
 {
 //timer2
 TCNT2   = 0;
 //	TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode
 TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8
 //	TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt
 TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt
 //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64
 TCCR2A = 0x00;
 
 
 OCR2 = wert;									//Setzen des Compare Registers auf Servoimpulsdauer
 }
 */

volatile uint16_t timer2Counter=0;
volatile uint16_t timer2BatterieCounter=0;

#pragma mark TIMER0_OVF

ISR (TIMER0_OVF_vect)
{
   mscounter++;
    
   if (mscounter%BLINK_DIV ==0)
   {
      blinkcounter++;
   }
   
}

#pragma mark INT0
ISR(INT0_vect) // Interrupt bei CS, falling edge
{
   OSZI_B_LO;
   inindex=0;
   SPDR = 18;// Erstes Byte an Slave
   OSZI_B_HI;
   
 //  spi_txbuffer[0]++;
 //  spi_txbuffer[2]--;

}



#pragma mark PIN_CHANGE
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328

/*
ISR (PCINT0_vect)
{
   
   if(INTERRUPT_PIN & (1<< MASTER_EN_PIN))// LOW to HIGH pin change, Sub ON
   {
      //OSZI_C_LO;
     
      masterstatus |= (1<<SUB_TASK_BIT); // Zeitfenster fuer Task offen
      adc_counter ++; // loest adc aus
      
   }
   else // HIGH to LOW pin change, Sub ON
   {
      displaystatus |= (1<<UHR_UPDATE);
      //masterstatus &= ~(1<<SUB_TASK_BIT);
   }
   
}
 */

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
   /*
    // Atmega168
    
    #define TASTE1		19
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		186
    #define TASTE9		212
    #define TASTE_L	234
    #define TASTE0		248
    #define TASTE_R	255
    
    
    // Atmega328
    #define TASTE1		17
    #define TASTE2		29
    #define TASTE3		44
    #define TASTE4		67
    #define TASTE5		94
    #define TASTE6		122
    #define TASTE7		155
    #define TASTE8		166
    #define TASTE9		214
    #define TASTE_L	234
    #define TASTE0		252
    #define TASTE_R	255
    */
   
   //lcd_gotoxy(0,0);
   //lcd_putint(Tastaturwert);
   /*
   if (Tastaturwert < TASTE1)
      return 1;
   if (Tastaturwert < TASTE2)
      return 2;
   if (Tastaturwert < TASTE3)
      return 3;
   if (Tastaturwert < TASTE4)
      return 4;
   if (Tastaturwert < TASTE5)
      return 5;
   if (Tastaturwert < TASTE6)
      return 6;
   if (Tastaturwert < TASTE7)
      return 7;
   if (Tastaturwert < TASTE8)
      return 8;
   if (Tastaturwert < TASTE9)
      return 9;
   
   if (Tastaturwert < TASTE_L)
      return 10;
   if (Tastaturwert < TASTE0)
      return 0;
   if (Tastaturwert <= TASTE_R)
      return 12;
   */
   
   
   
   // Tastatur2 // Reihenfolge anders
   /*
    #define WERT1    11    // 1 oben  Taste 2
    #define WERT3    34    // 2 links  Taste 4
    #define WERT4    64    // 3 unten  Taste 8
    #define WERT6    103   // 4 rechts  Taste 6
    #define WERT9    174   // 5 Mitte  Taste 5
    #define WERT2 	26    //  A links oben Taste  1
    #define WERT5    72       //    B links unten Taste 7
    #define WERT7    116      //   C rechts oben Taste 3
    #define WERT8    161      // D rechts unten Taste 9
    
    */

   if (Tastaturwert < WERT1)
      return 2;
   if (Tastaturwert < WERT2)
      return 1;
   if (Tastaturwert < WERT3)
      return 4;
   if (Tastaturwert < WERT4)
      return 8;
   if (Tastaturwert < WERT5)
      return 7;
   if (Tastaturwert < WERT6)
      return 6;
   if (Tastaturwert < WERT7)
      return 3;
   if (Tastaturwert < WERT8)
      return 9;
   if (Tastaturwert < WERT9)
      return 5;

   return -1;



}












// MARK:  - main
int main (void)
{
   int8_t r;
   
   uint16_t spi_count=0;
   
	// set for 16 MHz clock
	CPU_PRESCALE(CPU_8MHz); // Strom sparen
   
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
   
   //   sei();
	Master_Init();
   spi_slave_init();
   // ---------------------------------------------------
   // in attach verschobe, nur wenn USB eingesteckt
     usb_init();
   	while (!usb_configured()) /* wait */ ;
   
	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(100);
   
   volatile    uint8_t outcounter=0;
   volatile    uint8_t testdata =0x00;
//   volatile    uint8_t testaddress =0x00;
   volatile    uint8_t errcount =0x00;
//   volatile    uint8_t ram_indata=0;
   
 //  volatile    uint8_t eeprom_indata=0;
 //  volatile    uint8_t eeprom_testdata =0x00;
 //  volatile    uint8_t eeprom_testaddress =0x00;
   volatile    uint8_t usb_readcount =0x00;
   
   // ---------------------------------------------------
 	// initialize the LCD
   // ---------------------------------------------------
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   
	lcd_puts("Guten Tag\0");
	delay_ms(100);
	lcd_cls();
	//lcd_puts("READY\0");
	lcd_puts("V: \0");
	lcd_puts(VERSION);
   lcd_clr_line(1);
	
	uint8_t TastaturCount=0;
		
	//initADC(1);
	
	uint16_t loopcount0=0;
	uint16_t loopcount1=0;
	/*
    Bit 0: 1 wenn wdt ausgelšst wurde
    */
   
   PWM = 0;
   
   char* versionstring[4] = {};
   strncpy(versionstring, VERSION+9, 3);
   versionstring[3]='\0';
   volatile uint16_t versionint = atoi(versionstring);
   volatile uint8_t versionintl = versionint & 0x00FF;
   //versionint >>=8;
   volatile uint8_t versioninth = (versionint & 0xFF00)>>8;
   
   uint8_t anzeigecounter=0;
   uint8_t ind = 32;
   
 //  masterstatus |= (1<<SUB_READ_EEPROM_BIT); // sub soll EE lesen
   

	

   // ---------------------------------------------------
   // Vorgaben fuer Homescreen
   // ---------------------------------------------------
	lcd_gotoxy(0,0);
	lcd_puts("Digital_Power\0");
   delay_ms(1000);
   lcd_cls();
   //substatus |= (1<<SETTINGS_READ);;
   // ---------------------------------------------------
   // Settings beim Start lesen
   // ---------------------------------------------------
   eepromstatus |= (1<<READ_EEPROM_START);
   
//   timer0();
   
   sei();
   
   uint8_t i=0;
// MARK:  while
   
   volatile   uint8_t old_H=0;
   volatile   uint8_t old_L=0;
   uint8_t teensy_err =0;

	while (1)
	{
      //OSZI_B_LO;
		//Blinkanzeige
		loopcount0+=1;
      
      
      /* **** spi_buffer abfragen **************** */

      if (spi_rxdata) // Etwas ueber SPI angekommen
      {
         spi_rxdata=0;
         
         spi_count++;
         lcd_gotoxy(0,0);
         //lcd_puts("SPI");
         //lcd_gotoxy(3,0);
        // lcd_puts("  ");
        // lcd_gotoxy(4,0);
         //lcd_puthex(spi_count & 0xFF);
         //lcd_puthex(inindex & 0x07);
         //lcd_clr_line(1);
         //lcd_gotoxy(0,1);
         //lcd_puthex(spi_rxbuffer[inindex]);
         
         for (i=0;i<SPI_BUFFERSIZE;i++)
         {
            //lcd_puthex(spi_rxbuffer[i]);
            sendbuffer[i+CODE_OFFSET] = spi_rxbuffer[i];
         }
         
         
         
         //lcd_gotoxy(0,0);
         //lcd_putc('o');
         //lcd_gotoxy(12,1);
         //lcd_puts("  ");
         /*
          // Inhalt von USB
         lcd_gotoxy(0,0);
         lcd_puthex(spi_txbuffer[0]);
         lcd_puthex(spi_txbuffer[1]);
         lcd_puthex(spi_txbuffer[2]);
         lcd_puthex(spi_txbuffer[3]);
         lcd_putc(' ');
         lcd_puthex(spi_txbuffer[4]);
         lcd_puthex(spi_txbuffer[5]);
         lcd_puthex(spi_txbuffer[6]);
         lcd_puthex(spi_txbuffer[7]);
         */
         //lcd_puthex(sendbuffer[ROTARY_OFFSET]);
         //lcd_puthex(sendbuffer[ROTARY_OFFSET+1]);
         //lcd_puthex(spi_txbuffer[1]);
         //   SPDR = spi_txbuffer[0];
         
      }
      /* **** end spi_buffer abfragen **************** */

      
		if (loopcount0==0x2FFF)
		{
			loopcount0=0;
			loopcount1+=1;
			LOOPLEDPORT ^=(1<<LOOPLED);
         
         //lcd_gotoxy(18,0);
         //lcd_puthex(loopcount1);
         sendbuffer[0] = 0;//usb_readcount;
         sendbuffer[1] = 13;
         sendbuffer[2] = 17;
         sendbuffer[3] = 75;
         sendbuffer[5] = spi_rxbuffer[0];
         
         lcd_gotoxy(0,1);
         
         lcd_puthex(spi_rxbuffer[0]);
         lcd_puthex(spi_rxbuffer[1]);
         lcd_putc(' ');
         lcd_puthex(spi_rxbuffer[2]);
         lcd_puthex(spi_rxbuffer[3]);
         lcd_putc(' ');
         /*
         lcd_puthex(spi_rxbuffer[4]);
         lcd_puthex(spi_rxbuffer[5]);
         lcd_puthex(spi_rxbuffer[6]);
         lcd_puthex(spi_rxbuffer[7]);
         */
          lcd_gotoxy(0,0);
         lcd_puthex(spi_txbuffer[0]);
         lcd_puthex(spi_txbuffer[1]);
         lcd_putc(' ');
         lcd_puthex(spi_txbuffer[2]);
         lcd_puthex(spi_txbuffer[3]);

         
         if (!((old_H == spi_rxbuffer[3]) && (old_L == spi_rxbuffer[2])) )
         {
            teensy_err++;
            old_L = spi_rxbuffer[2];
            old_H = spi_rxbuffer[3];
         }
         //lcd_gotoxy(18,1);
         //lcd_puthex(teensy_err);
/*
         lcd_puthex(sendbuffer[CODE_OFFSET]);
         lcd_puthex(sendbuffer[CODE_OFFSET+1]);
         lcd_puthex(sendbuffer[ROTARY_OFFSET]);
         lcd_puthex(sendbuffer[ROTARY_OFFSET+1]);
         lcd_putc(' ');
         uint16_t rot = (sendbuffer[ROTARY_OFFSET+1]<<8) | sendbuffer[ROTARY_OFFSET];
         lcd_putint16(rot);
 */
         //lcd_puthex(sendbuffer[9]);
         //lcd_putc(' ');
         //lcd_puthex(sendbuffer[8]);
         //lcd_putc(' ');
         //lcd_puthex(SPI_Data_counter);


         uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         //lcd_puthex(usberfolg);
         
         
         if(loopcount1%16 == 0)
         {
            anzeigecounter = 0;
            if (anzeigecounter)
            {
               if (anzeigecounter > 10)
               {
                  anzeigecounter=0;
                  ind = 32;
               }
               else
               {
                  lcd_gotoxy(0,1);
                  lcd_puthex(sendbuffer[32]);
                  lcd_putc(' ');
                  lcd_puthex(sendbuffer[33]);
                  lcd_putc(' ');
                  lcd_puthex(sendbuffer[34]);
                  lcd_putc(' ');
                  lcd_puthex(sendbuffer[35]);
                  lcd_putc(' ');
                  
                   anzeigecounter++;
                  
                   
               }
               /*
               uint16_t wert = eeprombuffer[ind+0]+(eeprombuffer[ind+1]<<8);
               
               lcd_putint12(wert);
               wert = eeprombuffer[ind+2]+(eeprombuffer[ind+3]<<8);
               lcd_putc(' ');
               lcd_putint12(wert);
               wert = eeprombuffer[ind+4]+(eeprombuffer[ind+5]<<8);
               lcd_putc(' ');
               lcd_putint12(wert);
               wert = eeprombuffer[ind+6]+(eeprombuffer[ind+7]<<8);
               lcd_putc(' ');
               lcd_putint12(wert);
                */
               /*
               lcd_putc('*');
               lcd_puthex(buffer[ind+0]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+1]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+2]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+3]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+4]);
               lcd_putc('*');
               lcd_puthex(buffer[ind+5]);
               lcd_putc('*');
                */
               //sei();
            }
         } //

         
         
			

// MARK:  USB send
         // neue Daten abschicken
//         if ((usbtask & (1<<EEPROM_WRITE_PAGE_TASK) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         //OSZI_C_LO;
         
      //   uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
         //OSZI_C_HI;
         if ((masterstatus & (1<< HALT_BIT) )) //|| usbtask & (1<<EEPROM_WRITE_BYTE_TASK))
         {
            // Write im Gang, nichts senden
            masterstatus |= (1<<SUB_TASK_BIT);
         
         }
         else
         {
                           //OSZI_C_LO;
            //uint8_t anz = usb_rawhid_send((void*)sendbuffer, 50); // 20 us
                     //OSZI_C_HI;
         }
      } // if loopcount0
      
      /**	ADC	***********************/
      
      /**	END ADC	***********************/
      
      /**	Begin USB-routinen	***********************/
// MARK USB read
      // Start USB
      //OSZI_D_LO;
      r=0;
      
      r = usb_rawhid_recv((void*)buffer, 0); // 5us
      //OSZI_D_HI;
      // MARK: USB_READ
      
      if (r > 0)
      {
         OSZI_A_LO ;
         
         //OSZI_D_LO;
         cli();
         usb_readcount++;
         uint8_t code = 0x00;
         
         /*
         lcd_gotoxy(0,0);
         //lcd_putc('*');
         lcd_puthex(r);
         lcd_putc('*');
         lcd_putint2(usb_readcount);
         lcd_putc('*');
         
         lcd_puthex(buffer[0]);
         
         lcd_puthex(buffer[1]);
         lcd_puthex(buffer[2]);
         lcd_puthex(buffer[3]);
         lcd_putc('*');
          */

         for (i=0;i<SPI_BUFFERSIZE;i++)
         {
            //lcd_puthex(spi_rxbuffer[i]);
            //sendbuffer[i+ROTARY_OFFSET] = spi_rxbuffer[i];
         }

                  //lcd_putc('*');
         //lcd_puthex(r);
         //lcd_putc('*');
         //lcd_puthex(sendbuffer[ROTARY_OFFSET]);
         //lcd_putc('*');
         //lcd_puthex(spi_rxbuffer[0]);
         lcd_gotoxy(0,0);
         lcd_putc('b');
         //lcd_puthex(spi_rxbuffer[1]);
         lcd_puthex(buffer[0]);
         //lcd_putc('d');
         lcd_puthex(buffer[1]);
         lcd_puthex(buffer[2]);
         
         spi_txbuffer[0] = buffer[0];
         
         spi_txbuffer[1] = buffer[1];
         spi_txbuffer[2] = buffer[2];
         spi_txbuffer[3] = buffer[3];
         spi_txbuffer[4] = buffer[4];
         
         
         //lcd_puthex(buffer[0]);

         /*
         sendbuffer[0] = usb_readcount;
         sendbuffer[1] = 13;
         sendbuffer[2] = 17;
         sendbuffer[3] = 75;
         sendbuffer[5] = spi_rxbuffer[0];
         lcd_puthex(sendbuffer[5]);
          */
       //  lcd_putc('*');
        
         
      //   uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
         
        // lcd_gotoxy(0,1);
        // lcd_putc('*');
       //  lcd_putint(usberfolg);
       //  lcd_putc('*');
         
         

         usbstatus |= (1<<USB_RECV);
         {
            code = 0;//buffer[0];
            /*
            switch (code)
            {
               case 0xC0: // Write EEPROM Page start
               {
                  
                  usb_readcount=0;
                  abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  anzahlpakete = buffer[3];
                  //eepromstatus |= (1<<EE_WRITE);
                  lcd_gotoxy(8,0);
                  lcd_putc('E');
                  lcd_puthex(code);
                  lcd_putc('*');
                  lcd_putint(anzahlpakete);
                  lcd_putc('*');
                  lcd_putint(abschnittnummer);
                  sendbuffer[0] = 0xC1;
                  usb_rawhid_send((void*)sendbuffer, 50);
                  lcd_putc('*');
                  lcd_gotoxy(18,1);
                  lcd_putc('W');
                  lcd_putc('P');
                  
                  usbtask |= (1<<EEPROM_WRITE_PAGE_TASK);
                  masterstatus |= (1<<SUB_TASK_BIT);
               }break;
                  
                  
                  
                  
                  
                  // ---------------------------------------------------
                  // MARK: C4 Write EEPROM Byte
                  // ---------------------------------------------------
                  
               case 0xC4: // Write EEPROM Byte  // 10 ms
               {
                  //OSZI_A_TOGG;
                  usb_readcount=0;
                  //abschnittnummer++;
                  eepromstartadresse = buffer[1] | (buffer[2]<<8);
                  eeprom_databyte = buffer[3];
                  
                  //anzahlpakete = buffer[3];
                  //eepromstatus |= (1<<EE_WRITE);
                  lcd_gotoxy(18,1);
                  lcd_putc('W');
                  lcd_putc('1');
                  //lcd_putc('*');
                  
                  
                  
                  sendbuffer[0] = 0xC5;
                  
                  sendbuffer[1] = eepromstartadresse & 0xFF;
                  sendbuffer[2] = (eepromstartadresse & 0xFF00)>>8;
                  
                  sendbuffer[3] = buffer[3];
                  sendbuffer[4] = 0; //check;// ist bytechecksumme
                  sendbuffer[5] = usb_readcount;
                  
                  sendbuffer[6] = 0xFF;
                  sendbuffer[7] = 0xFF;
                  
                  sendbuffer[8] = 0xF8;
                  sendbuffer[9] = 0xFB;
                  
                  usb_rawhid_send((void*)sendbuffer, 50);
                  
                  masterstatus |= (1<<SUB_TASK_BIT);
                  //usbtask |= (1<<EEPROM_WRITE_BYTE_TASK);
                  
               }break;
                  // ---------------------------------------------------
             
                  
            } // switch code
             
             */
         }
         //OSZI_A_HI;
         //lcd_putc('$');
         code=0;
         sei();
         
         //OSZI_D_HI;
         OSZI_A_HI ;
		} // r>0, neue Daten
      else
      {
         //OSZI_B_LO;
      }
      
      /**	End USB-routinen	***********************/
 		
	    
      
		//OSZI_B_HI;
      
	}//while
   //free (sendbuffer);
   
   // return 0;
}
