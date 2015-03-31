/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "ets_sys.h"

#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"

#include "gpio.h"
#include "driver/spi.h"
#include "driver/spi_register.h"

/* Only one instance, currently */
uint8_t ce_pin;
uint8_t csn_pin;
bool p_variant;
uint8_t payload_size;
bool dynamic_payloads_enabled;
uint8_t pipe0_reading_address[5];
uint8_t addr_width;

#if defined (__XTENSA__)

/* Defines to look like 'wiring' library */
#define HIGH 1
#define LOW 0
#define delay(x) ets_delay_us((x) * 1000)
#define delayMicroseconds ets_delay_us
#define min(a, b) ((a) > (b) ? (b) : (a))

LOCAL void ICACHE_FLASH_ATTR
SPI_begin () {
     spi_master_init(HSPI);
     /* 1 Mhz-ish clock, I think */
     WRITE_PERI_REG( SPI_CLOCK(HSPI),
		    (((0x40) & SPI_CLKDIV_PRE) << SPI_CLKDIV_PRE_S)|
		    ((3 & SPI_CLKCNT_N) << SPI_CLKCNT_N_S)|
		    ((1 & SPI_CLKCNT_H) << SPI_CLKCNT_H_S)|
		    ((3 & SPI_CLKCNT_L) << SPI_CLKCNT_L_S));
}

LOCAL unsigned char ICACHE_FLASH_ATTR
SPI_transfer (unsigned char v) {
     unsigned char d = 0;

     while (READ_PERI_REG(SPI_CMD(HSPI)) & SPI_USR);

     /* BIT(0) from Old SDK (IN and OUT) to do duplex transfer, disable all other
      * bytes (MISO - happens when set to MOSI and duplex,
      * COMMAND, ADDR, DUMMY */
     SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MOSI | BIT(0) );
     CLEAR_PERI_REG_MASK(SPI_USER(HSPI),
			 SPI_USR_MISO |
			 SPI_USR_COMMAND |
			 SPI_USR_ADDR |
			 SPI_USR_DUMMY);

     /* 8 bit xfers */
     WRITE_PERI_REG(SPI_USER1(HSPI),
		    ((7 & SPI_USR_MOSI_BITLEN) << SPI_USR_MOSI_BITLEN_S) |
		    ((7 & SPI_USR_MISO_BITLEN) << SPI_USR_MISO_BITLEN_S));

     /* To send */
     WRITE_PERI_REG(SPI_W0(HSPI), (uint32)(v));
     SET_PERI_REG_MASK(SPI_CMD(HSPI), SPI_USR);

     /* Wait for operation (read included) to complete */
     while (READ_PERI_REG(SPI_CMD(HSPI))&SPI_USR);
     d = (uint8)(READ_PERI_REG(SPI_W0(HSPI))&0xff);

     return d;
}

LOCAL long ICACHE_FLASH_ATTR
millis() {
     uint32 us = system_get_time() / 1000;
     return us;
}

#endif

/****************************************************************************/

void ICACHE_FLASH_ATTR
csn(bool mode)
{
	// Minimum ideal SPI bus speed is 2x data rate
	// If we assume 2Mbs data rate and 16Mhz clock, a
	// divider of 4 is the minimum we want.
	// CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
  GPIO_OUTPUT_SET(csn_pin, mode);
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
ce(bool level)
{
  GPIO_OUTPUT_SET(ce_pin, level);
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_read_register_v(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  csn(LOW);
  status = SPI_transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- ){
    *buf++ = SPI_transfer(0xff);
  }
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_read_register(uint8_t reg)
{

  csn(LOW);
  SPI_transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  uint8_t result = SPI_transfer(0xff);
  csn(HIGH);

  return result;
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_write_register_v(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  csn(LOW);
  status = SPI_transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    SPI_transfer(*buf++);

  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  csn(LOW);
  status = SPI_transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI_transfer(value);
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_write_payload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
  uint8_t status;
  const uint8_t* current = (const uint8_t*)(buf);

  if(data_len > 32) data_len = 32;
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : 32 - data_len;

  csn(LOW);
  status = SPI_transfer( writeType );
  while ( data_len-- ) {
    SPI_transfer(*current++);
  }
  while ( blank_len-- ) {
    SPI_transfer(0);
  }
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_read_payload(void* buf, uint8_t data_len)
{
  uint8_t status;
  uint8_t* current = (uint8_t*)(buf);

  if(data_len > payload_size) data_len = payload_size;
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : 32 - data_len;

	//printf("[Reading %u bytes %u blanks]",data_len,blank_len);

  csn(LOW);
  status = SPI_transfer( R_RX_PAYLOAD );
  while ( data_len-- ) {
    *current++ = SPI_transfer(0xFF);
  }
  while ( blank_len-- ) {
    SPI_transfer(0xff);
  }
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
flush_rx(void)
{
  return rf24_spiTrans( FLUSH_RX );
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
flush_tx(void)
{
  return rf24_spiTrans( FLUSH_TX );
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_spiTrans(uint8_t cmd){

  uint8_t status;

  csn(LOW);
  status = SPI_transfer( cmd );
  csn(HIGH);

  return status;

}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_get_status(void)
{
  return rf24_spiTrans(NOP);
}

#ifndef MINIMAL_NO_TEXT
/****************************************************************************/
void ICACHE_FLASH_ATTR
rf24_print_status(uint8_t status)
{
     printf_P(PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0b111),
           (status & _BV(TX_FULL))?1:0
          );
}

/****************************************************************************/

void rf24_print_observe_tx(uint8_t value)
{
	// printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"),
	//          value,
	//          (value >> PLOS_CNT) & B1111,
	//          (value >> ARC_CNT) & B1111
	//         );
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
     char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
     printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
     while (qty--)
	  printf_P(PSTR(" 0x%02x"),rf24_read_register(reg++));
     printf_P(PSTR("\r\n"));
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
  printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);

  while (qty--)
  {
    uint8_t buffer[addr_width];
    rf24_read_register_v(reg++,buffer,sizeof buffer);

    printf_P(PSTR(" 0x"));
    uint8_t* bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer ) {
      printf_P(PSTR("%02x"),*bufptr);
    }
  }

  printf_P(PSTR("\r\n"));
}
#endif

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_init(uint8_t _cepin, uint8_t _cspin)
{
     ce_pin = _cepin;
     csn_pin = _cspin;
     p_variant = false;
     payload_size = 32;
     dynamic_payloads_enabled = false;
     addr_width = 5;	//,pipe0_reading_address(0);
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_setChannel(uint8_t channel)
{
  const uint8_t max_channel = 127;
  rf24_write_register(RF_CH,min(channel,max_channel));
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_setPayloadSize(uint8_t size)
{
  payload_size = min(size,32);
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_getPayloadSize(void)
{
  return payload_size;
}

/****************************************************************************/

#if !defined (MINIMAL_NO_TEXT)

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = {
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};

void ICACHE_FLASH_ATTR
rf24_printDetails(void)
{
  rf24_print_status(rf24_get_status());

  rf24_print_address_register(PSTR("RX_ADDR_P0-1"),RX_ADDR_P0,2);
  rf24_print_byte_register(PSTR("RX_ADDR_P2-5"),RX_ADDR_P2,4);
  rf24_print_address_register(PSTR("TX_ADDR"),TX_ADDR, 1);

  rf24_print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
  rf24_print_byte_register(PSTR("EN_AA"),EN_AA, 1);
  rf24_print_byte_register(PSTR("EN_RXADDR"),EN_RXADDR, 1);
  rf24_print_byte_register(PSTR("RF_CH"),RF_CH, 1);
  rf24_print_byte_register(PSTR("RF_SETUP"),RF_SETUP, 1);
  rf24_print_byte_register(PSTR("CONFIG"),CONFIG, 1);
  rf24_print_byte_register(PSTR("DYNPD/FEATURE"),DYNPD,2);

#if defined(__arm__)
  printf_P(PSTR("Data Rate\t = %s\r\n"),pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
  printf_P(PSTR("Model\t\t = %s\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
  printf_P(PSTR("CRC Length\t = %s\r\n"),pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
  printf_P(PSTR("PA Power\t = %s\r\n"),pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
#elif defined (__XTENSA__)
  printf("Data Rate\t = %s\r\n",rf24_datarate_e_str_P[rf24_getDataRate()]);
  printf("Model\t\t = %s\r\n",rf24_model_e_str_P[rf24_isPVariant()]);
  printf("CRC Length\t = %s\r\n",rf24_crclength_e_str_P[rf24_getCRCLength()]);
  printf("PA Power\t = %s\r\n",rf24_pa_dbm_e_str_P[rf24_getPALevel()]);
#else
  printf_P(PSTR("Data Rate\t = %S\r\n"),pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
  printf_P(PSTR("Model\t\t = %S\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
  printf_P(PSTR("CRC Length\t = %S\r\n"),pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
  printf_P(PSTR("PA Power\t = %S\r\n"),pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
#endif
}

#endif
/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_begin(void)
{
  SPI_begin();
  gpio_init ();
	// Initialize pins
  /* TODO: Have to move this out */
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
  gpio_output_set(BIT5, BIT4, BIT4 | BIT5, 0);

  ce(LOW);
  csn(HIGH);

	// Must allow the radio time to settle else configuration bits will not necessarily stick.
	// This is actually only required following power up but some settling time also appears to
	// be required after resets too. For full coverage, we'll always assume the worst.
	// Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
	// Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
	// WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay( 5 ) ;

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
  rf24_setRetries(5,15);

	// Reset value is MAX
	//setPALevel( RF24_PA_MAX ) ;

	// Determine if this is a p or non-p RF24 module and then
	// reset our data rate back to default value. This works
	// because a non-P variant won't allow the data rate to
	// be set to 250Kbps.
  if( rf24_setDataRate( RF24_250KBPS ) )
  {
    p_variant = true ;
  }

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
  rf24_setDataRate( RF24_1MBPS ) ;

	// Initialize CRC and request 2-byte (16bit) CRC
  rf24_setCRCLength( RF24_CRC_16 ) ;

	// Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
	//rf24_write_register(DYNPD,0);

	// Reset current status
	// Notice reset and flush is the last thing we do
  rf24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
  rf24_setChannel(76);

	// Flush buffers
  flush_rx();
  flush_tx();

  rf24_powerUp();	//Power up by default when begin() is called

	// Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from rf24_powerUp )
	// PTX should use only 22uA of power
  rf24_write_register(CONFIG, ( rf24_read_register(CONFIG) ) & ~_BV(PRIM_RX) );

}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_startListening(void)
{
  rf24_powerUp();
  rf24_write_register(CONFIG, rf24_read_register(CONFIG) | _BV(PRIM_RX));
  rf24_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	// Restore the pipe0 adddress, if exists
  if (pipe0_reading_address[0] > 0){
    rf24_write_register_v(RX_ADDR_P0, pipe0_reading_address, addr_width);
  }

	// Flush buffers
	//flush_rx();
  flush_tx();

	// Go!
  ce(HIGH);

}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_stopListening(void)
{

  ce(LOW);
  #if defined(__arm__)
  delayMicroseconds(130);
  #endif
  flush_tx();
	//flush_rx();

  rf24_write_register(CONFIG, ( rf24_read_register(CONFIG) ) & ~_BV(PRIM_RX) );
  delayMicroseconds(130);	//Found that adding this delay back actually increases response time
}

/****************************************************************************/

void rf24_powerDown(void)
{
  ce(LOW);	// Guarantee CE is low on powerDown
  rf24_write_register(CONFIG,rf24_read_register(CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void ICACHE_FLASH_ATTR
rf24_powerUp(void)
{
   uint8_t cfg = rf24_read_register(CONFIG);

	// if not powered up then power up and wait for the radio to initialize
   if (!(cfg & _BV(PWR_UP))){
      rf24_write_register(CONFIG,rf24_read_register(CONFIG) | _BV(PWR_UP));

	// For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	// There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	// the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
      delay(5);

   }
}

/******************************************************************/
#if defined (FAILURE_HANDLING)
void ICACHE_FLASH_ATTR
rf24_errNotify(){
	IF_SERIAL_DEBUG(printf_P(PSTR("HARDWARE FAIL\r\n")));
	failureDetected = 1;
}
#endif
/******************************************************************/

//Similar to the previous write, clears the interrupt flags
bool ICACHE_FLASH_ATTR
rf24_write_v( const void* buf, uint8_t len, const bool multicast )
{
	//Start Writing
	rf24_startFastWrite(buf,len,multicast);

	//Wait until complete or failed
	#if defined (FAILURE_HANDLING)
		uint32_t timer = millis();
	#endif

	while( ! ( rf24_get_status()  & ( _BV(TX_DS) | _BV(MAX_RT) ))) {
		#if defined (FAILURE_HANDLING)
			if(millis() - timer > 75){
				errNotify();
				return 0;
			}
		#endif
	}

	ce(LOW);

	uint8_t status = rf24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	//Max retries exceeded
  if( status & _BV(MAX_RT)){
	     flush_tx();	//Only going to be 1 packet int the FIFO at a time using this method, so just flush
	     return 0;
  }
	//TX OK 1 or 0
  return 1;
}

bool ICACHE_FLASH_ATTR
rf24_write( const void* buf, uint8_t len ){
	return rf24_writeBlocking(buf,len,0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool ICACHE_FLASH_ATTR
rf24_writeBlocking( const void* buf, uint8_t len, uint32_t timeout )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//This way the FIFO will fill up and allow blocking until packets go through
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	uint32_t timer = millis();	//Get the time that the payload transmission started

	while( ( rf24_get_status()  & ( _BV(TX_FULL) ))) {	//Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

		if( rf24_get_status() & _BV(MAX_RT)){	//If MAX Retries have been reached
			rf24_reUseTX();	//Set re-transmit and clear the MAX_RT interrupt flag
			if(millis() - timer > timeout){ return 0; }	//If this payload has exceeded the user-defined timeout, exit and return 0
		}
		#if defined (FAILURE_HANDLING)
			if(millis() - timer > (timeout+75) ){
				errNotify();
				return 0;
			}
		#endif

  	}

	//Start Writing
	rf24_startFastWrite(buf,len,0);	//Write the payload if a buffer is clear

	return 1;	//Return 1 to indicate successful transmission
}

/****************************************************************************/

void rf24_reUseTX(){
		rf24_write_register(STATUS,_BV(MAX_RT) );	//Clear max retry flag
		rf24_spiTrans( REUSE_TX_PL );
		ce(LOW);	//Re-Transfer packet
		ce(HIGH);
}

/****************************************************************************/

bool ICACHE_FLASH_ATTR
rf24_writeFast_noack( const void* buf, uint8_t len, const bool multicast )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//Return 0 so the user can control the retrys and set a timer or failure counter if required
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	#if defined (FAILURE_HANDLING)
		uint32_t timer = millis();
	#endif

	while( ( rf24_get_status()  & ( _BV(TX_FULL) ))) {	//Blocking only if FIFO is full. This will loop and block until TX is successful or fail

		if( rf24_get_status() & _BV(MAX_RT)){
	//reUseTX();	//Set re-transmit
			rf24_write_register(STATUS,_BV(MAX_RT) );	//Clear max retry flag
			return 0;	//Return 0. The previous payload has been retransmitted
	//From the user perspective, if you get a 0, just keep trying to send the same payload
		}
		#if defined (FAILURE_HANDLING)
			if(millis() - timer > 75 ){
				errNotify();
				return 0;
			}
		#endif
  	}
	//Start Writing
	rf24_startFastWrite(buf,len,multicast);

	return 1;
}

bool rf24_writeFast( const void* buf, uint8_t len ){
	return rf24_writeFast_noack(buf,len,0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void rf24_startFastWrite( const void* buf, uint8_t len, const bool multicast){	//TMRh20

	//write_payload( buf,len);
	rf24_write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	ce(HIGH);

}


//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
void ICACHE_FLASH_ATTR
rf24_startWrite( const void* buf, uint8_t len, const bool multicast ){

	// Send the payload

	//write_payload( buf, len );
  rf24_write_payload( buf, len,multicast? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
  ce(HIGH);
  #if defined(CORE_TEENSY) || !defined(ARDUINO)
	delayMicroseconds(10);
  #endif
  ce(LOW);


}

bool ICACHE_FLASH_ATTR
rf24_txStandBy(){
    #if defined (FAILURE_HANDLING)
		uint32_t timeout = millis();
	#endif
	while( ! (rf24_read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
		if( rf24_get_status() & _BV(MAX_RT)){
			rf24_write_register(STATUS,_BV(MAX_RT) );
			ce(LOW);
			flush_tx();	//Non blocking, flush the data
			return 0;
		}
		#if defined (FAILURE_HANDLING)
			if( millis() - timeout > 75){
				errNotify();
				return 0;
			}
		#endif
	}

	ce(LOW);	//Set STANDBY-I mode
	return 1;
}

bool rf24_txStandBy_retry(uint32_t timeout){

	uint32_t start = millis();

	while( ! (rf24_read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
		if( rf24_get_status() & _BV(MAX_RT)){
			rf24_write_register(STATUS,_BV(MAX_RT) );
				ce(LOW);	//Set re-transmit
				ce(HIGH);
				if(millis() - start >= timeout){
					ce(LOW); flush_tx(); return 0;
				}
		}
		#if defined (FAILURE_HANDLING)
			if( millis() - start > (timeout+75)){
				errNotify();
				return 0;
			}
		#endif
	}


	ce(LOW);	//Set STANDBY-I mode
	return 1;

}
/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_maskIRQ(bool tx, bool fail, bool rx){

	rf24_write_register(CONFIG, ( rf24_read_register(CONFIG) ) | fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR  );
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  csn(LOW);
  SPI_transfer( R_RX_PL_WID );
  result = SPI_transfer(0xff);
  csn(HIGH);

  if(result > 32) { flush_rx(); return 0; }
  return result;
}

/****************************************************************************/

bool ICACHE_FLASH_ATTR
rf24_available(void)
{
  return rf24_available_on_pipe(NULL);
}

/****************************************************************************/

bool rf24_available_on_pipe(uint8_t* pipe_num)
{

	//Check the FIFO buffer to see if data is waitng to be read

  if (!( rf24_read_register(FIFO_STATUS) & _BV(RX_EMPTY) )){

	// If the caller wants the pipe number, include that
    if ( pipe_num ){
	  uint8_t status = rf24_get_status();
      *pipe_num = ( status >> RX_P_NO ) & 0B111;
  	}
  	return 1;
  }


  return 0;


}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_read( void* buf, uint8_t len ){

	// Fetch the payload
  rf24_read_payload( buf, len );

	//Clear the two possible interrupt flags with one command
  rf24_write_register(STATUS,_BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );

}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_whatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready)
{
	// Read the status & reset the status in one easy call
	// Or is that such a good idea?
  uint8_t status = rf24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	// Report to the user what happened
  *tx_ok = status & _BV(TX_DS);
  *tx_fail = status & _BV(MAX_RT);
  *rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_openWritingPipe(uint64_t value)
{
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.

     rf24_write_register_v(RX_ADDR_P0, (uint8_t*)(&value), addr_width);
     rf24_write_register_v(TX_ADDR, (uint8_t*)(&value), addr_width);

	//const uint8_t max_payload_size = 32;
	//rf24_write_register(RX_PW_P0,min(payload_size,max_payload_size));
     rf24_write_register(RX_PW_P0,payload_size);
}

/****************************************************************************/
void ICACHE_FLASH_ATTR
rf24_openWritingPipe_v(const uint8_t *address)
{
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.

  rf24_write_register_v(RX_ADDR_P0,address, addr_width);
  rf24_write_register_v(TX_ADDR, address, addr_width);

	//const uint8_t max_payload_size = 32;
	//rf24_write_register(RX_PW_P0,min(payload_size,max_payload_size));
  rf24_write_register(RX_PW_P0,payload_size);
}

/****************************************************************************/
static const uint8_t child_pipe[] PROGMEM =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] PROGMEM =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] PROGMEM =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void ICACHE_FLASH_ATTR
rf24_openReadingPipe(uint8_t child, uint64_t address)
{
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,&address,addr_width);
  }

  if (child <= 6)
  {
	// For pipes 2-5, only write the LSB
    if ( child < 2 )
	 rf24_write_register_v(pgm_read_byte(&child_pipe[child]), (const uint8_t*)(&address), addr_width);
    else
	 rf24_write_register_v(pgm_read_byte(&child_pipe[child]), (const uint8_t*)(&address), 1);

    rf24_write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

	// Note it would be more efficient to set all of the bits for all open
	// pipes at once.  However, I thought it would make the calling code
	// more simple to do it this way.
    rf24_write_register(EN_RXADDR,rf24_read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
  }
}

/****************************************************************************/
void ICACHE_FLASH_ATTR
rf24_setAddressWidth(uint8_t a_width){

	if(a_width -= 2){
		rf24_write_register(SETUP_AW,a_width%4);
		addr_width = (a_width%4) + 2;
	}

}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_openReadingPipe_v(uint8_t child, const uint8_t *address)
{
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,address,addr_width);
  }
  if (child <= 6)
  {
	// For pipes 2-5, only write the LSB
    if ( child < 2 ){
      rf24_write_register_v(pgm_read_byte(&child_pipe[child]), address, addr_width);
    }else{
      rf24_write_register_v(pgm_read_byte(&child_pipe[child]), address, 1);
	}
    rf24_write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

	// Note it would be more efficient to set all of the bits for all open
	// pipes at once.  However, I thought it would make the calling code
	// more simple to do it this way.
    rf24_write_register(EN_RXADDR,rf24_read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

  }
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_closeReadingPipe( uint8_t pipe )
{
  rf24_write_register(EN_RXADDR,rf24_read_register(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_toggle_features(void)
{
  csn(LOW);
  SPI_transfer( ACTIVATE );
  SPI_transfer( 0x73 );
  csn(HIGH);
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_enableDynamicPayloads(void)
{
	// Enable dynamic payload throughout the system

    rf24_toggle_features();
    rf24_write_register(FEATURE,rf24_read_register(FEATURE) | _BV(EN_DPL) );


  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",rf24_read_register(FEATURE)));

	// Enable dynamic payload on all pipes
	//
	// Not sure the use case of only having dynamic payload on certain
	// pipes, so the library does not support it.
  rf24_write_register(DYNPD,rf24_read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_enableAckPayload(void)
{
	//
	// enable ack payload and dynamic payload features
	//

    rf24_toggle_features();
    rf24_write_register(FEATURE,rf24_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",rf24_read_register(FEATURE)));

	//
	// Enable dynamic payload on pipes 0 & 1
	//

  rf24_write_register(DYNPD,rf24_read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_enableDynamicAck(void){
	//
	// enable dynamic ack features
	//
    rf24_toggle_features();
    rf24_write_register(FEATURE,rf24_read_register(FEATURE) | _BV(EN_DYN_ACK) );

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",rf24_read_register(FEATURE)));


}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
     const uint8_t* current = (const uint8_t*)(buf);

  uint8_t data_len = min(len,32);

  csn(LOW);
  SPI_transfer(W_ACK_PAYLOAD | ( pipe & 0B111 ) );

  while ( data_len-- )
    SPI_transfer(*current++);

  csn(HIGH);
}

/****************************************************************************/

bool ICACHE_FLASH_ATTR
rf24_isAckPayloadAvailable(void)
{
  return ! rf24_read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/

bool ICACHE_FLASH_ATTR
rf24_isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_setAutoAck(bool enable)
{
  if ( enable )
    rf24_write_register(EN_AA, 0B111111);
  else
    rf24_write_register(EN_AA, 0);
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_setAutoAck_on_pipe( uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = rf24_read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    rf24_write_register( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

bool ICACHE_FLASH_ATTR
rf24_testCarrier(void)
{
  return ( rf24_read_register(CD) & 1 );
}

/****************************************************************************/

bool ICACHE_FLASH_ATTR
rf24_testRPD(void)
{
  return ( rf24_read_register(RPD) & 1 ) ;
}

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_setPALevel(uint8_t level)
{

  uint8_t setup = rf24_read_register(RF_SETUP) & 0b11111000;

  if(level > 3){	// If invalid level, go to max PA
	  level = (RF24_PA_MAX << 1) + 1;	// +1 to support the SI24R1 chip extra bit
  }else{
	  level = (level << 1) + 1;	// Else set level as requested
  }


  rf24_write_register( RF_SETUP, setup |= level ) ;	// Write it to the chip
}

/****************************************************************************/

uint8_t ICACHE_FLASH_ATTR
rf24_getPALevel(void)
{

  return (rf24_read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}

/****************************************************************************/

bool ICACHE_FLASH_ATTR
rf24_setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = rf24_read_register(RF_SETUP) ;

	// HIGH and LOW '00' is 1Mbs - our default
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
	// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
	// Making it '10'.
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
	// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
	// Making it '01'
    if ( speed == RF24_2MBPS )
    {
      setup |= _BV(RF_DR_HIGH);
    }
  }
  rf24_write_register(RF_SETUP,setup);

	// Verify our result
  if ( rf24_read_register(RF_SETUP) == setup )
  {
    result = true;
  }

  return result;
}

#ifndef MINIMAL_NO_FULL_API
/****************************************************************************/

rf24_datarate_e ICACHE_FLASH_ATTR
rf24_getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = rf24_read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

	// switch uses RAM (evil!)
	// Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
	// '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
	// '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
	// '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}
#endif

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_setCRCLength(rf24_crclength_e length)
{
  uint8_t config = rf24_read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

	// switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
	// Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  rf24_write_register( CONFIG, config ) ;
}

#ifndef MINIMAL_NO_FULL_API
/****************************************************************************/

rf24_crclength_e ICACHE_FLASH_ATTR
rf24_getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = rf24_read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}
#endif

/****************************************************************************/

void ICACHE_FLASH_ATTR
rf24_disableCRC( void )
{
  uint8_t disable = rf24_read_register(CONFIG) & ~_BV(EN_CRC) ;
  rf24_write_register( CONFIG, disable ) ;
}

/****************************************************************************/
void ICACHE_FLASH_ATTR
rf24_setRetries(uint8_t delay, uint8_t count)
{
 rf24_write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

bool ICACHE_FLASH_ATTR
rf24_isValid() { return ce_pin != 0xff && csn_pin != 0xff; }
