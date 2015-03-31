#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "driver/uart.h"

#include "RF24.h"
#include "nRF24L01.h"

#define printf os_printf

ETSTimer timer;
/* address[0] for testing sending, and address[1] for testing listening */
const uint8_t address[][6] = { { 0xF2, 0xF2, 0xF2, 0xF2, 0x01, 0x00 },
			       { 0xF2, 0xF2, 0xF2, 0xF2, 0x02, 0x00 } };

struct softap_config _ap_conf = {
     .ssid = SSID,
     .password = SSID_PASSWORD,
     .ssid_len = 0,
     .channel = 1,
     .authmode = AUTH_WPA_WPA2_PSK,
     .ssid_hidden = 0,
     .max_connection = 4,
     .beacon_interval = 1,
};

void ICACHE_FLASH_ATTR
uart_putc1(char c) {
     uart0_tx_buffer(&c, 1);
}

/* Define to do 'listen' */
#define TEST_LISTEN

void ICACHE_FLASH_ATTR 
timer_fn (void *arg) {
     static unsigned char counter = 0;

#ifndef TEST_LISTEN /* Test send */
     rf24_powerUp();
     rf24_writeFast_noack("ESP8266 Here", 12, 1);
     rf24_txStandBy();
     rf24_powerDown();
#else
     uint8_t pipeNo;
     uint8_t buf[36];

     if (rf24_available_on_pipe(&pipeNo)) {
	  memset(buf, 0, 36);
	  rf24_read(buf, 32);
	  printf(":: [%s] Status: %s (p=%d)\r\n",
		 counter ? "XXXXXX" : ".......",
		 buf, pipeNo);
	  counter ^= 1;
     }
#endif
}

void ICACHE_FLASH_ATTR init_wifi () {
     bool rc;
     struct softap_config ap_conf = {0};

     wifi_set_opmode( SOFTAP_MODE );
     rc = wifi_softap_set_config (&_ap_conf);
     printf("\r\nwifi_softap_set_config = %d\r\n", rc);

     rc = wifi_softap_get_config (&ap_conf);
     if (rc != 0) {
	  printf("SSID = %s\r\n", ap_conf.ssid);
	  printf("PWD = %s\r\n", ap_conf.password);
	  printf("SSID_LEN = %d\r\n", ap_conf.ssid_len);
	  printf("Hidden = %d\r\n", ap_conf.ssid_hidden);
	  printf("Channel = %d\r\n", ap_conf.channel);
	  printf("Max_Conn = %d\r\n", ap_conf.max_connection);
	  printf("beacon interval = %d\r\n", ap_conf.beacon_interval);
     }

     rc = wifi_softap_set_config (&_ap_conf);
     printf("\r\nwifi_softap_set_config = %d\r\n", rc);

     rc = wifi_softap_dhcps_status();
     if (rc == DHCP_STOPPED) {
	  printf("Starting dhcps");
	  wifi_softap_dhcps_start();
     }
}

void ICACHE_FLASH_ATTR
user_init()
{
     uart_init(BIT_RATE_115200, BIT_RATE_115200);
     os_install_putc1((void *)uart_putc1);
     init_wifi();

     /* Ummm... its kinda wired in RF24.c :: rf24_begin(), chose GPIO
      * 4 and 5 since those appeared un-encumbered by other functions
      * and weirdness */
     rf24_init(4, 5);
     rf24_begin();
     rf24_printDetails();

     rf24_setAutoAck(0);
     rf24_setPALevel(RF24_PA_MIN);
     rf24_setRetries(0,0);

#ifndef TEST_LISTEN
     rf24_openWritingPipe_v(address[0]);
#else
     rf24_openReadingPipe_v(1, address[1]);
     rf24_startListening();
#endif

     os_timer_setfn(&timer, timer_fn, NULL);
     os_timer_arm(&timer, 100, 1);
}
