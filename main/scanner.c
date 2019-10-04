/* 
  2.4 Ghz Wifi Traffic Scanner using ESP32-S AI-Thinker WROOM-32 Board.
  The  RSSI value of all received packets is evaluated and the blink frequency of onboard LED will increase proportional. 
  Thomas Krueger 2019
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"   // this file contains the format of received packets in monitor/promicuous mode! in:esp/esp-idf/components/esp32/include
#include "nvs_flash.h"


// helper structs to overlay received payload, if MGMT type frame was received:
typedef struct {
	unsigned frame_ctrl:16;
	unsigned duration_id:16;
	uint8_t addr1[6]; /* receiver address */
	uint8_t addr2[6]; /* sender address */
	uint8_t addr3[6]; /* filtering address */
	unsigned sequence_ctrl:16;
	uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
	wifi_ieee80211_mac_hdr_t hdr;
	uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;


//set the filter for received packets in monitor mode(which packets do i want to be received?), here all packets are received
const wifi_promiscuous_filter_t filt={ 
    .filter_mask=WIFI_PROMIS_FILTER_MASK_ALL
};

IRAM_ATTR void scanner_cb(void* buf, wifi_promiscuous_pkt_type_t type);

TimerHandle_t tmr;
void timerCallBack( TimerHandle_t xTimer );

// LED blink settings:  ms are actually ms*10 in esp32, or generally tickvalue/portTICK_PERIOD_MS
int ton=1;  // ms the led shall be on. (*10)
int toffmax=500; //max-ms the led shall be off  (*10)
int toffnow; //current-ms the led shall be off
int state,expired; // will be init to 0

volatile int8_t maxrssi=-90;

volatile uint8_t maxchan;

unsigned lockchan=0;
uint8_t channel = 1;
	
void app_main()
{
  int counter=0;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  int delayms = 50;

	/* setup */

// WIFI init

  nvs_flash_init(); // this needs to be there,otherwise not work. dont know why? shopuld have been done in startup();
  esp_wifi_init(&cfg); //init default mode
  esp_wifi_set_storage(WIFI_STORAGE_RAM); // capture frames in RAM
  esp_wifi_set_mode(WIFI_MODE_NULL);  // there is a dispute about if WIFI_MODE_NULL works.!
  
  esp_wifi_set_promiscuous(true);  // first, set monitor mode. promiscuous is actually monitor mode!
//  esp_wifi_set_promiscuous_filter(&filt);  // set receive filter to: all packets
  esp_wifi_set_promiscuous_rx_cb(&scanner_cb); // set the callback service routine for received frames.
  ESP_ERROR_CHECK( esp_wifi_start() ); // ESP_ERROR_CHECK will abort() if result is NOT ESP_OK, thus leaving app_main from this point.
 

// LED init and blink timer
	gpio_set_direction(4, GPIO_MODE_OUTPUT); // set portpin to output
	gpio_set_level(4, 0); // turn led off
	state=0;
	
	toffnow=toffmax; // preset to slowest blink freuency
	//NOTE: the RTOS tick is configured to 10ms. so if you set timerperiod to 1, then its actually 10ms!!
    tmr = xTimerCreate("MyTimer", 1, pdTRUE, (void *)0, &timerCallBack); // create a 1ms software timer
    xTimerStart(tmr,0);  // and start it
 
	/* loop */
	while (1) 
	{
//		gpio_set_level(4, level ^= 1); // toggle test
		esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
		vTaskDelay(delayms/portTICK_PERIOD_MS);  // wait a little in millisecs
		channel = (channel % 14) + 1;
		
// adjust blink frequency asto signal strength:	(experimental!)	
// usually wifi activity is pretty quiet with occasional frames being sent. So this detector works solala
		if (maxrssi > -65) // minimum detectable signal strength, about 4 meter away
		{
			toffnow=ton+100; // 3 meter away 1Hz
			if (maxrssi > -60) toffnow=ton+80; // 2 meter away 6Hz
			if (maxrssi > -50) toffnow=ton+70; // 1 meter away 7Hz
			if (maxrssi > -45) toffnow=ton+50; // 80 cm away 10Hz
			if (maxrssi > -40) toffnow=ton+30; // 60cm away  16Hz
			if (maxrssi > -35) toffnow=ton+10; // 40cm away  25Hz
			if (maxrssi > -30) toffnow=ton;    // 10cm away  50Hz
			

		}
//fallback to default value blink frequency after 2 seconds,then reacumulate again.otherwise you stay on maxvalue.		
		counter++;
		if (counter > (2000/delayms)) //reset blink frequency after about 2 seconds (msecs/delayms)
		{
			maxrssi=-90;
			maxchan=0;
			toffnow=toffmax; //5 secs = no signal in reach
			counter=0;
		}
		
    }
}

// blink frequency proportional to signal strength
// note: toffnow must always be >= ton !!(time-on)
void timerCallBack( TimerHandle_t xTimer )
{
	expired++; // count the timer expires
	
//	gpio_set_level(4, state ^= 1); //toggle test
//	return;
	
	// shall i turn off the led?
	if (expired >= ton)
	{
		if (state==1) gpio_set_level(4, 0); // turn led off
		state=0;
	}
	
	if (expired > toffnow) // has the led interval expired? ...then turn led on
	{
		if (state==0) gpio_set_level(4, 1); // turn led on
		state=1;
		expired =0;
			
	}
}



/* callback, if wifi-frames are received into RAM. This code shall be placed in RAM for faster execution! It really makes a difference!
 the *buf pointer is of type wifi_promiscuous_pkt_t
 * typedef struct {
    wifi_pkt_rx_ctrl_t rx_ctrl; // metadata header 
    uint8_t payload[0];       // Data or management payload. Length of payload is described by rx_ctrl.sig_len. Type of content determined by packet type argument of callback.
} wifi_promiscuous_pkt_t;

RSSI values: strong = -20dbm, weak = -80dbm
 */
IRAM_ATTR void scanner_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
	int currssi;
	unsigned int curchan;
	wifi_promiscuous_pkt_t *pbuf = buf; //convert the pointer
	

/*	
// Debug Output:
	wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)pbuf->payload; // seperate rx-frame into mac-header and payload
	wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr; // overlay the mac-header part  with struct2



	if (type == WIFI_PKT_MGMT) // otherwise we are getting fludded with printf's
	printf("PACKET TYPE=MGMT, CHAN=%02d, RSSI=%02d,"
		" ToMAC=%02x:%02x:%02x:%02x:%02x:%02x,"
		" FromMAC=%02x:%02x:%02x:%02x:%02x:%02x,"
		" SDMAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
		pbuf->rx_ctrl.channel,
		pbuf->rx_ctrl.rssi,
		//addr1
		hdr->addr1[0],hdr->addr1[1],hdr->addr1[2],
		hdr->addr1[3],hdr->addr1[4],hdr->addr1[5],
		//addr2
		hdr->addr2[0],hdr->addr2[1],hdr->addr2[2],
		hdr->addr2[3],hdr->addr2[4],hdr->addr2[5],
		//addr3
		hdr->addr3[0],hdr->addr3[1],hdr->addr3[2],
		hdr->addr3[3],hdr->addr3[4],hdr->addr3[5]
	);
*/	
	
	
	
	curchan=pbuf->rx_ctrl.channel; // get primary channel
	currssi=pbuf->rx_ctrl.rssi; // get receive signal strenth
	if (currssi > -60) // signal is nearby
	{
		if (currssi > maxrssi)
		{
			maxrssi = currssi;
			maxchan = curchan;
			
		}
	}
	
//	toffnow=20; //test
//gpio_set_level(4, state ^= 1); //toggle test


}
