/*
 * main.cpp
 *
 * Author : deanm
 */ 

#include "qpcpp.h"

#include "event.h"
#include "bsp.h"
#include "fw_macro.h"

#include "System.h"
#include "AOLED.h"

using namespace QP;

//* ============== POOL SIZES =================== *//
#define EVT_SIZE_SMALL 32
#define EVT_SIZE_MEDIUM 64
#define EVT_SIZE_LARGE 256
#define EVT_COUNT_SMALL 128
#define EVT_COUNT_MEDIUM 64
#define EVT_COUNT_LARGE 8

uint32_t evtPoolSmall[ROUND_UP_DIV_4(EVT_SIZE_SMALL * EVT_COUNT_SMALL)];
uint32_t evtPoolMedium[ROUND_UP_DIV_4(EVT_SIZE_MEDIUM * EVT_COUNT_MEDIUM)];
uint32_t evtPoolLarge[ROUND_UP_DIV_4(EVT_SIZE_LARGE * EVT_COUNT_LARGE)];
QP::QSubscrList subscrSto[MAX_PUB_SIG];

static System sys;
static AOLED led;

int main(void)
{
	QF::init();
	QF::poolInit(evtPoolSmall, sizeof(evtPoolSmall), EVT_SIZE_SMALL);
	QF::poolInit(evtPoolMedium, sizeof(evtPoolMedium), EVT_SIZE_MEDIUM);
	QF::poolInit(evtPoolLarge, sizeof(evtPoolLarge), EVT_SIZE_LARGE);
	QP::QF::psInit(subscrSto, Q_DIM(subscrSto)); // init publish-subscribe

	BspInit();

	//Start active objects.
	sys.Start(PRIO_SYSTEM);
	led.Start(PRIO_LED);

	//publish a start request
	Evt *evt = new SystemStartReq(0);
	QF::PUBLISH(evt, dummy);

	QP::QF::run();
}
