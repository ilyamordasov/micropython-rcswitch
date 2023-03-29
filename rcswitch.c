#include "py/runtime.h"
#include "py/mphal.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h" // for esp-idf v5

extern bool mp_sched_schedule(mp_obj_t function, mp_obj_t arg);

// ----------------------------------------------------------------

typedef struct _rcswitch_HighLow_obj_t {
    mp_uint_t high;
    mp_uint_t low;
} rcswitch_HighLow_obj_t;

typedef struct _rcswitch_Protocol_obj_t {
    /** base pulse length in microseconds, e.g. 350 */
    mp_uint_t pulseLength;

    rcswitch_HighLow_obj_t syncFactor;
    rcswitch_HighLow_obj_t zero;
    rcswitch_HighLow_obj_t one;

    bool invertedSignal;
} rcswitch_Protocol_obj_t;

#define RCSWITCH_MAX_CHANGES 67
#define ESP_INTR_FLAG_DEFAULT 0
#define TAG "RF433"

typedef struct _rcswitch_RCSwitch_obj_t {
    // All objects start with the base.
    mp_obj_base_t base;
	gpio_num_t id;

    mp_uint_t nReceivedValue;
	mp_uint_t nReceivedBitlength;
	mp_uint_t nReceivedDelay;
	mp_uint_t nReceivedProtocol;
	mp_int_t nReceiveTolerance;
	mp_uint_t nSeparationLimit;
	/*
	 * timings[0] contains sync timing, followed by a number of bits
	 */
	mp_uint_t timings[RCSWITCH_MAX_CHANGES];
	mp_int_t nReceiverInterrupt;

	mp_int_t nTransmitterPin;
	mp_int_t nRepeatTransmit;

    mp_obj_t handler;

	rcswitch_Protocol_obj_t protocol;
} rcswitch_RCSwitch_obj_t;

// -----------------------------------------------------------------

static const rcswitch_Protocol_obj_t proto[] = {
  { 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1
  { 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
  { 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
  { 380, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
  { 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
  { 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 6 (HT6P20B)
  { 150, {  2, 62 }, {  1,  6 }, {  6,  1 }, false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
  { 200, {  3, 130}, {  7, 16 }, {  3,  16}, false},     // protocol 8 Conrad RS-200 RX
  { 200, { 130, 7 }, {  16, 7 }, { 16,  3 }, true},      // protocol 9 Conrad RS-200 TX
  { 365, { 18,  1 }, {  3,  1 }, {  1,  3 }, true },     // protocol 10 (1ByOne Doorbell)
  { 270, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 11 (HT12E)
  { 320, { 36,  1 }, {  1,  2 }, {  2,  1 }, true }      // protocol 12 (SM5212)
};

enum {
	 numProto = sizeof(proto) / sizeof(proto[0])
};

STATIC inline unsigned int diff(int A, int B) {
	return abs(A - B);
}

STATIC bool receiveProtocol(mp_obj_t self_in, const int p, unsigned int changeCount) {

    rcswitch_RCSwitch_obj_t *self = MP_OBJ_TO_PTR(self_in);

	const rcswitch_Protocol_obj_t pro = proto[p-1];

	unsigned long code = 0;
	//Assuming the longer pulse length is the pulse captured in timings[0]
	const unsigned int syncLengthInPulses =  ((pro.syncFactor.low) > (pro.syncFactor.high)) ? (pro.syncFactor.low) : (pro.syncFactor.high);
	const unsigned int delay = self->timings[0] / syncLengthInPulses;
	const unsigned int delayTolerance = delay * self->nReceiveTolerance / 100;

	const unsigned int firstDataTiming = (pro.invertedSignal) ? (2) : (1);

	for (unsigned int i = firstDataTiming; i < changeCount - 1; i += 2) {
		code <<= 1;
		if (diff(self->timings[i], delay * pro.zero.high) < delayTolerance &&
			diff(self->timings[i + 1], delay * pro.zero.low) < delayTolerance) {
			// zero
		} else if (diff(self->timings[i], delay * pro.one.high) < delayTolerance && diff(self->timings[i + 1], delay * pro.one.low) < delayTolerance) {
			// one
			code |= 1;
		} else {
			// Failed
			return false;
		}
	}

	if (changeCount > 7) {		// ignore very short transmissions: no device sends them, so this must be noise
		self->nReceivedValue = code;
		self->nReceivedBitlength = (changeCount - 1) / 2;
		self->nReceivedDelay = delay;
		self->nReceivedProtocol = p;
		return true;
	}

	return false;
}

STATIC void handleInterrupt(void *self_in)
{
	rcswitch_RCSwitch_obj_t *self = MP_OBJ_TO_PTR(self_in);

	static unsigned int changeCount = 0;
	static unsigned long lastTime = 0;
	static unsigned int repeatCount = 0;

	const long time = esp_timer_get_time();
	const unsigned int duration = time - lastTime;

	if (duration > self->nSeparationLimit) {
		if (diff(duration, self->timings[0]) < 200) {
			repeatCount++;
			if (repeatCount == 2) {
				for(uint8_t i = 1; i <= numProto; i++) {
					if (receiveProtocol(self, i, changeCount)) {
						// mp_printf(&mp_plat_print, "received succeeded for protocol %d\r\nCode: %d\r\n", i, self->nReceivedValue);
                        mp_sched_schedule(self->handler, MP_OBJ_NEW_SMALL_INT(self->nReceivedValue));
                        mp_hal_wake_main_task_from_isr();
						break;
					}
				}
				repeatCount = 0;
			}
		}
		changeCount = 0;
	}
	// detect overflow
	if (changeCount >= RCSWITCH_MAX_CHANGES) {
		changeCount = 0;
		repeatCount = 0;
	}

	self->timings[changeCount++] = duration;
	lastTime = time;
}

STATIC mp_obj_t rcswitch_RCSwitch_enableReceive(mp_obj_t self_in, mp_obj_t interrupt, mp_obj_t handler) {
    rcswitch_RCSwitch_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->nReceiverInterrupt = mp_obj_get_int(interrupt);
    self->handler = handler;

    uint64_t gpio_pin_sel = ( 1ULL << self->nReceiverInterrupt );
	ESP_LOGI(TAG, "ESP_LOGI: RCSwitch->nReceiverInterrupt=%d gpio_pin_sel=%llu", self->nReceiverInterrupt, gpio_pin_sel);
    // mp_printf(&mp_plat_print, "RCSwitch->nReceiverInterrupt=%d gpio_pin_sel=%llu", self->nReceiverInterrupt, gpio_pin_sel);
    gpio_isr_handler_remove(self->nReceiverInterrupt);
    gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_ANYEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = gpio_pin_sel,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE
	};
	gpio_config(&io_conf);
    //install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(self->nReceiverInterrupt, handleInterrupt, (void*)self);

    mp_printf(&mp_plat_print, "call enableReceiveInternal\r\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(rcswitch_RCSwitch_enableReceive_obj, rcswitch_RCSwitch_enableReceive);

STATIC mp_obj_t rcswitch_RCSwitch_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // Allocates the new object and sets the type.
    rcswitch_RCSwitch_obj_t *self = m_new_obj(rcswitch_RCSwitch_obj_t);
    self->base.type = (mp_obj_type_t *)type;

    self->nReceivedValue = 0;
	self->nReceivedBitlength = 0;
	self->nReceivedDelay = 0;
	self->nReceivedProtocol = 0;
	self->nReceiveTolerance = 60;
	self->nSeparationLimit = 4300;

	self->nReceiverInterrupt = -1;
	self->nReceivedValue = 0;

    mp_printf(&mp_plat_print, "RCSwitch init v0.2\r\n");

    // The make_new function always returns self.
    return MP_OBJ_FROM_PTR(self);
}

// This collects all methods and other static class attributes of the RCSwitch.
// The table structure is similar to the module table, as detailed below.
STATIC const mp_rom_map_elem_t rcswitch_RCSwitch_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_enableReceive), MP_ROM_PTR(&rcswitch_RCSwitch_enableReceive_obj) },
};
STATIC MP_DEFINE_CONST_DICT(rcswitch_RCSwitch_locals_dict, rcswitch_RCSwitch_locals_dict_table);

// This defines the type(RCSwitch) object.
MP_DEFINE_CONST_OBJ_TYPE(
    rcswitch_type_RCSwitch,
    MP_QSTR_RCSwitch,
    MP_TYPE_FLAG_NONE,
    make_new, rcswitch_RCSwitch_make_new,
    locals_dict, &rcswitch_RCSwitch_locals_dict
    );


STATIC const mp_rom_map_elem_t rcswitch_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_rcswitch) },
    // { MP_ROM_QSTR(MP_QSTR_setProtocol), MP_ROM_PTR(&rcswitch_setProtocol_obj) },
    { MP_ROM_QSTR(MP_QSTR_RCSwitch),    MP_ROM_PTR(&rcswitch_type_RCSwitch) },
};
STATIC MP_DEFINE_CONST_DICT(rcswitch_globals, rcswitch_globals_table);

// Define module object.
const mp_obj_module_t rcswitch_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&rcswitch_globals,
};

// Register the module to make it available in Python.
MP_REGISTER_MODULE(MP_QSTR_rcswitch, rcswitch_user_cmodule);