/*
 * Copyright 2020 Poly Effects <info@polyeffects.com>
 * based on midi gate example, 
  Copyright 2013-2016 David Robillard <d@drobilla.net>
  used DLL from JACK MIDI beat clock parser
 * (C) 2013  Robin Gareus <robin@gareus.org>

  Permission to use, copy, modify, and/or distribute this software for any
  purpose with or without fee is hereby granted, provided that the above
  copyright notice and this permission notice appear in all copies.

  THIS SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include "lv2/atom/atom.h"
#include "lv2/atom/util.h"
#include "lv2/core/lv2.h"
#include "lv2/core/lv2_util.h"
#include "lv2/log/log.h"
#include "lv2/log/logger.h"
#include "lv2/midi/midi.h"
#include "lv2/urid/urid.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MCLK_IN_URI "http://polyeffects.com/lv2/mclk_in"

#define TRIGGER_LENGTH 48 // 1 ms at 48kHz

typedef enum {
	MCLK_IN_CONTROL = 0,
	MCLK_IN_BPM     = 1,
	MCLK_IN_IS_RUNNING = 2,
	MCLK_IN_START_TRIGGER = 3,
	MCLK_IN_BANDWIDTH_PARAM = 4,
	MCLK_IN_DIVIDER_PARAM = 5,
	MCLK_IN_PULSE_OUT = 6,
} PortIndex;

static const double dll_bandwidth = 6.0; // 1/Hz
static const int  samplerate = 48000; // shouldn't fix this

typedef struct {
  double t0; ///< time of the current Mclk tick
  double t1; ///< expected next Mclk tick
  double e2; ///< second order loop error
  double b, c, omega; ///< DLL filter coefficients
} DelayLockedLoop;

typedef struct {
	// Port buffers
	const LV2_Atom_Sequence* control;
	float*                   bpm;
	float*                   is_running;
	float*                   start_trigger;
	const float*             bandwidth_param;
	const float*             divider;
	float*                   pulse_out;

	uint32_t previous_time;
	uint32_t sequence;
	float previous_bpm;
	bool dll_started;
	bool run_state;
	int trigger_countdown;
	int pulse_countdown;
	int pulse_count;
	double current_dll_bandwidth;

	DelayLockedLoop dll;

	// Features
	LV2_URID_Map*  map;
	LV2_Log_Logger logger;

	struct {
		LV2_URID midi_MidiEvent;
	} uris;

} Mclk_in;

static LV2_Handle
instantiate(const LV2_Descriptor*     descriptor,
            double                    rate,
            const char*               bundle_path,
            const LV2_Feature* const* features)
{
	Mclk_in* self = (Mclk_in*)calloc(1, sizeof(Mclk_in));
	if (!self) {
		return NULL;
	}
	self->previous_time = 0;
	self->sequence = 0;
	self->trigger_countdown = 0;
	self->pulse_countdown = 0;
	self->pulse_count = 0;
	self->previous_bpm = 120.0;
	self->dll_started = false;
	self->run_state = false;
	self->current_dll_bandwidth = dll_bandwidth;

	// Scan host features for URID map
	const char* missing = lv2_features_query(
		features,
		LV2_LOG__log,  &self->logger.log, false,
		LV2_URID__map, &self->map,        true,
		NULL);
	lv2_log_logger_set_map(&self->logger, self->map);
	if (missing) {
		lv2_log_error(&self->logger, "Missing feature <%s>\n", missing);
		free(self);
		return NULL;
	}

	self->uris.midi_MidiEvent = self->map->map(
		self->map->handle, LV2_MIDI__MidiEvent);

	return (LV2_Handle)self;
}

static void
connect_port(LV2_Handle instance,
             uint32_t   port,
             void*      data)
{
	Mclk_in* self = (Mclk_in*)instance;

	switch ((PortIndex)port) {
	case MCLK_IN_CONTROL:
		self->control = (const LV2_Atom_Sequence*)data;
		break;
	case MCLK_IN_BPM:
		self->bpm = (float*)data;
		break;
	case MCLK_IN_IS_RUNNING:
		self->is_running = (float*)data;
		break;
	case MCLK_IN_START_TRIGGER:
		self->start_trigger = (float*)data;
		break;
	case MCLK_IN_BANDWIDTH_PARAM:
		self->bandwidth_param = (const float*)data;
		break;
	case MCLK_IN_DIVIDER_PARAM:
		self->divider = (const float*)data;
		break;
	case MCLK_IN_PULSE_OUT:
		self->pulse_out = (float*)data;
		break;
	}
}

static void
activate(LV2_Handle instance)
{
	Mclk_in* self = (Mclk_in*)instance;
	/* self->n_active_notes = 0; */
}


/**
 * initialize DLL
 * set current time and period in samples
 */
static void init_dll(DelayLockedLoop *dll, double tme, double period, double bandwidth) {
  const double omega = 2.0 * M_PI * period / bandwidth / samplerate;
  dll->b = 1.4142135623730950488 * omega;
  dll->c = omega * omega;

  dll->e2 = period / samplerate;
  dll->t0 = tme / samplerate;
  dll->t1 = dll->t0 + dll->e2;
}

/**
 * run one loop iteration.
 * @param tme time of event (in samples)
 * @return smoothed interval (period) [1/Hz]
 */
static double run_dll(DelayLockedLoop *dll, double tme) {
  const double e = tme / samplerate - dll->t1;
  dll->t0 = dll->t1;
  dll->t1 += dll->b * e + dll->e2;
  dll->e2 += dll->c * e;
  return (dll->t1 - dll->t0);
}

static void
run(LV2_Handle instance, uint32_t sample_count)
{
	Mclk_in* self   = (Mclk_in*)instance;
	uint32_t  offset = 0;
	float* const bpm = self->bpm;
	float* const is_running = self->is_running;
	float* const start_trigger = self->start_trigger;
	float* const pulse_out = self->pulse_out;
	const float bandwidth_param = *(self->bandwidth_param);
	const float divider = *(self->divider);

	int trigger_countdown = self->trigger_countdown;
	int pulse_countdown = self->pulse_countdown;
	int pulse_count = self->pulse_count;
	bool run_state = self->run_state;
	
	if (self->current_dll_bandwidth != bandwidth_param){
		self->current_dll_bandwidth = bandwidth_param;
		self->dll_started = false;
		self->previous_time = 0;
	}


	LV2_ATOM_SEQUENCE_FOREACH(self->control, ev) {
		if (ev->body.type == self->uris.midi_MidiEvent) {
			const uint8_t* const msg = (const uint8_t*)(ev + 1);
			switch (lv2_midi_message_type(msg)) {
			case LV2_MIDI_MSG_START:
				trigger_countdown = TRIGGER_LENGTH;
				run_state = true;
				// if current BPM to slow or too fast, restart DLL
				if (self->previous_bpm < 15 || self->previous_bpm > 300){
					self->dll_started = false;
					self->previous_time = 0;
				}
				break;
			case LV2_MIDI_MSG_STOP:
				trigger_countdown = 0;
				run_state = false;
				break;
			case LV2_MIDI_MSG_CONTINUE:
				trigger_countdown = TRIGGER_LENGTH;
				run_state = true;
				break;
			case LV2_MIDI_MSG_CLOCK:
				// if this is the first message, set initial time
				// second message init_dll
				//
				//
				offset = ev->time.frames + self->sequence; 
				pulse_count++;
				if (pulse_count >= divider){
					pulse_count = 0;
					pulse_out[ev->time.frames] = 1.0f;
				};
				if (self->previous_time == 0){
					self->previous_time = offset; 
				}
				else if (!self->dll_started) {
					/* 2nd event in sequence -> initialize DLL with time difference */
					init_dll(&self->dll, offset, (offset - self->previous_time), self->current_dll_bandwidth);
					self->previous_bpm = samplerate * 60.0 / (24.0 * (double)(offset - self->previous_time));
					self->dll_started = true;
					/* fprintf (stderr, "initial set %f", self->previous_bpm); */
				}
				// otherwise calculate BPM
				else {
					/* run dll, calculate filtered bpm */
					self->previous_bpm = 60.0 / (24.0 * run_dll(&self->dll, offset));
					/* fprintf (stderr, "later set %f frames %u", self->previous_bpm, offset); */
				}
				break;
			default: break;
			}
		}

	}
	self->sequence = self->sequence + sample_count;
	bpm[0] = self->previous_bpm;
	for (uint32_t s = 0; s < sample_count; ++s) {
		if (trigger_countdown > 0){
			trigger_countdown--;
			start_trigger[s] = 1.0f;
		}
		else {
			start_trigger[s] = 0.0f;
		}
		if (pulse_out[s] > 0.9f){// we've set it to on
			pulse_countdown = TRIGGER_LENGTH;
		} else if (pulse_countdown > 0){
			pulse_countdown--;
			pulse_out[s] = 1.0f;
		}
		else {
			pulse_out[s] = 0.0f;
		}
		is_running[s] = (float) run_state;
	}

	self->trigger_countdown = trigger_countdown;
	self->pulse_countdown = pulse_countdown;
	self->pulse_count = pulse_count;
	self->run_state = run_state;
	
}

/**
   We have no resources to free on deactivation.
   Note that the next call to activate will re-initialise the state, namely
   self->n_active_notes, so there is no need to do so here.
*/
static void
deactivate(LV2_Handle instance)
{}

static void
cleanup(LV2_Handle instance)
{
	free(instance);
}

/**
   This plugin also has no extension data to return.
*/
static const void*
extension_data(const char* uri)
{
	return NULL;
}

static const LV2_Descriptor descriptor = {
	MCLK_IN_URI,
	instantiate,
	connect_port,
	activate,
	run,
	deactivate,
	cleanup,
	extension_data
};

LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
	switch (index) {
	case 0:
		return &descriptor;
	default:
		return NULL;
	}
}
