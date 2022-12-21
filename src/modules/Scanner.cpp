//----------------------------------------------------------------------------
//	EMC23 Tech tools Plugin for VCV Rack - 16 Step Glitch Sequencer Scanner
//  Copyright (C) 2022  EMC23
//----------------------------------------------------------------------------
#include "plugin.hpp"
#include "cmath"
//#include "dirent.h"
#include <algorithm> //----added by Joakim Lindbom
#include <osdialog.h>
#include <rtl-sdr.h>
//#include "libusb.h"
#include <stdio.h>
#include <limits.h>
#include <cstring>
#include <iostream>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#define HZ_CEIL 110.0
#define HZ_FLOOR 80.0
#define HZ_SPAN (HZ_CEIL-HZ_FLOOR)
#define HZ_CENTER (HZ_FLOOR + (0.5 * HZ_SPAN))
#define MAX_VOLTAGE 5.0
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <vector>
#include <complex>
#include <pthread.h>
#include <logger.hpp>

/*! External libraries */
static int n;
static rtlsdr_dev_t *dev; /*!< RTL-SDR device */
static int iterate = 0;
int MyFreq = 1;
int MyFreq2 = 1;
int r, opt;
int startUpCounter = 20;
static int _gain = 14; /*!< [ARG] Device gain (optional) */
static uint32_t _dev_id = 0;
static int	_offset_tuning = 1;
static long	_center_freq = 95105000;
#define NUM_READ 512
long currentFreq = 95105000;
static uint32_t applied_bw = 80000;
static uint32_t new_bw = applied_bw;
#define DEFAULT_BUF_LENGTH		(16 * 16384)
uint32_t out_block_size = DEFAULT_BUF_LENGTH;
static const int n_read = NUM_READ;

uint32_t longFreq = _center_freq;
void* do_freq(void*){
	rtlsdr_set_center_freq(dev, longFreq);
	return 0;
}
struct int_queue{
    float_t *arr;
    size_t size;
    int len;
    int first_elem;
};
struct int_queue queue;
void init_int_queue(struct int_queue *queue, size_t nelems)
{
    queue->arr = (float_t*)malloc(nelems * sizeof(float_t));
    queue->first_elem = 0;
    queue->len = 0;
    queue->size = nelems;
}
void destroy_int_queue(struct int_queue *queue)
{
	free(queue->arr);
}
void push_float(struct int_queue *queue, float new_val)
{
	queue->arr[(queue->first_elem + (queue->len)++) % queue->size] = new_val;
	if (queue->len > (int)queue->size){
		queue->len--;
		queue->first_elem++;
		queue->first_elem %= queue->size;
	}
}

float get_float(struct int_queue *queue, int index)
{
	// note does not handle the case for index out of bounds
	// wraps around for overflow
	return queue->arr[(queue->first_elem + index) % queue->size];
}




int my_reset(struct int_queue queue){
			destroy_int_queue(&queue);
			init_int_queue(&queue, NUM_READ);
			return 0;
		}
struct Scanner : Module {
	enum ParamIds {
		PITCH_PARAM,
		TUNE_ATT,
		QUANT_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		TUNE_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		SINE_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		BLINK_LIGHT,
		NUM_LIGHTS
	};
	float freq = (float) currentFreq;
	static void sig_handler(int signum){
		do_exit();
	}
	static int register_signals(){
		INFO("sux: #%d\n",  NUM_PARAMS   );
		signal(SIGINT, sig_handler);
		signal(SIGTERM, sig_handler);
		signal(SIGABRT, sig_handler);
		/**!
		* NOTE: Including the SIGPIPE signal might cause
		* problems with the pipe communication.
		* However, in tests we got any at problems at all.
		*/
		// sigaction(SIGPIPE, &sig_act, NULL);
		return 0;
	}
	static int configure_rtlsdr(){
	INFO("sux2: #%d\n",  NUM_PARAMS   );
		int device_count = rtlsdr_get_device_count();
		if (!device_count) {
			INFO("No supported devices found.\n");
		}
		for(int i = 0; i < device_count; i++){
			INFO("#%d: %s\n", n, rtlsdr_get_device_name(i));
		}
		int dev_open = rtlsdr_open(&dev, _dev_id);
		if (dev_open < 0) {
			INFO("Failed to open RTL-SDR device #%d\n", _dev_id);
		}else{
			INFO("Using device: #%d\n", dev_open);
		}
		/**!
		 * Set gain mode auto if '_gain' equals to 0.
		 * Otherwise, set gain mode to manual.
		 * (Mode 1 [manual] needs gain value so
		 * gain setter function must be called.)
		 */
		if(!_gain){
			rtlsdr_set_tuner_gain_mode(dev, _gain);
			INFO("Gain mode set to auto.\n");
		}else{
			rtlsdr_set_tuner_gain_mode(dev, 1);
			int gain_count = rtlsdr_get_tuner_gains(dev, NULL);
			int gains[gain_count], supported_gains = rtlsdr_get_tuner_gains(dev, gains);
			for (int i = 0; i < supported_gains; i++){
				/**!
				 * Different RTL-SDR devices have different supported gain
				 * values. So select gain value between 1.0 and 3.0
				 */
				if (gains[i] > 10 && gains[i] < 30)
					_gain = gains[i];
				INFO( "%.1f ", gains[i] / 10.0);
			}
			INFO( "\n");
			INFO("Gain set to %.1f\n", _gain / 10.0);
			rtlsdr_set_tuner_gain(dev, _gain);
 		}
		/**!
		 * Enable or disable offset tuning for zero-IF tuners, which allows to avoid
		 * problems caused by the DC offset of the ADCs and 1/f noise.
		 */

		rtlsdr_set_offset_tuning(dev, _offset_tuning);
		rtlsdr_set_center_freq(dev, _center_freq);
        rtlsdr_set_sample_rate(dev, (int) APP->engine->getSampleRate());
		int r = rtlsdr_reset_buffer(dev);
		if (r < 0){
			INFO("Failed to reset buffers.\n");
			return 1;
		}
		return 0;
	}
	static void do_exit(){
	rtlsdr_cancel_async(dev);
	exit(0);
	}
	static void my_callback(unsigned char *buf, uint32_t len, void *ctx){
	if (ctx) {
             for (uint32_t iter=0; iter < len; iter++){
		    	push_float(&queue, (uint8_t)buf[iter]);
		     }
			//	do_exit = 1;
            //	rtlsdr_cancel_async(dev);
		}
	}
	Scanner() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
     	configParam(PITCH_PARAM, HZ_FLOOR, HZ_CEIL, HZ_CENTER, "");
        configParam(TUNE_ATT, -HZ_SPAN/2.0, +HZ_SPAN/2.0, 0.0, "");
        configParam(QUANT_PARAM, 0.0, 2.0, 0.0, "");
		register_signals();
		configure_rtlsdr();
		init_int_queue(&queue, NUM_READ);
		read();
	}
	long getFreq(float knob) {
		return int(knob*1000000); // float quantities are in millions so this is a million
	}
	float getMegaFreq(long longFreq) {
		return (longFreq)/ 1000000; // float quantities are in millions so this is a million
	}
	//float freq=(float) _center_freq;
	void read() {
			int actual_length;
			uint8_t *data = (uint8_t *)(malloc(out_block_size));
			rtlsdr_read_sync(dev, data, NUM_READ , &actual_length);
			for (int iter=0; iter< NUM_READ; iter++){
				push_float(&queue, (float)data[iter]);
			}
			iterate=0;
			free(data);
			freq = params[PITCH_PARAM].getValue();
	        float freqOff = params[TUNE_ATT].getValue() * inputs[TUNE_INPUT].getVoltage()/MAX_VOLTAGE;
	       	freq = getFreq(freq + freqOff); // lots of zeros
		    //	enum Quantization {HUNDREDK, TENK, NONE};
			if (freq - currentFreq) {
				pthread_t t1;
				//do_freq2(longFreq);
				pthread_create(&t1, NULL, &do_freq, &freq);
				pthread_join(t1, NULL);
				currentFreq = freq;
			}
	}
	void process(const ProcessArgs &args) override {

		if (iterate == NUM_READ){
			read();
		// wait a number of cycles before we use the clock and run inputs to allow them propagate correctly after startup
		if (startUpCounter > 0)
			startUpCounter--;
	}
     //float value = MAX_VOLTAGE*float(sample)/(float)SHRT_MAX;


	//float value =((100 *  MAX_VOLTAGE * (get_int(&queue, iterate))/(float)SHRT_MAX)-1.9)* 100;


	float value = (MAX_VOLTAGE * 100 *  ( (get_float(&queue, iterate))/(float)SHRT_MAX )-1.9) * 50 ;

    // b float value = (MAX_VOLTAGE * 100 *  ( (get_float(&queue, iterate))/(float)SHRT_MAX )) * 50 ;

    // c float value = (100 *  ( (get_float(&queue, iterate))/(float)SHRT_MAX )) * 50 ;

   // d  float value = (100 *  ( (get_float(&queue, iterate))/(float)SHRT_MAX )) ;
   
    // e float value = get_float(&queue, iterate) ;

	// f float value = MAX_VOLTAGE*float(get_float(&queue, iterate))/(float)SHRT_MAX;

    // float value = MAX_VOLTAGE * ((get_float(&queue, iterate) - 128)/15) ;


	//outputs[SDR::AUDIO_OUT].value 	= value;
	outputs[SINE_OUTPUT].setVoltage(value);
	//	})
	iterate++;
	};
};
struct ScannerDisplay : TransparentWidget {
	std::shared_ptr<Font> font;
	NVGcolor txtCol;
	Scanner * module;
	const int fh = 18; // font height

	ScannerDisplay(Vec pos) {
		box.pos = pos;
		box.size.y = fh;
		box.size.x = fh * 4;
		setColor(0x00, 0x00, 0x00, 0xFF);
		font = APP->window->loadFont(asset::plugin(pluginInstance, "res/DejaVuSansMono.ttf"));
	}

	ScannerDisplay(Vec pos, unsigned char r, unsigned char g, unsigned char b, unsigned char a) {
		box.pos = pos;
		box.size.y = fh;
		box.size.x = fh * 4;
		setColor(r, g, b, a);
		font = APP->window->loadFont(asset::plugin(pluginInstance, "res/DejaVuSansMono.ttf"));
	}

	void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a) {
		txtCol.r = r;
		txtCol.g = g;
		txtCol.b = b;
		txtCol.a = a;
	}

	void draw(const DrawArgs &args) override {
		char tbuf2[8];
		if (module == NULL) return;
	//	snprintf(tbuf, sizeof(tbuf), "%d", module->stepNr+1);
		snprintf(tbuf2, sizeof(tbuf2), "%d", (int)module->freq);
		TransparentWidget::draw(args);
		drawBackground(args);
		drawValue(args, tbuf2);
	}

	void drawBackground(const DrawArgs &args) {
		Vec c = Vec(box.size.x/2, box.size.y);
		int whalf = box.size.x/2;
		int hfh = floor(fh / 2);

		// Draw rounded rectangle
		nvgFillColor(args.vg, nvgRGBA(0xff, 0xff, 0xff, 0xF0));
		{
			nvgBeginPath(args.vg);
			nvgMoveTo(args.vg, c.x -whalf, c.y +2);
			nvgLineTo(args.vg, c.x +whalf, c.y +2);
			nvgQuadTo(args.vg, c.x +whalf +5, c.y +2+hfh, c.x +whalf, c.y+fh+2);
			nvgLineTo(args.vg, c.x -whalf, c.y+fh+2);
			nvgQuadTo(args.vg, c.x -whalf -5, c.y +2+hfh, c.x -whalf, c.y +2);
			nvgClosePath(args.vg);
		}
		nvgFill(args.vg);
		nvgStrokeColor(args.vg, nvgRGBA(0x00, 0x00, 0x00, 0x0F));
		nvgStrokeWidth(args.vg, 1.f);
		nvgStroke(args.vg);
	}

	void drawValue(const DrawArgs &args, const char * txt) {
		Vec c = Vec(box.size.x/2, box.size.y);
		nvgFontSize(args.vg, fh);
		nvgFontFaceId(args.vg, font->handle);
		nvgTextLetterSpacing(args.vg, -2);
		nvgTextAlign(args.vg, NVG_ALIGN_CENTER);
		nvgFillColor(args.vg, nvgRGBA(txtCol.r, txtCol.g, txtCol.b, txtCol.a));
		nvgText(args.vg, c.x, c.y+fh-1, txt, NULL);
	}
};
struct ScannerWidget : ModuleWidget {

	ScannerWidget(Scanner* module) {

		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/MyModule.svg")));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 46.063)), module, Scanner::PITCH_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 66.063)), module, Scanner::TUNE_ATT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(15.24, 77.478)), module, Scanner::TUNE_INPUT));
		addParam(createParam<CKSSThree>(Vec(RACK_GRID_WIDTH/2, 240), module, Scanner::QUANT_PARAM));
    //  addParam(createParam<LightupButton>(Vec(15.24, 77.478 + 3), module, Scanner::BUTTON_PARAM));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.24, 100)), module, Scanner::SINE_OUTPUT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(15.24, 25.81)), module, Scanner::BLINK_LIGHT));
        {
			ScannerDisplay * sd2 = new ScannerDisplay(Vec(11,20));
			sd2->module = module;
			addChild(sd2);
		}
	}
};
Model* modelScanner = createModel<Scanner, ScannerWidget>("Scanner");