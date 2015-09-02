#include "stdafx.h"
#include "resource.h"
#include "hackrf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <mutex> 


#define BUF_LEN 262144         //hackrf tx buf
#define BUF_NUM  31
#define BYTES_PER_SAMPLE  2
#define M_PI 3.14159265358979323846


DECLARE_COMPONENT_VERSION(
"HackRF Transmitter",
"0.0.7",
"Source Code:https://github.com/jocover/foo_hackrf \n"
"DLL:https://github.com/jocover/foo_hackrf/blob/master/Release/foo_hackrf.dll \n");


// This will prevent users from renaming your component around (important for proper troubleshooter behaviors) or loading multiple instances of it.
VALIDATE_COMPONENT_FILENAME("foo_hackrf.dll");


struct config {
	double freq;
	uint32_t gain;
	uint32_t mode;
	uint32_t tx_vga;
	uint32_t enableamp;

};
static config default{
	433.00,//433Mhz
		90,
		0,//mode WBFM=0 NBFM=1 AM=2
		40,
		1};

static bool running = false;

static void RunDSPConfigPopup(const dsp_preset & p_data, HWND p_parent, dsp_preset_edit_callback & p_callback);

int _hackrf_tx_callback(hackrf_transfer *transfer);

class dsp_sample : public dsp_impl_base
{
public:

	int hackrf_tx_callback(int8_t *buffer, uint32_t length) {

		m_mutex.lock();

		if (count == 0) {
			memset(buffer, 0, length);
		}
		else {
			memcpy(buffer, _buf[tail], length);
			tail = (tail + 1) % BUF_NUM;
			count--;
		}
		m_mutex.unlock();

		return 0;
	}

	void interpolation(float * in_buf, uint32_t in_samples, float * out_buf, uint32_t out_samples, float last_in_samples[4]) {
		uint32_t i;		/* Input buffer index + 1. */
		uint32_t j = 0;	/* Output buffer index. */
		float pos;		/* Position relative to the input buffer
						* + 1.0. */

						/* We always "stay one sample behind", so what would be our first sample
						* should be the last one wrote by the previous call. */
		pos = (float)in_samples / (float)out_samples;
		while (pos < 1.0)
		{
			out_buf[j] = last_in_samples[3] + (in_buf[0] - last_in_samples[3]) * pos;
			j++;
			pos = (float)(j + 1)* (float)in_samples / (float)out_samples;
		}

		/* Interpolation cycle. */
		i = (uint32_t)pos;
		while (j < (out_samples - 1))
		{

			out_buf[j] = in_buf[i - 1] + (in_buf[i] - in_buf[i - 1]) * (pos - (float)i);
			j++;
			pos = (float)(j + 1)* (float)in_samples / (float)out_samples;
			i = (uint32_t)pos;
		}

		/* The last sample is always the same in input and output buffers. */
		out_buf[j] = in_buf[in_samples - 1];

		/* Copy last samples to last_in_samples (reusing i and j). */
		for (i = in_samples - 4, j = 0; j < 4; i++, j++)
			last_in_samples[j] = in_buf[i];
	}

	void modulation(float * input, float * output, uint32_t mode) {

		if (mode == 0) {
			fm_deviation = 2.0 * M_PI * 75.0e3 / hackrf_sample; // 75 kHz max deviation WBFM
		}
		else if (mode == 1)
		{
			fm_deviation = 2.0 * M_PI * 5.0e3 / hackrf_sample; // 5 kHz max deviation NBFM
		}

		//AM mode
		if (mode == 2) {
			for (uint32_t i = 0; i < BUF_LEN; i++) {
				double	audio_amp = input[i] * gain;

				if (fabs(audio_amp) > 1.0) {
					audio_amp = (audio_amp > 0.0) ? 1.0 : -1.0;
				}

				IQ_buf[i * BYTES_PER_SAMPLE] = (float)audio_amp;
				IQ_buf[i * BYTES_PER_SAMPLE + 1] = 0;
			}
		}
		//FM mode
		else {

			for (uint32_t i = 0; i < BUF_LEN; i++) {

				double	audio_amp = input[i] * gain;

				if (fabs(audio_amp) > 1.0) {
					audio_amp = (audio_amp > 0.0) ? 1.0 : -1.0;
				}
				fm_phase += fm_deviation * audio_amp;
				while (fm_phase > (float)(M_PI))
					fm_phase -= (float)(2.0 * M_PI);
				while (fm_phase < (float)(-M_PI))
					fm_phase += (float)(2.0 * M_PI);

				output[i * BYTES_PER_SAMPLE] = (float)sin(fm_phase);
				output[i * BYTES_PER_SAMPLE + 1] = (float)cos(fm_phase);
			}
		}


	}

	void work(float *input_items, uint32_t len) {

		m_mutex.lock();
		int8_t * buf = (int8_t *)_buf[head];
		for (uint32_t i = 0; i < BUF_LEN; i++) {
			buf[i] = (int8_t)(input_items[i] * 127.0);
		}
		head = (head + 1) % BUF_NUM;
		count++;
		m_mutex.unlock();

	}

	dsp_sample(dsp_preset const & in) : conf(default) {
		parse_preset(conf, in);
		freq = uint64_t(conf.freq * 1000000);
		gain = (float)(conf.gain / 100.0);
		mode = conf.mode;
		tx_vga = conf.tx_vga;
		enableamp = conf.enableamp;



		count = tail = head = 0;

		_buf = (int8_t **)malloc(BUF_NUM * sizeof(int8_t *));
		if (_buf) {
			for (unsigned int i = 0; i < BUF_NUM; ++i) {
				_buf[i] = (int8_t *)malloc(BUF_LEN*sizeof(int8_t));
			}
		}
		// hackrf init  //

		hackrf_init();
		ret = hackrf_open(&_dev);
		if (ret != HACKRF_SUCCESS) {
			MessageBox(NULL, L"Failed to open HackRF device", NULL, MB_OK);
			hackrf_close(_dev);
		}
		else {
			hackrf_set_sample_rate(_dev, hackrf_sample);
			hackrf_set_baseband_filter_bandwidth(_dev, 1750000);
			hackrf_set_freq(_dev, freq);
			hackrf_set_txvga_gain(_dev, tx_vga);
			hackrf_set_amp_enable(_dev, enableamp);
			ret = hackrf_start_tx(_dev, _hackrf_tx_callback, (void *)this);
			if (ret != HACKRF_SUCCESS) {
				MessageBox(NULL, L"Failed to start TX streaming", NULL, MB_OK);
				hackrf_close(_dev);
			}
			running = true;
		}
	}

	~dsp_sample() {
		if (_dev) {
			hackrf_stop_tx(_dev);
			hackrf_close(_dev);
			_dev = NULL;
		}
		if (_buf) {
			for (unsigned int i = 0; i < BUF_NUM; ++i) {
				if (_buf[i])
					free(_buf[i]);
			}
			free(_buf);
		}

		delete IQ_buf;
		delete new_audio_buf;
		delete audio_buf;
		running = false;
	}

	static GUID g_get_guid() {
		//This is our GUID. Generate your own one when reusing this code.
		// {67BB9226-3830-4739-BD72-3951BF207388}
		static const GUID guid = { 0x67bb9226, 0x3830, 0x4739,{ 0xbd, 0x72, 0x39, 0x51, 0xbf, 0x20, 0x73, 0x88 } };

		return guid;
	}

	static void g_get_name(pfc::string_base & p_out) { p_out = "HackRF Transmitter"; }

	bool on_chunk(audio_chunk * chunk, abort_callback &) {
		// Perform any operations on the chunk here.
		// The most simple DSPs can just alter the chunk in-place here and skip the following functions.
		nch = chunk->get_channels();
		ch_mask = chunk->get_channel_config();
		m_sample_rate = chunk->get_sample_rate();
		m_sample_count = chunk->get_sample_count();
		hackrf_sample = m_sample_rate*1.0 / m_sample_count*BUF_LEN;

		//	uint32_t out_count = (uint32_t)m_sample_count*HACKRF_SAMPLE / m_sample_rate;
		audio_sample * source_audio_buf = chunk->get_data();


		if (audio_buf == NULL || new_audio_buf == NULL || IQ_buf == NULL) {
			hackrf_set_sample_rate(_dev, hackrf_sample);
			audio_buf = new float[m_sample_count]();
			new_audio_buf = new float[BUF_LEN]();
			IQ_buf = new float[BUF_LEN * 2]();
		}


		if (nch == 1 && ch_mask == audio_chunk::channel_config_mono) {
			for (uint32_t i = 0; i < m_sample_count; i++) {

				audio_buf[i] = source_audio_buf[i];
			}
		}
		else if (nch == 2 && ch_mask == audio_chunk::channel_config_stereo) {
			for (uint32_t i = 0; i < m_sample_count; i++) {

				audio_buf[i] = (source_audio_buf[i * 2] + source_audio_buf[i * 2 + 1]) / (float)2.0;
			}

		}

		if (running) {

			interpolation(audio_buf, m_sample_count, new_audio_buf, BUF_LEN, last_in_samples);

			modulation(new_audio_buf, IQ_buf, mode);

			for (uint32_t i = 0; i < (BUF_LEN * BYTES_PER_SAMPLE); i += BUF_LEN) {

				work(IQ_buf + i, BUF_LEN);
			}

			//AM mode


		}
		// To retrieve the currently processed track, use get_cur_file().
		// Warning: the track is not always known - it's up to the calling component to provide this data and in some situations we'll be working with data that doesn't originate from an audio file.
		// If you rely on get_cur_file(), you should change need_track_change_mark() to return true to get accurate information when advancing between tracks.

		return true; //Return true to keep the chunk or false to drop it from the chain.
	}

	void on_endofplayback(abort_callback &) {
		// The end of playlist has been reached, we've already received the last decoded audio chunk.
		// We need to finish any pending processing and output any buffered data through insert_chunk().
	}
	void on_endoftrack(abort_callback &) {
		// Should do nothing except for special cases where your DSP performs special operations when changing tracks.
		// If this function does anything, you must change need_track_change_mark() to return true.
		// If you have pending audio data that you wish to output, you can use insert_chunk() to do so.		
	}

	void flush() {
		// If you have any audio data buffered, you should drop it immediately and reset the DSP to a freshly initialized state.
		// Called after a seek etc.
	}

	double get_latency() {
		// If the DSP buffers some amount of audio data, it should return the duration of buffered data (in seconds) here.
		return 0;
	}

	bool need_track_change_mark() {
		// Return true if you need on_endoftrack() or need to accurately know which track we're currently processing
		// WARNING: If you return true, the DSP manager will fire on_endofplayback() at DSPs that are before us in the chain on track change to ensure that we get an accurate mark, so use it only when needed.
		return false;
	}
	static bool g_get_default_preset(dsp_preset & p_out) {
		make_preset(default, p_out);
		return true;
	}
	static void g_show_config_popup(const dsp_preset & p_data, HWND p_parent, dsp_preset_edit_callback & p_callback) {
		if (running) {
			MessageBox(NULL, L"Must Stop Playing", NULL, MB_OK);
		}
		else {
			::RunDSPConfigPopup(p_data, p_parent, p_callback);
		}
	}

	static bool g_have_config_popup() { return true; }
	static void make_preset(config conf, dsp_preset & out) {
		dsp_preset_builder builder;
		builder << conf.freq << conf.gain << conf.mode << conf.tx_vga << conf.enableamp;
		builder.finish(g_get_guid(), out);
	}


	static void parse_preset(config & conf, const dsp_preset & in) {

		try {
			dsp_preset_parser parser(in);
			parser >> conf.freq >> conf.gain >> conf.mode >> conf.tx_vga >> conf.enableamp;
		}
		catch (exception_io_data) { conf = default; }
	}

private:

	std::mutex m_mutex;
	hackrf_device * _dev;
	int ret;
	uint64_t freq;
	float gain;
	uint32_t mode;
	uint32_t tx_vga;
	uint8_t enableamp;

	uint32_t m_sample_rate;
	size_t m_sample_count;
	uint32_t nch;
	uint32_t ch_mask;

	config conf;
	int8_t ** _buf;
	int count;
	int tail;
	int head;
	uint32_t hackrf_sample = 2000000;
	float * audio_buf = NULL;
	float * IQ_buf = NULL;
	float * new_audio_buf = NULL;
	double fm_phase = NULL;
	double fm_deviation = NULL;
	float last_in_samples[4] = { 0.0, 0.0, 0.0, 0.0 };
};

int _hackrf_tx_callback(hackrf_transfer *transfer) {
	dsp_sample *obj = (dsp_sample *)transfer->tx_ctx;
	return obj->hackrf_tx_callback((int8_t *)transfer->buffer, transfer->valid_length);
}

// Use dsp_factory_nopreset_t<> instead of dsp_factory_t<> if your DSP does not provide preset/configuration functionality.
static dsp_factory_t<dsp_sample> g_dsp_sample_factory;

class CMyDSPPopup : public CDialogImpl<CMyDSPPopup> {
public:
	CMyDSPPopup(const dsp_preset & initData, dsp_preset_edit_callback & callback) : m_initData(initData), m_callback(callback) {}

	enum { IDD = IDD_DSP };
	enum {
		GainMin = 0,
		GainMax = 100,
		GainTotal = GainMax - GainMin,
		TXGainMin = 0,
		TXGainMax = 47,
		TxGainTotal = TXGainMax - TXGainMin
	};

	BEGIN_MSG_MAP(CMyDSPPopup)
		MSG_WM_INITDIALOG(OnInitDialog)
		MSG_WM_HSCROLL(OnHScroll)
		MSG_WM_COMMAND(OnCommand)
	END_MSG_MAP()

private:

	BOOL OnInitDialog(CWindow, LPARAM) {
		config _config;
		m_slider = GetDlgItem(IDC_SLIDER);
		m_slider_tx = GetDlgItem(IDC_SLIDER_TX);
		m_edit_freq = GetDlgItem(IDC_EDIT_FREQ);
		m_check_amp = GetDlgItem(IDC_CHECK_AMP);
		m_combo_mode = GetDlgItem(IDC_COMBO_MODE);
		m_slider.SetRange(0, GainTotal);
		m_slider_tx.SetRange(0, TxGainTotal);

		{
			dsp_sample::parse_preset(_config, m_initData);
			m_slider.SetPos(pfc::clip_t<t_int32>(pfc::rint32(_config.gain), GainMin, GainMax) - GainMin);
			m_slider_tx.SetPos(pfc::clip_t<t_int32>(pfc::rint32(_config.tx_vga), TXGainMin, TXGainMax) - TXGainMin);

			m_edit_freq.SetLimitText(7);
			char freq[20];
			sprintf_s(freq, "%.2f", _config.freq);
			pfc::string_formatter msg; msg << freq;
			::uSetDlgItemText(*this, IDC_EDIT_FREQ, msg);

			m_check_amp.SetCheck(_config.enableamp);

			m_combo_mode.AddString(L"WBFM");
			m_combo_mode.AddString(L"NBFM");
			m_combo_mode.AddString(L"AM");
			m_combo_mode.SetCurSel(_config.mode);

			RefreshLabel((uint32_t)_config.gain);
			RefreshTXLabel(_config.tx_vga);
			conf = _config;
		}
		return TRUE;
	}

	void OnCommand(UINT uNotifyCode, int nID, CWindow wndCtl) {

		switch (nID) {
		case IDOK: {
			pfc::string freqstr = ::uGetDlgItemText(*this, IDC_EDIT_FREQ);
			conf.freq = atof(freqstr.c_str());
			conf.mode = m_combo_mode.GetCurSel();
			conf.enableamp = m_check_amp.GetCheck();
			conf.gain = (uint32_t)m_slider.GetPos();
			conf.tx_vga = (uint32_t)m_slider_tx.GetPos();
			{
				dsp_preset_impl preset;
				dsp_sample::make_preset(conf, preset);
				m_callback.on_preset_changed(preset);
			}
			EndDialog(IDOK);
		}break;
		case IDCANCEL:
		{
			// Data not changed
			EndDialog(0);
		}
		break;
		}
	}


	void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar pScrollBar) {

		conf.gain = (uint32_t)m_slider.GetPos();
		conf.tx_vga = (uint32_t)m_slider_tx.GetPos();

		RefreshLabel((uint32_t)conf.gain);
		RefreshTXLabel(conf.tx_vga);
	}


	void RefreshTXLabel(uint32_t val) {
		pfc::string_formatter msg; msg << pfc::format_int(val);
		::uSetDlgItemText(*this, IDC_SLIDER_TX_LABEL, msg);
	}

	void RefreshLabel(uint32_t val) {
		pfc::string_formatter msg; msg << pfc::format_int(val);
		::uSetDlgItemText(*this, IDC_SLIDER_LABEL, msg);
	}

	const dsp_preset & m_initData; // modal dialog so we can reference this caller-owned object.
	dsp_preset_edit_callback & m_callback;

	CEdit m_edit_freq;
	CTrackBarCtrl m_slider;
	CTrackBarCtrl m_slider_tx;
	CCheckBox m_check_amp;
	CComboBox m_combo_mode;
	config conf;
};

static void RunDSPConfigPopup(const dsp_preset & p_data, HWND p_parent, dsp_preset_edit_callback & p_callback) {
	CMyDSPPopup popup(p_data, p_callback);
	if (popup.DoModal(p_parent) != IDOK) {
		// If the dialog exited with something else than IDOK,k 
		// tell host that the editing has been cancelled by sending it old preset data that we got initialized with
		p_callback.on_preset_changed(p_data);
	}
}