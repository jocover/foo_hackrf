#include "stdafx.h"
#include "resource.h"
#include "hackrf.h"

#include "soxr.h" //http://sourceforge.net/projects/soxr/
#include <stdlib.h>
#include <math.h>
#include <complex>
#include <stdio.h>

#define BUF_LEN 262144         //hackrf tx buf
#define BUF_NUM   15
#define BUFFER_SIZE 2048
#define BYTES_PER_SAMPLE  2
#define M_PI 3.14159265358979323846
#define HACKRF_SAMPLE 400000  //0.4Mhz
#define AUDIO_OUT_BUF_LEN (HACKRF_SAMPLE/8000*BUFFER_SIZE)   //PCM minimum 8000Hz



DECLARE_COMPONENT_VERSION(
"HackRF Transmitter", 
"0.0.1 alpha 1", 
"SourceCode:https://github.com/jocover/foo_hackrf \n"
"DLL:https://github.com/jocover/foo_hackrf/blob/master/Release/foo_hackrf.dll \n");


// This will prevent users from renaming your component around (important for proper troubleshooter behaviors) or loading multiple instances of it.
VALIDATE_COMPONENT_FILENAME("foo_hackrf.dll");


struct config {
	double freq;
	double fm_gain;
	uint32_t mode;
	uint32_t tx_vga;
	uint32_t enableamp;

};
static config default{
	433.00,//433Mhz
		90.0,
		0,//WBFM mode
		40,
		1};

static bool running = false;

static void RunDSPConfigPopup(const dsp_preset & p_data, HWND p_parent, dsp_preset_edit_callback & p_callback);

int _hackrf_tx_callback(hackrf_transfer *transfer);

class dsp_sample : public dsp_impl_base
{
public:

	int hackrf_tx_callback(int8_t *buffer, uint32_t length) {

		WaitForSingleObject(m_hMutex, INFINITE);
		if (count == 0) {
			memset(buffer, 0, length);
		}
		else {
			memcpy(buffer, _buf[tail], length);
			tail = (tail + 1) % BUF_NUM;
			count--;
		}
		ReleaseMutex(m_hMutex);
		return 0;
	}



	void work(float *input_items, uint32_t len) {
		WaitForSingleObject(m_hMutex, INFINITE);
		int8_t * buf = (int8_t *)_buf[head] + offset;
		if (len < samp_avail) {
			for (uint32_t i = 0; i < len; i++) {
				buf[i] = (int8_t)(input_items[i] * 128.0);
			}
			offset += len;
			samp_avail -= len;
		}
		else {
			for (uint32_t i = 0; i < samp_avail; i++) {
				buf[i] = (int8_t)(input_items[i] * 128.0);
			}
			head = (head + 1) % BUF_NUM;
			count++;

			buf = (int8_t*)_buf[head];
			int remaining = len - samp_avail;
			for (int32_t i = 0; i < remaining; i++) {
				buf[i] = (int8_t)(input_items[i] * 128.0);
			}
			offset = remaining;
			samp_avail = BUF_LEN - remaining;
		}
		ReleaseMutex(m_hMutex);
	}

	dsp_sample(dsp_preset const & in) : conf(default) {
		parse_preset(conf, in);
		freq = uint64_t(conf.freq * 1000000);
		fm_gain = (float)conf.fm_gain;
		mode = conf.mode;
		tx_vga = conf.tx_vga;
		enableamp = conf.enableamp;

		m_hMutex = CreateMutex(NULL, NULL, NULL);
		audio_buf = new float[BUFFER_SIZE];
		new_audio_sample = new float[AUDIO_OUT_BUF_LEN];
		audio_IQ_buf = new float[AUDIO_OUT_BUF_LEN * 2];
		if (mode == 0) {
			fm_deviation = 2.0 * M_PI * 75.0e3 / HACKRF_SAMPLE; // 75 kHz max deviation WBFM
		}
		else if (mode == 1)
		{
			fm_deviation = 2.0 * M_PI * 5.0e3 / HACKRF_SAMPLE; // 5 kHz max deviation NBFM
		}

		count = tail = head = offset = 0;
		samp_avail = BUF_LEN;
		_buf = (int8_t **)malloc(BUF_NUM * sizeof(int8_t *));
		if (_buf) {
			for (unsigned int i = 0; i < BUF_NUM; ++i) {
				_buf[i] = (int8_t *)malloc(BUF_LEN*sizeof(int8_t));
			}
		}


		// hackrf init TODO RunDSPConfigPopup crash //

		hackrf_init();
		ret = hackrf_open(&_dev);
		if (ret != HACKRF_SUCCESS) {
			MessageBox(NULL, L"Failed to open HackRF device", NULL, MB_OK);
			hackrf_close(_dev);
		}
		else {
			hackrf_set_sample_rate(_dev, HACKRF_SAMPLE);
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

		delete audio_IQ_buf;
		delete new_audio_sample;
		delete audio_buf;
		running = false;
	}

	static GUID g_get_guid() {
		//This is our GUID. Generate your own one when reusing this code.
		// {67BB9226-3830-4739-BD72-3951BF207388}
		static const GUID guid = { 0x67bb9226, 0x3830, 0x4739,{ 0xbd, 0x72, 0x39, 0x51, 0xbf, 0x20, 0x73, 0x88 } };

		return guid;
	}

	static void g_get_name(pfc::string_base & p_out) { p_out = "HackRF FM Transmitter"; }

	bool on_chunk(audio_chunk * chunk, abort_callback &) {
		// Perform any operations on the chunk here.
		// The most simple DSPs can just alter the chunk in-place here and skip the following functions.
		nch = chunk->get_channels();
		ch_mask = chunk->get_channel_config();
		m_sample_rate = chunk->get_sample_rate();
		m_sample_count = chunk->get_sample_count();
		uint32_t out_count = (uint32_t)m_sample_count*HACKRF_SAMPLE / m_sample_rate;
		audio_sample * source_audio_buf = chunk->get_data();

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

		//				if (debug) {
		//					str.Format(_T("in %d out:%d"), m_sample_count, out_count);
		//					MessageBox(NULL, str, L"Debug", MB_OK);
		//					debug = false;
		//				}

	//		Resample to 200000Mhz
		soxr_oneshot(m_sample_rate, HACKRF_SAMPLE, 1,
			audio_buf, m_sample_count, NULL,
			new_audio_sample, out_count, NULL,
			NULL, NULL, NULL);

		for (uint32_t i = 0; i < out_count; i++) {

			double	audio_amp = new_audio_sample[i] * (fm_gain / 100.0);

			if (fabs(audio_amp) > 1.0) {
				audio_amp = (audio_amp > 0.0) ? 1.0 : -1.0;
			}
			fm_phase += fm_deviation * audio_amp;
			while (fm_phase > (float)(M_PI))
				fm_phase -= (float)(2.0 * M_PI);
			while (fm_phase < (float)(-M_PI))
				fm_phase += (float)(2.0 * M_PI);
			audio_IQ_buf[i * BYTES_PER_SAMPLE] = (float)sin(fm_phase);
			audio_IQ_buf[i * BYTES_PER_SAMPLE + 1] = (float)cos(fm_phase);
		}

		work(audio_IQ_buf, out_count * BYTES_PER_SAMPLE);


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
		if (running) { MessageBox(NULL, L"Must Stop Playing", NULL, MB_OK); }
		else {
			::RunDSPConfigPopup(p_data, p_parent, p_callback);
		}
	}

	static bool g_have_config_popup() { return true; }
	static void make_preset(config conf, dsp_preset & out) {
		dsp_preset_builder builder;
		builder << conf.freq << conf.fm_gain << conf.mode << conf.tx_vga << conf.enableamp;
		builder.finish(g_get_guid(), out);
	}


	static void parse_preset(config & conf, const dsp_preset & in) {

		try {
			dsp_preset_parser parser(in);
			parser >> conf.freq >> conf.fm_gain >> conf.mode >> conf.tx_vga >> conf.enableamp;
		}
		catch (exception_io_data) { conf = default; }
	}

private:
	HANDLE m_hMutex;

	hackrf_device * _dev;
	int ret;
	uint64_t freq;
	float fm_gain;
	uint32_t mode;
	uint32_t tx_vga;
	uint8_t enableamp;

	uint32_t m_sample_rate;
	size_t m_sample_count;
	uint32_t nch;
	uint32_t ch_mask;

	CString str;
	BOOL debug = false;;

	config conf;
	int8_t ** _buf;
	int count;
	int tail;
	int head;
	int offset;
	uint32_t samp_avail;

	float * audio_buf;
	float * audio_IQ_buf;
	float * new_audio_sample;
	double fm_phase;
	double fm_deviation;
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
		RangeMin = 0,
		RangeMax = 100,
		RangeTotal = RangeMax - RangeMin,
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
		m_slider.SetRange(0, RangeTotal);
		m_slider_tx.SetRange(0, TxGainTotal);

		{
			dsp_sample::parse_preset(_config, m_initData);
			m_slider.SetPos(pfc::clip_t<t_int32>(pfc::rint32(_config.fm_gain), RangeMin, RangeMax) - RangeMin);
			m_slider_tx.SetPos(pfc::clip_t<t_int32>(pfc::rint32(_config.tx_vga), TXGainMin, TXGainMax) - TXGainMin);

			m_edit_freq.SetLimitText(7);
			char freq[20];
			sprintf_s(freq, "%.2f", _config.freq);
			pfc::string_formatter msg; msg << freq;
			::uSetDlgItemText(*this, IDC_EDIT_FREQ, msg);

			m_check_amp.SetCheck(_config.enableamp);

			m_combo_mode.AddString(L"WBFM");
			m_combo_mode.AddString(L"NBFM");
			m_combo_mode.SetCurSel(_config.mode);

			RefreshFreqLabel(_config.freq);
			RefreshLabel((uint32_t)_config.fm_gain);
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
			RefreshFreqLabel(conf.freq);
			conf.mode = m_combo_mode.GetCurSel();
			conf.enableamp = m_check_amp.GetCheck();
			conf.fm_gain = (float)(m_slider.GetPos() + RangeMin);
			conf.tx_vga = (uint32_t)(m_slider_tx.GetPos() + RangeMin);

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

		conf.fm_gain = (float)(m_slider.GetPos() + RangeMin);
		conf.tx_vga = (uint32_t)(m_slider_tx.GetPos() + RangeMin);
		{
			dsp_preset_impl preset;
			dsp_sample::make_preset(conf, preset);
			m_callback.on_preset_changed(preset);
		}
		RefreshLabel((uint32_t)conf.fm_gain);
		RefreshTXLabel(conf.tx_vga);
	}


	void RefreshFreqLabel(double val) {
		char freq[20];
		sprintf_s(freq, "%.2f", val);
		pfc::string_formatter msg; msg << "Freq:" << freq << " Mhz";
		::uSetDlgItemText(*this, IDC_SLIDER_FREQ_LABEL, msg);

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