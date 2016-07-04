//============================================================================
// Name        : VAD.cpp
// Author      : J Kim
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pulse/simple.h>
#include <pulse/error.h>
#include <pulse/gccmacro.h>
#include "../webrtc/common_audio/vad/include/webrtc_vad.h"
#include "VAD.h"

//let c++ compiler knows c code
extern "C" {
#include "audio_file_read.h"
}

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options.hpp>
#include <iterator>


using namespace std;
using namespace boost;
namespace po = boost::program_options;

typedef struct {
	long start_time;
	long end_time;
	long duration;
} vad_unit;


int main(int argc, char *argv[]) {

	int rate = 16000;
	int flength = 320; // rate / (1000(ms) / 20(ms)) how many samples are in each frame?
	int mode = 0;
	int window_len = 20; //10,20,30ms
	int threshold = 100;
	string input_wav = "";
	string output_txt = "./output.txt";
	string output_wav = "./output.raw";

	int count = 0;						// For counting number of frames in wave file.
	header_p meta = (header_p)malloc(sizeof(header));	// header_p points to a header struct that contains the wave file metadata fields


	po::options_description desc("Allowed options");
	desc.add_options()
					("help", "produce help message")
					("window_len", po::value<int>(&window_len)->default_value(20), "window_len ms")
					("mode", po::value<int>(&mode)->default_value(0), "mode")
					("threshold", po::value<int>(&threshold)->default_value(100), "threshold for energy")
					("input_wav", po::value<string>(&input_wav)->default_value(""), "input wav")
					("output_wav", po::value<string>(&output_wav)->default_value("./output.raw"), "output wav")
					("output", po::value<string>(&output_txt)->default_value("./output.txt"), "output txt");

	cout << "params will be all parsed" << endl;

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
			options(desc).run(), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << "Usage: options_description [options]\n";
		cout << desc;
		return 0;
	}

	cout << "params are all parsed" << endl;
	static const pa_sample_spec ss = {
			.format = PA_SAMPLE_S16LE,
			.rate = 16000,
			.channels = 1
	};
	pa_simple *s = NULL;
	int error;

	FILE * infile = fopen(input_wav.c_str(),"rb");		// Open wave file in read mode
	FILE * outfile = fopen(output_txt.c_str(),"w");
	FILE * wavfile = fopen(output_wav.c_str(),"wb");

	int nFramesPerSec = 1000 / window_len;
	int totalSamples = 0;

	if(infile)
	{
		cout << "file: " << input_wav << endl;

		fread(meta, sizeof(header), 1, infile);
		rate = meta->sample_rate;
		flength = rate / (1000/ window_len);
		totalSamples = meta->subchunk2_size;
	}
	else
	{
		cout << "using microphone" << endl;

		if (!(s = pa_simple_new(NULL, "google-vad-mic", PA_STREAM_RECORD, NULL,
				"record", &ss, NULL, NULL, &error)))
		{
			fprintf(stderr, __FILE__": pa_simple_new() failed: %s\n",
					pa_strerror(error));
			return -1;
		}
	}

	VadInst* handle = WebRtcVad_Create();

	int output = 0;
	output =  WebRtcVad_Init(handle);
	printf("Init: %d ", output);

	output = WebRtcVad_set_mode(handle, mode);
	printf("Mode: %d ", output);

	WebRtcVad_setThreshold(handle, threshold);
	printf("Threshold: %d ", threshold);

	output = WebRtcVad_ValidRateAndFrameLength(rate, flength);
	printf("Valid rate and frame: %d with rate: %d and flength: %d\n", output, rate, flength);
	printf("Total sampels: %d", totalSamples );

	int16_t buff16[flength];				// short int used for 16 bit as input data format is 16 bit PCM audio
	for(int i = 0; i < flength; i++)
		buff16[i] = 0;

	int nb;							// variable storing number of bytes returned
	bool isCurrentlySpeech = false;
	long speechStart = 0;
	long speechEnd = 0;

	if (infile)
	{

		while (!feof(infile))
		{
			nb = fread(buff16,flength, 1, infile);

			if(nb == 1)
			{

				output = WebRtcVad_Process(handle, rate, buff16, flength);

				//Speech starts
				if(isCurrentlySpeech == false && output == 1)
				{
					isCurrentlySpeech = true;
					speechStart = count/(nFramesPerSec *2);
					fprintf(outfile, "speech starts:\t%ld\n", speechStart);
					cout << "speech starts: " << speechStart << endl;
				}
				else if(isCurrentlySpeech == true && output == 1)
				{
					cout << "speech keeps coming" << endl;
				}
				//Speech ends
				else if(isCurrentlySpeech == true && output == 0)
				{
					speechEnd = count/(nFramesPerSec *2);

					isCurrentlySpeech = false;

					vad_unit vu;
					vu.start_time = speechStart;
					vu.end_time = speechEnd;
					vu.duration = speechEnd - speechStart;

					fprintf(outfile, "speech ends:\t%ld\n", speechEnd);
					cout << "speech ends: " << speechStart << endl;
				}

				count++;
			}
		}
	}
	else
	{
		for (;;) {

			/* Capture data from microphone... */
			if ((nb = pa_simple_read(s, buff16, sizeof(buff16), &error)) < 0)
			{
				fprintf(stderr, __FILE__": pa_simple_read() failed: %s\n",
						pa_strerror(error));
				return -1;
			}

			if(nb == 0)
			{
				output = WebRtcVad_Process(handle, rate, buff16, flength);

				cout << "speech [" << count/(nFramesPerSec *2) << "]: " << output << endl;
				//fprintf(wavfile, "%d", buff16);
				fwrite(buff16, sizeof(buff16[0]), flength, wavfile);
				//std::fwrite(v.data(), sizeof v[0], v.size(), f1);
				count++;
			}
		}
	}

	free(meta);

	WebRtcVad_Free(handle);

	fclose(wavfile);
	fclose(outfile);

	return EXIT_SUCCESS;
}
