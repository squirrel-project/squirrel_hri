
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include <squirrel_vad_msgs/vad.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <pulse/simple.h>
#include <pulse/error.h>
#include <pulse/gccmacro.h>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options.hpp>

#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <iterator>

#include "webrtc_vad.h"
#include "audio_file_read.h"

using namespace std;
using namespace boost;
using namespace boost::posix_time;

namespace po = boost::program_options;

void publish_vad_msg(ros::Publisher &chatter_pub, const ros::Time& timestamp, int duration, int energy)
{	
	squirrel_vad_msgs::vad msg;
	msg.header.stamp = timestamp;
	msg.duration = duration;
	msg.energy = energy;


	chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "voice_detector");
	if (!ros::isInitialized())
    	return 1;

	int rate = 16000;	
	int mode = 0;
	int window_len = 20; //10,20,30ms
	int flength = 320; // rate / (1000(ms) / 20(ms)) how many samples are in each frame?
	int threshold = 100;
	int minLength = 500; // 500ms
	string device = "";
	string ros_topic = "voice_detector";
	int count = 0;						// For counting number of frames in wave file.
	header_p meta = (header_p)malloc(sizeof(header));	// header_p points to a header struct that contains the wave file metadata fields


	po::options_description desc("Allowed options");
	desc.add_options()
													("help", "produce help message")
													("window_len", po::value<int>(&window_len)->default_value(20), "window_len in ms(10,20,30)")
													("sp_rate", po::value<int>(&rate)->default_value(16000), "sampling rate in hz")
													("mode", po::value<int>(&mode)->default_value(0), "mode: 0(normal),1(lowbit),2(aggressive),3(very aggressive)")
													("threshold", po::value<int>(&threshold)->default_value(100), "threshold for energy")
													("minLength", po::value<int>(&minLength)->default_value(500), "minimum length of an utterance(ms)")
													("device", po::value<string>(&device)->default_value("default"), "device name (pactl list)")
													("topic", po::value<string>(&ros_topic)->default_value("voice_detector"), "ros topic");

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

	//setting frame length 
	flength = rate / (1000/window_len);
	static const pa_sample_spec ss = {PA_SAMPLE_S16LE, rate, 1};

	pa_simple *s = NULL;
	int error;

	int nFramesPerSec = 1000 / window_len;
	int totalSamples = 0;

	ros::NodeHandle n;

	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("voice_detector", 1000);
	ros::Publisher chatter_pub = n.advertise<squirrel_vad_msgs::vad>(ros_topic, 1000);
	// %EndTag(PUBLISHER)%

	ros::Rate loop_rate(10);

	ROS_INFO("using microphone");

	if(device == "default")
		s = pa_simple_new(NULL, "gmm-vad-mic", PA_STREAM_RECORD, NULL,
				"record", &ss, NULL, NULL, &error);
	else
		s = pa_simple_new(NULL, "gmm-vad-mic", PA_STREAM_RECORD, device.c_str(),
				"record", &ss, NULL, NULL, &error);

	if (!s)
	{
		fprintf(stderr, __FILE__": pa_simple_new() failed: %s\n",
				pa_strerror(error));
		return -1;
	}


	VadInst* handle = WebRtcVad_Create();

	int output = 0;
	VadResult vad_output;

	output =  WebRtcVad_Init(handle);
	ROS_INFO("Init: %d ", output);
	if(output == -1)
	{
		ROS_INFO("Failed to init vad");
		exit(1);
	}

	output = WebRtcVad_set_mode(handle, mode);
	ROS_INFO("Mode: %d ", output);
	if(output == -1)
	{
		ROS_INFO("Failed to set a mode");
		exit(1);
	}

	WebRtcVad_setThreshold(handle, threshold);
	ROS_INFO("Threshold: %d ", threshold);

	output = WebRtcVad_ValidRateAndFrameLength(rate, flength);
	ROS_INFO("Sampling rate: %d and flength: %d\n", rate, flength);
	if(output == -1)
	{
		ROS_INFO("Failed to set sampling rate and frame length.");
		exit(1);
	}

	ROS_INFO("Total sampels: %d", totalSamples );

	int16_t buff16[flength];				// short int used for 16 bit as input data format is 16 bit PCM audio
	for(int i = 0; i < flength; i++)
		buff16[i] = 0;

	int nb;							// variable storing number of bytes returned
	bool isCurrentlySpeech = false;
	system_time speechStart, speechEnd;


	while(ros::ok()) {

		/* Capture data from microphone... */
		if ((nb = pa_simple_read(s, buff16, sizeof(buff16), &error)) < 0)
		{
			fprintf(stderr, __FILE__": pa_simple_read() failed: %s\n",
					pa_strerror(error));
			return -1;
		}
		count++;

		if(nb == 0)
		{
			vad_output = WebRtcVad_Process(handle, rate, buff16, flength);

			//Speech starts
			if(isCurrentlySpeech == false && vad_output.vad == 1)
			{
				isCurrentlySpeech = true;
				//speechStart = count/(nFramesPerSec *2);
				speechStart = get_system_time();

				ROS_INFO("sound starts: %s", to_simple_string(speechStart).c_str());
			}
			else if(isCurrentlySpeech == true && vad_output.vad == 1)
			{
				ROS_INFO("sound keeps coming");
				ROS_INFO("sound energy: %d", vad_output.total_energy);
			}
			//Speech ends
			else if(isCurrentlySpeech == true && vad_output.vad == 0)
			{
				//speechEnd = count/(nFramesPerSec *2);

				isCurrentlySpeech = false;
				speechEnd = get_system_time();
				ROS_INFO("sound ends : %s", to_simple_string(speechEnd).c_str());
				
				time_duration td(speechEnd - speechStart);
				int duration = td.total_milliseconds();
				if(duration > minLength)
				{
					//std_msgs::String msg;
					ROS_INFO("Detected speech duration: %d", duration);				

					ros::Time timestamp = ros::Time::now();
					
					publish_vad_msg(chatter_pub, timestamp, duration, vad_output.total_energy);
					
				}
			}

			ros::spinOnce();
		}
	}


	free(meta);

	WebRtcVad_Free(handle);


	return 0;
}
// %EndTag(FULLTEXT)%
