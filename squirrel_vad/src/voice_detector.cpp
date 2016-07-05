
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

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
#include <iterator>

#include "webrtc_vad.h"
#include "audio_file_read.h"

using namespace std;
using namespace boost;
namespace po = boost::program_options;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "voice_detector");
// %EndTag(INIT)%

	int rate = 16000;
	int flength = 320; // rate / (1000(ms) / 20(ms)) how many samples are in each frame?
	int mode = 0;
	int window_len = 20; //10,20,30ms
	int threshold = 100;
	string input_wav = "";
	string output_txt = "./output.txt";
	string output_wav = "./output.raw";
	string device = "";

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
					("output", po::value<string>(&output_txt)->default_value("./output.txt"), "output txt")
("device", po::value<string>(&device)->default_value("default"), "device name (pactl list)");

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
	static const pa_sample_spec ss = {PA_SAMPLE_S16LE, 16000,1};

	pa_simple *s = NULL;
	int error;

	FILE * infile = fopen(input_wav.c_str(),"rb");		// Open wave file in read mode
	FILE * outfile = fopen(output_txt.c_str(),"w");
	FILE * wavfile = fopen(output_wav.c_str(),"wb");

	int nFramesPerSec = 1000 / window_len;
	int totalSamples = 0;
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("voice_detector", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
 
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

		while (!feof(infile) && ros::ok())
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

					fprintf(outfile, "speech ends:\t%ld\n", speechEnd);
					cout << "speech ends: " << speechStart << endl;
				}

				count++;
			}
		}
	}
	else
	{
		while(ros::ok()) {

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
				
				count++;

				 std_msgs::String msg;

    std::stringstream ss;
   
ss << "speech [" << count/(nFramesPerSec *2) << "]: " << output;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
			

}
		}
	}

	free(meta);

	WebRtcVad_Free(handle);

	fclose(wavfile);
	fclose(outfile);



  return 0;
}
// %EndTag(FULLTEXT)%
