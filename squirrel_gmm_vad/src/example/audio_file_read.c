
#include <stdio.h>
#include <stdlib.h>
#include "audio_file_read.h"





int read(char *in, char *out)

{

	FILE * infile = fopen(in,"rb");		// Open wave file in read mode
	FILE * outfile = fopen(out,"wb");		// Create output ( wave format) file in write mode;


	int BUFSIZE = 256;					// BUFSIZE can be changed according to the frame size required (eg:512)
	int count = 0;						// For counting number of frames in wave file.
	short int buff16[BUFSIZE];				// short int used for 16 bit as input data format is 16 bit PCM audio
	header_p meta = (header_p)malloc(sizeof(header));	// header_p points to a header struct that contains the wave file metadata fields
	int nb;							// variable storing number of bytes returned
	if (infile)
	{
		fread(meta, 1, sizeof(header), infile);
		fwrite(meta,1, sizeof(*meta), outfile);



		while (!feof(infile))
		{
			nb = fread(buff16,1,BUFSIZE,infile);		// Reading data in chunks of BUFSIZE
			//cout << nb <<endl;
			count++;					// Incrementing Number of frames



			fwrite(buff16,1,nb,outfile);			// Writing read data into output file (.txt)
		}
	}
return 0;
}
