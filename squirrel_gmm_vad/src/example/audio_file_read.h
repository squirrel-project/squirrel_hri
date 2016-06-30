/*
 * audio_file_read.h
 *
 *  Created on: Apr 12, 2016
 *      Author: kimj
 */

#ifndef AUDIO_FILE_READ_H_
#define AUDIO_FILE_READ_H_

// WAVE PCM soundfile format (you can find more in https://ccrma.stanford.edu/courses/422/projects/WaveFormat/ )
typedef struct header_file
{
    char chunk_id[4];
    int chunk_size;
    char format[4];
    char subchunk1_id[4];
    int subchunk1_size;
    short int audio_format;
    short int num_channels;
    int sample_rate;			// sample_rate denotes the sampling rate.
    int byte_rate;
    short int block_align;
    short int bits_per_sample;
    char subchunk2_id[4];
    int subchunk2_size;			// subchunk2_size denotes the number of samples.
} header;

typedef struct header_file* header_p;

int read_write(char *in, char *out);

#endif /* AUDIO_FILE_READ_H_ */
