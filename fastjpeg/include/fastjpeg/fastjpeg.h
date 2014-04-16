/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with
  or without modification, are permitted provided that the
  following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#ifndef FASTJPEG_H
#define FASTJPEG_H

#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <jpeglib.h>


namespace fastjpeg
{

struct Image
{
  std::vector<uint8_t> pix;
  int width, height;
  int nchannels;
};

struct jpeg_decompress_struct* init_decompress();
struct jpeg_compress_struct* init_compress();
void release( struct jpeg_decompress_struct **cinfo );

/** Decompresses the data in jpeg_data
  *
  * - If dest_img is provided, it will be used to store the decompressed image.
  * Otherwise the destination image will be allocated.
  * - If histogram is provided, it will contain the histogram of values
  * (TODO: how should it be allocated, and what does the histogram mean)
  *
  * Returns the decompressed image.
  */
Image* decompress_memory( struct jpeg_decompress_struct *cinfo,
                          const unsigned char *jpeg_data, unsigned jpeg_size,
                          Image *dest_img=0, int *histogram=0 );

Image* decompress_file( jpeg_decompress_struct *cinfo, FILE *input_file,
                        Image* dest_img, int n_output_channels );

/**
  * Quality is from 5..95
  **/
void compress_file( jpeg_compress_struct *cinfo, Image* src_img,
                    const std::string& output_filename, int quality );
  
} //namespace fastjpeg

#endif //FASTJPEG_H
