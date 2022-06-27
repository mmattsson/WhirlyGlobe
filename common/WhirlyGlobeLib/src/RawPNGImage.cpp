/*
 *  RawPNGImage.cpp
 *  WhirlyGlobeLib
 *
 *  Created by Steve Gifford on 12/3/20.
 *  Copyright 2011-2022 mousebird consulting
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <stdlib.h>
#include <string>
#import "WhirlyKitLog.h"
#import "RawPNGImage.h"

// Note: These also need to be set on the compiler options for lodepng.cpp
//       in order to actually exclude the un-used code from the build.
//#define LODEPNG_COMPILE_PNG
//#define LODEPNG_COMPILE_ZLIB
//#define LODEPNG_COMPILE_DECODER
#define LODEPNG_NO_COMPILE_ENCODER
#define LODEPNG_NO_COMPILE_DISK
// Note that this disables color profiles (ICC, gamma, whitepoint), background color for
// transparent pixels, text chunks, modifiation time, extension and unrecognized chunks,
// but *not* transparent color-key (tRNS)
#define LODEPNG_NO_COMPILE_ANCILLARY_CHUNKS
#define LODEPNG_NO_COMPILE_ERROR_TEXT
//#define LODEPNG_COMPILE_ALLOCATORS
#define LODEPNG_NO_COMPILE_CRC  // We'll use the one from libz provided by the system
#define LODEPNG_NO_COMPILE_CPP  // We'll use the C API
#import "lodepng.h"

// Use zlib's crc32
#import <zlib.h>
unsigned lodepng_crc32(const unsigned char* buffer, size_t length)
{
    return crc32_z(crc32(0L, Z_NULL, 0), buffer, length);
}

namespace WhirlyKit
{

static int getByteWidth(const LodePNGState& pngState)
{
    switch (pngState.info_png.color.colortype)
    {
        case LCT_GREY:  return 1 * pngState.info_raw.bitdepth / 8;
        case LCT_RGB:   return 3 * pngState.info_raw.bitdepth / 8;
        case LCT_RGBA:  return 4 * pngState.info_raw.bitdepth / 8;
        default:        return 0;
    }
}

unsigned char *RawPNGImageLoaderInterpreter(unsigned int &width,unsigned int &height,
                                          const unsigned char *data,size_t length,
                                          const std::vector<int> &valueMap,
                                          int &byteWidth,
                                          unsigned int &err,
                                          std::string* errStr)
{
    unsigned char *outData = NULL;
    try
    {
        LodePNGState pngState;
        lodepng_state_init(&pngState);
        err = lodepng_inspect(&width, &height, &pngState, data, length);
        if (!err)
        {
            byteWidth = getByteWidth(pngState);
            err = lodepng_decode_memory(&outData, &width, &height, data, length,
                                        pngState.info_png.color.colortype,
                                        pngState.info_raw.bitdepth);
        }
        if (!err && byteWidth < 1)
        {
            if (errStr)
            {
                *errStr = "Unsupported image type";
            }
            err = -1;
        }
    }
    catch (const std::exception &ex)
    {
        wkLogLevel(Error, "Exception in MaplyQuadImageLoader::dataForTile: %s", ex.what());
        if (errStr)
        {
            *errStr = ex.what();
        }
        err = -2;
    }
    catch (...)
    {
        wkLogLevel(Error, "Exception in MaplyQuadImageLoader::dataForTile");
        if (errStr)
        {
            *errStr = "Unknown exception";
        }
        err = -3;
    }

#if defined(LODEPNG_COMPILE_ERROR_TEXT)
    if (err > 0 && errStr)
    {
        *errStr = lodepng_error_text(err);
    }
#endif

    // Remap data values
    if (byteWidth == 1 && !valueMap.empty())
    {
        unsigned char *data = outData;
        for (unsigned int ii=0;ii<width*height;ii++)
        {
            int newVal = valueMap[*data];
            if (newVal >= 0)
            {
                *data = newVal;
            }
            data++;
        }
    }
    
    return outData;
}

}

