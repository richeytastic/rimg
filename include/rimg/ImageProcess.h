/************************************************************************
 * Copyright (C) 2017 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#ifndef rimg_IMAGE_PROCESS_H
#define rimg_IMAGE_PROCESS_H

#include <opencv2/opencv.hpp>
#include "rimg_Export.h"

namespace rimg {

// Swap the left and right most end bytes of each pixel in the provided image.
// Returns image out for convenience.
rimg_EXPORT cv::Mat swapEndBytes( const cv::Mat &img, cv::Mat &out);

// Reduce the colour space of the image.
rimg_EXPORT void colourReduce( const cv::Mat &img, cv::Mat &out, int div=64);

// Sharpen the image by subtracting the Laplacian (without using explicit convolution kernel).
rimg_EXPORT void sharpen_OLD( const cv::Mat &img, cv::Mat &out);

// Sharpen the image by subtracting the Laplacian (using explicit convolution kernel).
rimg_EXPORT void sharpen( const cv::Mat &img, cv::Mat &out);


}  // end namespace rimg

#endif
