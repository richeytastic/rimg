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

#pragma once
#ifndef rimg_STAR_KEYPOINTS_H
#define rimg_STAR_KEYPOINTS_H

#include "KeypointsDetector.h"
using namespace rimg;


namespace rimg
{

class rimg_EXPORT StarKeypoints : public KeypointsDetector
{
public:
    StarKeypoints( const cv::Mat &originalImage);
    virtual ~StarKeypoints(){}

    virtual Keypoints find() const;

private:
    cv::Mat working_image;
};  // end class StarKeypoints

}   // end namespace rimg

#endif



