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

#include "FeatureExceptions.h"
using rimg::Exception;


Exception::Exception( const string &err) : m_err(err){}
Exception::~Exception() throw(){}
const char* Exception::what() const throw(){ return m_err.c_str();}
string Exception::error() const throw(){ return m_err;}
string Exception::errStr() const throw(){ return m_err;}


rimg::FeatureException::FeatureException( const string &es) : Exception( es) {}

rimg::FeatureSizeException::FeatureSizeException( const string &es) : Exception( es) {}

rimg::ScaledFeatureException::ScaledFeatureException( const string &es) : Exception( es) {}

rimg::ImageTypeException::ImageTypeException( const string &es) : Exception( es) {}

rimg::ImageOutOfBoundsException::ImageOutOfBoundsException( const string &es) : Exception( es) {}

rimg::ImageSizeException::ImageSizeException( const string &es) : Exception( es) {}

rimg::DescriptorLengthException::DescriptorLengthException( const string &es) : Exception( es) {}

rimg::VectorLengthException::VectorLengthException( const string &es) : Exception( es) {}

rimg::ExtractorTypeException::ExtractorTypeException( const string& es) : Exception( es) {}
