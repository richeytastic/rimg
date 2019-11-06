/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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

template <typename T>
cv::Vec3d calcBarycentric( const cv::Vec<T,3> &A, const cv::Vec<T,3> &B, const cv::Vec<T,3> &C, const cv::Vec<T,3> &P)
{
    const cv::Vec<T,3> PC = P-C;
    const cv::Vec<T,3> AC = A-C;
    const cv::Vec<T,3> BC = B-C;
    const double denom = cv::norm( (A-B).cross(AC));   // Area of parallelogram but no need to divide by 2 since they cancel.
    const double u = cv::norm( PC.cross(BC)) / denom;
    const double v = cv::norm( AC.cross(PC)) / denom;
    return cv::Vec3d( u, v, 1-u-v);
}   // end calcBarycentric


template <typename T, int cn>
cv::Vec3d calcBarycentric( const cv::Vec<T,cn> &A, const cv::Vec<T,cn> &B, const cv::Vec<T,cn> &C, const cv::Vec<T,cn> &P)
{
    const double normAB = cv::norm(A-B);
    const double normAC = cv::norm(A-C);
    const double normBC = cv::norm(B-C);
    const double areaABC = calcTriangleArea( normAB, normAC, normBC);

    const double normAP = cv::norm(A-P);
    const double normBP = cv::norm(B-P);
    const double normCP = cv::norm(C-P);

    const double u = calcTriangleArea( normCP, normBP, normBC) / areaABC;
    const double v = calcTriangleArea( normCP, normAP, normAC) / areaABC;
    const double w = calcTriangleArea( normBP, normAP, normAB) / areaABC;
    return cv::Vec3d( u, v, w);
}   // end calcBarycentric


template <typename T>
double calcTriangleArea( const cv::Vec<T,3>& v0, const cv::Vec<T,3>& v1, const cv::Vec<T,3>& v2)
{
    // Magnitude of cross product is parallelogram's area.
    return cv::norm( (v2-v0).cross(v1-v0))/2;
}   // end calcTriangleArea


template <typename T, int cn>
double calcTriangleArea( const cv::Vec<T,cn>& v0, const cv::Vec<T,cn>& v1, const cv::Vec<T,cn>& v2)
{
    const double a = cv::norm(v0-v1);
    const double b = cv::norm(v1-v2);
    const double c = cv::norm(v2-v0);
    return calcTriangleArea( a, b, c);
}   // end calcTriangleArea


template <typename T>
T l2sq( const cv::Vec<T,2>& v) { return v[0]*v[0] + v[1]*v[1];}


template <typename T>
T l2sq( const cv::Vec<T,3>& v) { return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];}


template <typename T>
T min( T a, T b, T c)
{
    return std::min<T>( a, std::min<T>( b, c));
}   // end min


template <typename T>
T max( T a, T b, T c)
{
    return std::max<T>( a, std::max<T>( b, c));
}   // end max


template <typename T>
T min( T a, T b)
{
    return std::min<T>( a, b);
}   // end min


template <typename T>
T max( T a, T b)
{
    return std::max<T>( a, b);
}   // end max


template <typename T>
void reorderAscending( T& v0, T& v1, T& v2)
{
    if ( v0 > v1)
        std::swap( v0, v1);
    if ( v1 > v2)
        std::swap( v1, v2);
    if ( v0 > v1)
        std::swap( v0, v1);
}   // end reorderAscending

    
template <typename T>
ostream& operator<<( ostream &os, const cv::Size_<T> &s)
{
    return os << s.width << " " << s.height;
}   // end operator<<


template <typename T>
istream& operator>>( istream &is, cv::Size_<T> &s)
{
    return is >> s.width >> s.height;
}   // end operator>>


template <typename T>
ostream& operator<<( ostream &os, const cv::Rect_<T> &r)
{
    return os << "[" << r.x << " " << r.y << " " << r.width << " " << r.height << "]";
}   // end operator<<


template <typename T>
istream& operator>>( istream &is, cv::Rect_<T> &r)
{
    char ch;
    return is >> ch >> r.x >> r.y >> r.width >> r.height >> ch;
}   // end operator>>



template <typename T>
double calcHorizontalGrad( const cv::Mat_<T> &image, int row, int col, int ch)
{
    double grad = 0;
    assert( ch >= 0 && ch < image.channels());
    if ( col > 0 && col < image.cols - 1)
        grad = (double)image(row,col+1)[ch] - (double)image(row,col-1)[ch];
    else if ( col == 0) // At left edge
        grad = (double)image(row,col+1)[ch] - (double)image(row,col)[ch];
    else    // At right edge
        grad = (double)image(row,col)[ch] - (double)image(row,col-1)[ch];

    return grad;
}   // end calcHorizontalGrad



template <typename T>
double calcHorizontalGrad2( const cv::Mat_<T> &image, int row, int col, int ch)
{
    double grad = 0;
    assert( ch >= 0 && ch < image.channels());
    if ( col > 0 && col < image.cols - 1)
    {
        const double d1 = (double)image(row,col+1)[ch] - (double)image(row,col)[ch];
        const double d2 = (double)image(row,col)[ch] - (double)image(row,col-1)[ch];
        grad = d1 - d2; // Second derivative estimate
    }   // end if
    else if ( col == 0) // At left edge
        grad = (double)image(row,col+1)[ch] - (double)image(row,col)[ch];
    else    // At right edge

    return grad;
}   // end calcHorizontalGrad2



template <typename T>
double calcVerticalGrad( const cv::Mat_<T> &image, int row, int col, int ch)
{
    double grad = 0;
    assert( ch >= 0 && ch < image.channels());
    if ( row > 0 && row < image.rows - 1)
        grad = (double)image(row+1,col)[ch] - (double)image(row-1,col)[ch];
    else if ( row == 0) // At top row
        grad = (double)image(row+1,col)[ch] - (double)image(row,col)[ch];
    else    // At bottom row
        grad = (double)image(row,col)[ch] - (double)image(row-1,col)[ch];

    return grad;
}   // end calcVerticalGrad



template <typename T>
double calcVerticalGrad2( const cv::Mat_<T> &image, int row, int col, int ch)
{
    double grad = 0;
    assert( ch >= 0 && ch < image.channels());
    if ( row > 0 && row < image.rows - 1)
    {
        const double d1 = (double)image(row+1,col)[ch] - (double)image(row,col)[ch];
        const double d2 = (double)image(row,col)[ch] - (double)image(row-1,col)[ch];
        grad = d1 - d2; // Second derivative estimate
    }   // end if
    else if ( row == 0) // At top row
        grad = (double)image(row+1,col)[ch] - (double)image(row,col)[ch];
    else    // At bottom row
        grad = (double)image(row,col)[ch] - (double)image(row-1,col)[ch];

    return grad;
}   // end calcVerticalGrad2



template <typename T>
T getIntegralImageSum( const cv::Mat& iimg, const cv::Rect& rct)
{
    const int x1 = rct.x;
    const int y1 = rct.y;
    const int x2 = rct.x + rct.width;
    const int y2 = rct.y + rct.height;
    return iimg.at<T>(y2,x2) - iimg.at<T>(y2,x1) - iimg.at<T>(y1,x2) + iimg.at<T>(y1,x1);
}   // end getIntegralImageSum



template <typename T>
double calcVariance( const cv::Mat& iim, const cv::Mat_<double>& iimsq, const cv::Rect& rct, cv::Mat_<int> maskii)
{
    double sm = (double)getIntegralImageSum<T>( iim, rct);
    int N = rct.area();
    if ( !maskii.empty())
        N = getIntegralImageSum<int>( maskii, rct);
    double mn = sm / N;
    return ( getIntegralImageSum<double>( iimsq, rct) - 2*mn*sm + N*mn*mn) / N;  // Uses binomial theorem
}   // end calcVariance


template <typename T>
cv::Mat_<int> createMaskIntegralImage( const cv::Mat& m, T minv, T maxv, cv::Mat_<byte>& mask)
{
    mask = (m >= minv) & (m <= maxv);
    mask /= 255;    // mask is now zeros and ones (where m is in range)
    cv::Mat iim;
    cv::integral( mask, iim, CV_32S);
    return iim;
}   // end createMaskIntegralImage



template <typename T>
T findSumBetweenPoints( const cv::Mat& dmap, const cv::Point2f& f0, const cv::Point2f& f1, T minv, T maxv, int& count)
{
    assert( dmap.channels() == 1);

    const int ncols = dmap.cols;
    const int nrows = dmap.rows;
    const cv::Point p0( f0.x * ncols, f0.y * nrows);
    const cv::Point p1( f1.x * ncols, f1.y * nrows);

    const int xdiff = p1.x - p0.x;
    const int ydiff = p1.y - p0.y;
    const int nsteps = std::max<int>( abs(xdiff), abs(ydiff));

    const double xdelta = (double)(xdiff)/nsteps;
    const double ydelta = (double)(ydiff)/nsteps;

    count = 0;
    int x, y;
    T d;
    T dsum = 0;
    for ( int i = 0; i < nsteps; ++i)
    {
        x = p0.x + cvRound(i*xdelta);
        y = p0.y + cvRound(i*ydelta);
        d = dmap.ptr<T>(y)[x];

        if ( d > minv && d < maxv)
        {
            dsum += d;
            count++;
        }   // end if
    }   // end for

    return dsum;
}   // end findSumBetweenPoints



template <typename T>
void __createChangeMaps( const cv::Mat& rngImg, cv::Mat& hmap, cv::Mat& vmap, bool doAbs, cv::Mat_<byte> msk)
{
#ifndef NDEBUG
    if ( !msk.empty())
    {
        assert( rngImg.rows == msk.rows);
        assert( rngImg.cols == msk.cols);
    }   // end if
#endif

    // Create absolute difference maps in depth going horizontally and vertically
    hmap = cv::Mat::zeros( rngImg.size(), rngImg.type());   // From left to right
    vmap = cv::Mat::zeros( rngImg.size(), rngImg.type());   // From top to bottom

    const int nrows = rngImg.rows;
    const int ncols = rngImg.cols;

    const T* vprev = rngImg.ptr<T>(0);
    const byte* prevMskRow = NULL;
    if ( !msk.empty())
        prevMskRow = msk.ptr<byte>(0);

    for ( int i = 1; i < nrows; ++i)
    {
        const T* rngRow = rngImg.ptr<T>(i);
        const byte* mskRow = NULL;
        if ( !msk.empty())
            mskRow = msk.ptr<byte>(i);  // Mask row

        T* hrow = hmap.ptr<T>(i);   // Horizontal out
        T* vrow = vmap.ptr<T>(i);   // Vertical out

        for ( int j = 1; j < ncols; ++j)
        {
            // Ignore pixel if not masked at current pixel, above and to the left
            if ( mskRow != NULL && (!mskRow[j] || !prevMskRow[j] || !mskRow[j-1]))
                continue;

            // Set the horizontal change
            hrow[j] = rngRow[j] - rngRow[j-1];
            if ( doAbs)
                hrow[j] = (T)fabs( (double)hrow[j]);

            // Set the vertical change
            vrow[j] = rngRow[j] - vprev[j];
            if ( doAbs)
                vrow[j] = (T)fabs( (double)vrow[j]);
        }   // end for

        vprev = rngRow;
        prevMskRow = mskRow;
    }   // end for
}   // end __createChangeMaps
