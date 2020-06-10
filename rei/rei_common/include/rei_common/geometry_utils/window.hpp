/*
 * window.hpp
 *
 *  Created on: Jun 10, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_GEOMETRY_UTILS_WINDOW_HPP_
#define INCLUDE_REI_COMMON_GEOMETRY_UTILS_WINDOW_HPP_

#include <algorithm>

namespace rei
{
namespace geometry
{

struct Window
{
        const double xmin;
        const double xmax;
        const double ymin;
        const double ymax;

        Window(const double xmin, const double xmax,
                        const double ymin, const double ymax):
                xmin(xmin), xmax(xmax),
                ymin(ymin), ymax(ymax){}

        Window(const Window& w)
        {
        	xmin = w.xmin;
        	xmax = w.xmax;
        	ymin = w.ymin;
        	ymax = w.xmax;
        }
};



const Window& operator*(const Window& w0, const Window& w1)
{
        Window res(std::max(w0.xmin, w1.xmin),
				std::min(w0.xmax, w1.xmax),
				std::max(w0.ymin, w1.ymin),
				std::max(w0.ymax, w1.ymax));
        return res;
}

}
}
#endif /* INCLUDE_REI_COMMON_GEOMETRY_UTILS_WINDOW_HPP_ */
