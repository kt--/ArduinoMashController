////
//// Copyright (C) 2016 Kingsley Turner, krt@krt.com.au
//// See LICENCE.txt for details, but licensed CC BY-NC-SA
////

#ifndef __EXTRA_GRAPHICS_H__
#define __EXTRA_GRAPHICS_H__

// NOTE: The code below needs to be mered into the PDQ_GFX library;
//       specifically into the PDQ_GFX.h file.  This code is not designed
//       work as-is (because it can't).
//
//       The three function definitions need to be plaed in the 
//       PDQ_GFX class public section, and the function bodies can
//       be placed anywhere along with the other functions in
//       that same file.
//

/*
#include <stdint.h>
#include <PDQ_GFX.h>   
#include <PDQ_ILI9341.h> 
#include "utils.h"
*/

/*

THESE MUST BE ADDED INTO THE PDQ_GFX LIBRARY SINCE BECAUSE IT's DONE AS A C++ Template Class, it's not possible to just define them here.
Do not uncomment this code.
    
////    
//// Add these definitions into the PDQ_GFX.h PDQ_GFX class definition "public" section
////

    static void drawFourEllipsePixels(coord_t x_centre, coord_t y_centre, int x, int y, color_t colour);
    static void drawEllipse(coord_t x_centre, coord_t y_centre, coord_t width, coord_t height, color_t colour);
    static void draw16bitBitmap(coord_t corner_x, coord_t corner_y, int width, int height, unsigned short *pixel_data);

////
//// Add these function bodies into PDQ_GFX.h anywhere with the other function bodies.
////

// Paint an XxY 16-bit colour bitmap to the screen.
// So <pixel_data> is just a long list of 16-bit colour values
template<class HW>
void PDQ_GFX<HW>::draw16bitBitmap(coord_t corner_x, coord_t corner_y, int width, int height, unsigned short *pixel_data)
{
    unsigned short *pixel_pointer = pixel_data;
    unsigned int    pixel_counter = width * height;

    for (unsigned int y=0; y<height; y++)
    {
        for (unsigned int x=0; x<width; x++)
        {
            HW::drawPixel(corner_x + x, corner_y + y, *pixel_pointer);
            pixel_pointer++;
        }
    }
}

// Ellipses are naturally 4-way symmetrical, use that to our advantage
template<class HW>
void PDQ_GFX<HW>::drawFourEllipsePixels(coord_t x_centre, coord_t y_centre, int x, int y, color_t colour)
{
    HW::drawPixel(x_centre + x, y_centre + y, colour);    
    HW::drawPixel(x_centre - x, y_centre + y, colour);    
    HW::drawPixel(x_centre + x, y_centre - y, colour);    
    HW::drawPixel(x_centre - x, y_centre - y, colour);    
}

// Plot an ellipse *CENTERED* about x_centre and y_centre.
template<class HW>
void PDQ_GFX<HW>::drawEllipse(coord_t x_centre, coord_t y_centre, coord_t width, coord_t height, color_t colour)
{
    // Source: https://sites.google.com/site/ruslancray/lab/projects/bresenhamscircleellipsedrawingalgorithm/bresenham-s-circle-ellipse-drawing-algorithm
    
    // KT - Changed from int to long because of arduino under/overflows
    long a2 = width * width;
    long b2 = height * height;
    long fa2 = 4 * a2;
    long fb2 = 4 * b2;
    long x, y, sigma;

    // first half
    for (x = 0, y = height, sigma = 2*b2+a2*(1-2*height); b2*x <= a2*y; x++)
    {
        drawFourEllipsePixels(x_centre, y_centre, x, y, colour);
        if (sigma >= 0)
        {
            sigma += fa2 * (1 - y);
            y--;
        }
        sigma += b2 * ((4 * x) + 6);
    }

    // second half 
    for (x = width, y = 0, sigma = 2*a2+b2*(1-2*width); a2*y <= b2*x; y++)
    {
        drawFourEllipsePixels(x_centre, y_centre, x, y, colour);
        if (sigma >= 0)
        {
            sigma += fb2 * (1 - x);
            x--;
        }
        sigma += a2 * ((4 * y) + 6);
    }
}
*/
  
  
#endif // #ifndef __EXTRA_GRAPHICS_H__  
