
#ifndef OVR_Math_h
#define OVR_Math_h

#include <stdlib.h>
#include <math.h>
//-----------------------------------------------------------------------------------
// ***** Operator extensions
namespace OVR
{
template <typename T>  void Swap(T &a, T &b) 
{  T temp(a); a = b; b = temp; }


// ***** min/max are not implemented in Visual Studio 6 standard STL

template <typename T> const T Min(const T a, const T b)
{ return (a < b) ? a : b; }

template <typename T> const T Max(const T a, const T b)
{ return (b < a) ? a : b; }

template <typename T> const T Clamp(const T v, const T minVal, const T maxVal)
{ return Max<T>(minVal, Min<T>(v, maxVal)); }

template <typename T> int  Chop(T f)
{ return (int)f; }

template <typename T>int Lerp(T a, T b, T f) 
{ return (int)((b - a) * f + a); }
}
#endif