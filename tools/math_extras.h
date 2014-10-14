#ifdef _WIN32

#ifndef round
static inline double round(double val)
{
	return floor(val + 0.5);
}
#endif

#ifndef isnan
static inline bool isnan(double val)
{
	return !(val >= DBL_MIN && val <= DBL_MAX);
}
#endif

#ifndef M_PI_4
#define M_PI_4 M_PI/4.0
#endif


#endif
