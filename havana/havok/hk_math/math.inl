#ifdef HK_PS2
#	include <hk_math/math_ps2.inl>
#else	//HK_PS2

#ifdef _WIN32
inline hk_double hk_Math::fabsd( hk_double r ) { return hk_double(hk_Math::fabs(r)); }

inline hk_real hk_Math::sqrt( hk_real r) { return hk_real(hk_Math::sqrt(r)); }
inline hk_real hk_Math::sqrt_inv( hk_real r) { return 1.0f / hk_real(hk_Math::sqrt(r)); }

inline hk_real hk_Math::fast_sqrt( hk_real r) { return hk_real(hk_Math::sqrt(r)); }
inline hk_real hk_Math::fast_sqrt_inv( hk_real r) { return 1.0f / hk_real(hk_Math::sqrt(r)); }

inline hk_real hk_Math::fabs( hk_real r) { return hk_real(hk_Math::fabs(r)); }
inline hk_real hk_Math::tan( hk_real r) { return hk_real(hk_Math::tan(r)); }
inline hk_real hk_Math::sin( hk_real r) { return hk_real(hk_Math::sin(r)); }
inline hk_real hk_Math::cos( hk_real r) { return hk_real(hk_Math::cos(r)); }
inline hk_real hk_Math::atan2( hk_real a, hk_real b)  { return hk_real(hk_Math::atan2(a,b)); }
inline hk_real hk_Math::asin( hk_real r) { return hk_real(hk_Math::asin(r)); }
inline hk_real hk_Math::acos( hk_real r) { return hk_real(hk_Math::acos(r)); }
inline hk_real hk_Math::max( hk_real a, hk_real b) { return a>b ? a : b; }
inline hk_real hk_Math::min( hk_real a, hk_real b) { return a<b ? a : b; }

inline hk_real hk_Math::exp( hk_real e) { return hk_real(hk_Math::exp(e)); }

inline hk_real hk_Math::floor( hk_real r ) { return hk_real(hk_Math::floor(r)); }
inline hk_real hk_Math::ceil( hk_real r) { return hk_real(hk_Math::ceil(r)); }
inline hk_real hk_Math::pow( hk_real r, hk_real p) { return hk_real(hk_Math::pow(r,p)); }


#elif _LINUX

namespace c_math
{
	extern "C"
	{
		double sqrt(double);
		double fabs(double);
		double sin(double);
		double cos(double);
		double tan(double);
		double atan2(double, double);
		double asin(double);
		double acos(double);
		double exp(double);
		double floor(double);
		double ceil(double);
		double pow(double, double);
	}
}

inline hk_double hk_Math::fabsd( hk_double r ) { return hk_double(c_math::fabs(r)); }

inline hk_real hk_Math::sqrt( hk_real r) { return hk_real(c_math::sqrt(r)); }
inline hk_real hk_Math::sqrt_inv( hk_real r) { return 1.0f / hk_real(c_math::sqrt(r)); }

inline hk_real hk_Math::fast_sqrt( hk_real r) { return hk_real(c_math::sqrt(r)); }
inline hk_real hk_Math::fast_sqrt_inv( hk_real r) { return 1.0f / hk_real(c_math::sqrt(r)); }

inline hk_real hk_Math::fabs( hk_real r) { return hk_real(c_math::fabs(r)); }
inline hk_real hk_Math::tan( hk_real r) { return hk_real(c_math::tan(r)); }
inline hk_real hk_Math::sin( hk_real r) { return hk_real(c_math::sin(r)); }
inline hk_real hk_Math::cos( hk_real r) { return hk_real(c_math::cos(r)); }
inline hk_real hk_Math::atan2( hk_real a, hk_real b)  { return hk_real(c_math::atan2(a,b)); }
inline hk_real hk_Math::asin( hk_real r) { return hk_real(c_math::asin(r)); }
inline hk_real hk_Math::acos( hk_real r) { return hk_real(c_math::acos(r)); }
inline hk_real hk_Math::max( hk_real a, hk_real b) { return a>b ? a : b; }
inline hk_real hk_Math::min( hk_real a, hk_real b) { return a<b ? a : b; }

inline hk_real hk_Math::exp( hk_real e) { return hk_real(c_math::exp(e)); }

inline hk_real hk_Math::floor( hk_real r ) { return hk_real(c_math::floor(r)); }
inline hk_real hk_Math::ceil( hk_real r) { return hk_real(c_math::ceil(r)); }
inline hk_real hk_Math::pow( hk_real r, hk_real p) { return hk_real(c_math::pow(r,p)); }
#endif

inline hk_real hk_Math::clamp( hk_real r, hk_real mn, hk_real mx)
{
	HK_ASSERT(mn<=mx);
	return ((r<mn)
				? mn
				: ((r>mx) ? mx : r));
}

inline int hk_Math::int_log2( hk_real ) { return 0; }


inline hk_real hk_Math::_rand01()
{
	// BSD rand function
	const unsigned a = 1103515245;
	const unsigned c = 12345;
	const unsigned m = unsigned(-1) >> 1;
	hk_random_seed = (a * hk_random_seed + c ) & m;
	return hk_real(hk_random_seed) / m;
}

inline hk_real hk_Math::fast_approx_atan2_normized( hk_real y, hk_real x)
{
	//const hk_real delta_at_pi_4 = HK_PI * 0.25f - hk_Math::sin(HK_PI * 0.25f);
	//const hk_real f = delta_at_pi_4 / hk_Math::sin(HK_PI * 0.25f) / hk_Math::sin(HK_PI * 0.25f)/ hk_Math::sin(HK_PI * 0.25f);
	const hk_real f = 0.2214414775f;
	hk_real r;
	if ( x > y ){
		if ( x > - y){
			r = y;
			r += y * y * y * f;
		}else{
			r = x - HK_PI_2;
			r += x * x * x * f;
		}
	}else{
		if ( x > - y){
			r = HK_PI_2 - x;
			r -= x * x * x * f;
		}else{
			if ( y > 0 ){
				r = HK_PI - y;
			}else{
				r = -y - HK_PI;
			}
			r -= y * y * y * f;
		}
	}
	return r;
}

inline hk_real hk_Math::fast_approx_atan2( hk_real y, hk_real x)
{
	//const hk_real delta_at_pi_4 = HK_PI * 0.25f - hk_Math::sin(HK_PI * 0.25f);
	//const hk_real f = delta_at_pi_4 / hk_Math::sin(HK_PI * 0.25f) / hk_Math::sin(HK_PI * 0.25f)/ hk_Math::sin(HK_PI * 0.25f);
	const hk_real f = 0.2214414775f;
	hk_real r;
	hk_real q = hk_Math::sqrt_inv( x * x + y * y );
	if ( x > y ){
		if ( x > - y){
			y *= q;
			r = y;
			r += y * y * y * f;
		}else{
			x *= q;
			r = x - HK_PI_2;
			r += x * x * x * f;
		}
	}else{
		if ( x > - y){
			x *= q;
			r = HK_PI_2 - x;
			r -= x * x * x * f;
		}else{
			y *= q;
			if ( y > 0 ){
				r = HK_PI - y;
			}else{
				r = -y - HK_PI;
			}
			r -= y * y * y * f;
		}
	}
	return r;
}


#endif// HK_PS2

