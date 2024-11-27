#ifndef __CONFIG_H__
#define __CONFIG_H__

/* #undef ENABLE_FULL_UNDISTORT */
/* #undef ENABLE_RANDOMIZED_THRESHOLD */
/* #undef ENABLE_VERBOSE */

#if defined(ENABLE_VERBOSE)
#define WHYCON_DEBUG(x) cout << x << endl
#else
#define WHYCON_DEBUG(x)
#endif

#endif
