#include "fonts.h"

//#include "ascii0816.h"
#include "ascii8x16_lh.h"
//#include "ascii8x16_rh.h"
#include "hzk1616.h"


// -------------------------------------------------------------- //
sFONT EN_Font8x16 = {
	nAsciiDot8x16,
	8, /* Width */
	16, /* Height */
};

sFONT CH_Font16x16 = {
  (const uint8_t*)GB_16,
  16, /* Width */
  16, /* Height */
};
