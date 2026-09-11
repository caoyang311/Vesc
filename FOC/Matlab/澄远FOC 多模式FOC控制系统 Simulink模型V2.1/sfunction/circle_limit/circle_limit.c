#include "circle_limit.h"

#define START_INDEX     61
#define MAX_MODULE      32111   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*98%
const uint16 MMITABLE[] = {
	32494,32360,32096,31839,31587,31342,31102,30868,30639,30415,
	30196,29981,29771,29565,29464,29265,29069,28878,28690,28506,
	28325,28148,27974,27803,27635,27470,27309,27229,27071,26916,
	26764,26614,26467,26322,26180,26039,25901,25766,25632,25500,
	25435,25307,25180,25055,24932,24811,24692,24574,24458,24343,
	24230,24119,24009,23901,23848,23741,23637,23533,23431,23331,
	23231,23133,23036,22941,22846,22753,22661
};

void CircleLimit( int16 ud, int16 uq, int16 *ud_l, int16 *uq_l )
{
	uint16 table_element;
	uint32 uw_temp;
	int32  sw_temp;

	(*ud_l) = ud;
	(*uq_l) = uq;
	
	sw_temp = (int32)ud * ud + (int32)uq * uq;
	uw_temp = (uint32) sw_temp;

	/* uw_temp min value 0, max value 2*32767*32767 */
	if ( uw_temp > (uint32)( MAX_MODULE ) * MAX_MODULE )
	{
		uw_temp /= ( uint32 )( 16777216 );

		/* wtemp min value pHandle->Start_index, max value 127 */
		uw_temp -= START_INDEX;

		/* uw_temp min value 0, max value 127 - pHandle->Start_index */
		table_element = MMITABLE[( uint8 )uw_temp];

		sw_temp = ud * ( int32 )table_element;
		(*ud_l) = ( int16 )( sw_temp / 32768 );

		sw_temp = uq * ( int32 )( table_element );
		(*uq_l) = ( int16 )( sw_temp / 32768 );
	}
}

