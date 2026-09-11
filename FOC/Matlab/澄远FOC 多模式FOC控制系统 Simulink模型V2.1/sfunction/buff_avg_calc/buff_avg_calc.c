#include "data_type.h"
#include "buff_avg_calc.h"

#define BUFF_SIZE	16
static int32 buff[BUFF_SIZE];

int32 GetBuffAvgVal(int32 in_data)
{
	uint8 i;
	int32 sum;
	int32 avg;
	static uint8 init = 0;
	static uint8 index = 0;
//	static int32 buff[BUFF_SIZE];

	/* clr buff */
	if(init == 0)
	{
		init = 1;
		for(i = 0; i < BUFF_SIZE; i++)
		{
			buff[i] = 0;
		}
	}

	/* put new data in buffer */
	buff[index] = in_data;
	index++;
	if(index >= BUFF_SIZE)
		index = 0;

	/* calc buff average val */
	sum = 0;
	for(i = 0; i < BUFF_SIZE; i++)
	{
		sum += buff[i];
	}
	avg = sum/BUFF_SIZE;

	return avg;
}

void EncClear(void)
{
	uint8 i;
	
	for(i = 0; i < BUFF_SIZE; i++)
	{
		buff[i] = 0;
	}

}

