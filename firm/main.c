#include <stdint.h>



extern char _core_private_start;
extern char _core_private_end;

#define RED (0b11000000)
#define BLUE (0b111000)
#define GREEN (0b111)

#define WHITE 0xff

//#define OUTADDR ((volatile uint32_t*)&_core_private_start)
__attribute__((section(".core_private"))) uint32_t OUTADDR[100] = { 0xDEADBEEF, CORE_ID};
#if CORE_ID == 0
	__attribute__((section(".core_private"))) uint8_t color_arr[] = {RED, GREEN, BLUE, RED};
#elif CORE_ID == 1
	__attribute__((section(".core_private"))) uint8_t color_arr[] = {GREEN, BLUE, RED, GREEN};
#elif CORE_ID == 2
	__attribute__((section(".core_private"))) uint8_t color_arr[] = {BLUE, RED, GREEN, BLUE};
#elif CORE_ID == 3
	__attribute__((section(".core_private"))) uint8_t color_arr[] = {RED, GREEN, BLUE, RED};
#endif
__attribute__((section(".frame_buffer"))) uint8_t fb[100*80];


int main(void) {
	const int core_id = OUTADDR[1];
	for(int i = 0; i < 10; i++)
		OUTADDR[i + 2] = OUTADDR[1] * i;
	for(int blk_i = 0; blk_i < 4; blk_i++) {
		for(int j = 0; j < 20; j++) {
			int y_i = blk_i * 20 + j;
			for(int i = 0; i < 25; i++) {
				int x_i = core_id * 25 + i;
				fb[x_i + y_i * 100] = color_arr[blk_i];
			}
		}
	}
	while(1); // Hang here
}
