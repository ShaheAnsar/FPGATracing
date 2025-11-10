#include <stdint.h>


#define OUTADDR ((volatile uint8_t*)(512))




int main(void) {
	for(int i = 0; i < 10; i++)
		OUTADDR[i] = i;
	while(1); // Hang here
}
