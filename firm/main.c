#include <stdint.h>


#define OUTADDR ((volatile uint8_t*)(512))

int fac(int n) {
	if(n <= 1)
		return 1;
	else
		return n * fac(n - 1);
}

int main(void) {
	for(int i = 0; i < 10; i++)
		OUTADDR[i] = fac(i);
	while(1); // Hang here
}
