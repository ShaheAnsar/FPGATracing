#include <stdint.h>


#define OUTADDR ((volatile uint8_t*)(512))

int fib(int n) {
	if(n <= 0)
		return 0;
	if(n == 1)
		return 1;
	else
		return fib(n-1) + fib(n - 2);
}

int main(void) {
	for(int i = 0; i < 10; i++)
		OUTADDR[i] = fib(i);
	while(1); // Hang here
}
