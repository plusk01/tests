#include "test.h"

// definition of global (ODR)
// Initialized to 0 since stored in .bss (same as static below)
int my_global;

// definition of my static (only visible in this translation unit)
static int my_static;


int read_global(void) {
	return my_global;
}

void inc_global(void) {
	my_global++;
}

int read_static(void) {
	return my_static;
}

void inc_static(void) {
	my_static++;
}