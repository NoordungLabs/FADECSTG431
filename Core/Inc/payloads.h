#ifndef INC_PAYLOADS_H_
#define INC_PAYLOADS_H_

#include <stdbool.h>

struct Pressure {
	float tank1;
	float tank2;
	float injector1;
	float injector2;
};

struct Temperature {
	float tank1;
	float tank2;
	float injector1;
	float injector2;
};

struct Command {
	uint8_t type;
	uint8_t state;
};

#endif /* INC_PAYLOADS_H_ */
