#ifndef transmitter_h
#define transmitter_h

#include <home-meteo.h>
#include <RF24.h>


class Transmitter_meteo
{
private:
	uint8_t stack[12];
	uint8_t firstNo=0;
	uint8_t lastNo=0;
public:
	void push(uint8_t pipeNo);
	uint8_t pop();
	bool isEmpty();
};

#endif
