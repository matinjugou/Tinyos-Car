#ifndef TANKMESSAGE_H
#define TANKMESSAGE_H 

typedef nx_struct TankMsg {
	nx_uint16_t nodeid;
	nx_uint16_t type;
	nx_uint8_t action;
	nx_uint16_t data;
} TankMsg;

typedef nx_struct DebugMsg {
	nx_uint16_t pinA;
	nx_uint16_t pinB;
	nx_uint16_t pinC;
	nx_uint16_t pinD;
	nx_uint16_t pinE;
	nx_uint16_t pinF;
} DebugMsg;

enum {
	AM_TANKMSG = 7,
	AM_DEBUGMSG = 8
};

#endif