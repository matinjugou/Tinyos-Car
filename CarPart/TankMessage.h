#ifndef TANKMESSAGE_H
#define TANKMESSAGE_H 

typedef nx_struct TankMsg {
	nx_uint16_t nodeid;
	nx_uint16_t type;
	nx_uint8_t action;
	nx_uint16_t data;
} TankMsg;

enum {
	AM_TANKMSG = 7
};

#endif