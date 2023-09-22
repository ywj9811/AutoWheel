#ifndef _JOYSTICK_READER_H_
#define _JOYSTICK_READER_H_

int joystick_init();
void* joystick_run(void *arg);
void joystick_close();

#endif