#ifndef _APP_CLI_H_
#define _APP_CLI_H_

#include "command_interpreter.h"
#include "retarget.h"

#ifndef MAX_COMMAND_ARGUMENTS
#define MAX_COMMAND_ARGUMENTS 16
#endif

void cli_init(void);
void cli_proc(void);
extern uint8_t printmode;

//void updatescreen(void);
void printerror(int8_t error);

#endif
