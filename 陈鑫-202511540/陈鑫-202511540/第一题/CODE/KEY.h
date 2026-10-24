#ifndef KEY_H_
#define KEY_H_

#include "NEUQ_lib_include.h"


#define KEY_Pin  PE10
#define KEY_ACTIVE GPIO_Low

void KEY_init(void);
void Key_Scan(void);
void KEY_Process(void);


#endif // !
