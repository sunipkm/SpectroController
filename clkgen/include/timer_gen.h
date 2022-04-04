#ifndef TIMER_GEN_H
#define TIMER_GEN_H
#include <stdlib.h>

typedef enum
{
#ifndef __APPLE__
TIMER_SINGLE_SHOT = 0,
#endif
TIMER_PERIODIC
} t_timer;

typedef void (*time_handler)(size_t timer_id, void * user_data);

int     initialize();
size_t  start_timer(unsigned long long int interval, time_handler handler, t_timer type, void * user_data);
size_t  update_timer(size_t timer_id, unsigned long long int interval, t_timer type);
void    stop_timer(size_t timer_id);
void    finalize();

#endif
