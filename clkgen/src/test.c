#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "clkgen.h"

void time_handler1(clkgen_t timer_id, void * user_data)
{
    printf("Periodic timer 0x%lx: clk %d\n", timer_id, *(int *) user_data);
    *(int *) user_data = !(*(int *) user_data);
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Usage: ./test.out <CLK half cycle in nsec>\n\n");
        return 0;
    }
    static int clk = 0;
    unsigned long long int halfperiod = atoll(argv[1]);
    if (halfperiod > 5 * NSEC_PER_SEC)
    {
        printf("Warning: Clock period greater than 5 seconds, test program runs for 5 seconds. Setting half period to 2 seconds.\n");
        halfperiod = 2 * NSEC_PER_SEC;
    }
    if (halfperiod == 0)
    {
        printf("Invalid half period 0 ns, exiting...\n");
        return 0;
    }
    clkgen_t clkgen = create_clk(halfperiod, time_handler1, &clk);
    sleep(5);
    clkgen = update_clk(clkgen, halfperiod >> 1);
    sleep(5 / 2);
    destroy_clk(clkgen);
    return 0;
}
