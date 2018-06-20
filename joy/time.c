#include<sys/timeb.h>
#include <stdio.h>

void main()
{
    struct timeb t;
    ftime(&t);
    printf("%ld",t.time);
}
