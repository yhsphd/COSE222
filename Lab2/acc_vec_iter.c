#include <stdint.h>

int main()
{
    int32_t a[] = {0x100, 0x101, 0x102, 0x103, 0x104};
    int32_t sum = 0;
    int32_t i;

    for (i = 0; i < 5; i++)
        sum += a[i];

    return sum;
}