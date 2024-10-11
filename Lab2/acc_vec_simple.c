#include <stdint.h>

int main()
{
    int32_t a[] = {0x100, 0x101, 0x102, 0x103, 0x104};
    int32_t sum = 0;

    sum += a[0];
    sum += a[1];
    sum += a[2];
    sum += a[3];
    sum += a[4];

    return sum;
}