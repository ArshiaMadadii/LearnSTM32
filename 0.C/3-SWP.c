#include <stdio.h>

int a = 10, b = 50;
char aa[50];

void SWP(int *x, int *y) {
    *x = *x ^ *y;
    *y = *x ^ *y;
    *x = *x ^ *y;
}

int main(void) {
    sprintf(aa, "A=%d  B=%d\r\n", a, b);
    printf("%s", aa);

    SWP(&a, &b);
    sprintf(aa, "A=%d  B=%d", a, b);
    printf("%s\n", aa);

    return 0;
}
