#include <stdio.h>

int main() {
    int a = 5;
    int* b = &a;

    printf("a: %d\n", a);
    printf("b: %p\n", b);
    printf("*b: %d\n", *b);

    int myArray[3] = {1, 2, 3};
    int* myPointer = myArray;

    printf("myPointer: %p\n", myPointer);

    for (int i = 0; i < 3; i++) {
        printf("a[%d]: %p\n", i, &myArray[i]);
    }
    
    return 0;
}