#include<stdio.h>
#include <string.h>

#define ROWS 3
#define COLS 4

struct points{ float x, y, z;};
struct vortex { struct points pts[2]; float gamma; char hsN;};



void vortexPrint(struct vortex *myVortex){
    printf("Vortex:[%.2f, %.2f, %.2f] [%.2f, %.2f, %.2f] hsN=%d \n", 
    myVortex->pts[0].x,
    myVortex->pts[0].y,
    myVortex->pts[0].x,
    myVortex->pts[1].x,
    myVortex->pts[1].y,
    myVortex->pts[1].x,
    myVortex->hsN
    );

}

void printFile(char filename[]){
    FILE *file = fopen(filename,"r");
    char line[256];
    if (file == NULL) {
        printf("Error opening file!\n");
        return 1;
    }
    
    while(fgets(line,sizeof(line), file) !=NULL){
        char *pch = strtok(line, ", ");
        while (pch != 0){
            printf("%s ", pch);
            pch = strtok(NULL, "\n");
        }
        printf("\n");
    }

}

void fileDimension(FILE *file, int *rows) {
}
int main() {
    struct vortex myVortex;
    int nPoints;
    char filename[] = "./testCoordinates.csv";

    printFile(filename);
    
    return 0;
}