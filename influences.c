#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.141592653589793

void computeInfluenceMatrices(
    int N, // Number of collocation points
    int T, // Number of vortex segments
    double *points_flat, // 1D pointer to collocation points (Nx3)
    double *table_flat,   // 1D pointer to vortex table (Tx7)
    double *uInfluence_flat, // 1D pointer to u influence matrix (NxN)
    double *vInfluence_flat, // 1D pointer to v influence matrix (NxN)
    double *wInfluence_flat  // 1D pointer to w influence matrix (NxN)
){

    // Make them 2D arrays
    double (*points)[3] = (double (*)[3]) points_flat;
    double (*table)[7] = (double (*)[7]) table_flat;
    double (*uInfluence)[N] = (double (*)[N]) uInfluence_flat;
    double (*vInfluence)[N] = (double (*)[N]) vInfluence_flat;
    double (*wInfluence)[N] = (double (*)[N]) wInfluence_flat;

    // Print collocation points
    // for (int i = 0; i < N; i++) {
    //     printf("Collocation point %d: (%f, %f, %f)\n", i, points[i][0], points[i][1], points[i][2]);
    // }

    // Print vortex table
    for (int j = 0; j < T; j++) {
        printf("Vortex segment %d: (%f, %f, %f) to (%f, %f, %f) HS %d\n", j, table[j][0], table[j][1], table[j][2], table[j][3], table[j][4], table[j][5], (int)(table[j][6]));
    }

}