#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <omp.h>

#define PI 3.141592653589793

// This is a C-code intended to compute influence matrices for a Lifting Line code written in Python 

// The input will be as follows:
    // Number of collocation points (N),
    // Array of collocation points (Nx3), 
    // Vortex table which (Nx8) contains the following information:
        // 1. Point 1 (x1, y1, z1)
        // 2. Point 2 (x2, y2, z2)
        // 3. Horseshoe number (Horseshoe number)
        // 4. Vortex strength (Gamma)

// the output will be 3 influence matrices (NxN) for the three components of the velocity induced by the vortex at the collocation points

void computeVelocityField(
            int N, 
            int T,
            double *points_flat, // 1D pointer
            double *table_flat,   // 1D pointer
            double *u, // 1D pointer
            double *v, // 1D pointer
            double *w  // 1D pointer
        ){
            double (*points)[3] = (double (*)[3]) points_flat;
            double (*table)[8] = (double (*)[8]) table_flat;
            double (*u_vec)[1] = (double (*)[1]) u;
            double (*v_vec)[1] = (double (*)[1]) v;
            double (*w_vec)[1] = (double (*)[1]) w;

            #pragma omp parallel for
            for (int i=0; i<N; i++){
                for(int j=0; j<T; j++){
                    double x1 = table[j][0];
                    double y1 = table[j][1];
                    double z1 = table[j][2];
                    double x2 = table[j][3];
                    double y2 = table[j][4];
                    double z2 = table[j][5];
                    int flagK = 0; 
                    double gamma = table[j][7];
                    double K;

                    double x = points[i][0];
                    double y = points[i][1];
                    double z = points[i][2];

                    double crossX = (y - y1)*(z - z2) - (z - z1)*(y - y2);
                    double crossY = (z - z1)*(x - x2) - (x - x1)*(z - z2);
                    double crossZ = (x - x1)*(y - y2) - (y - y1)*(x - x2);

                    double crossMag = sqrt(crossX*crossX + crossY*crossY + crossZ*crossZ);
                    if (crossMag < 1e-3){
                        crossMag = 1; 
                        flagK = 1;
                    }

                    double dot1  = (x2 - x1)*(x - x1) + (y2 - y1)*(y - y1) + (z2 - z1)*(z - z1);
                    double dot2  = (x2 - x1)*(x - x2) + (y2 - y1)*(y - y2) + (z2 - z1)*(z - z2);

                    double r1 = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (z - z1)*(z - z1));
                    if (r1 < 1e-6){
                        r1 = 1e-5; 
                    }
                    double r2 = sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2) + (z - z2)*(z - z2));
                    if (r2 < 1e-6){
                        r2 = 1e-5; 
                    }

                    double coef = gamma/(4*PI*crossMag*crossMag);
                    if (flagK == 1){
                        K = 0;
                    }
                    else{
                        K = coef*(dot1/r1 - dot2/r2);
                    }

                    double u_induced = crossX*K;
                    double v_induced = crossY*K;
                    double w_induced = crossZ*K;

                    u_vec[i][0] += u_induced;
                    v_vec[i][0] += v_induced;
                    w_vec[i][0] += w_induced;
                }
            }
}


// Step one - read the input data
int computeInfluenceMatrices(
            int N, // Number of collocation points
            int T,
            double *points_flat, // 1D pointer
            double *table_flat,   // 1D pointer
            double *uInfluence_flat, // 1D pointer
            double *vInfluence_flat, // 1D pointer
            double *wInfluence_flat,  // 1D pointer
            float CORE

        ) {

    // Check if the input data is valid
    if (N <= 0) {
        printf("Error: N must be greater than 0.\n");
        return -1;
    }
    if (points_flat == NULL || table_flat == NULL || uInfluence_flat == NULL || vInfluence_flat == NULL || wInfluence_flat == NULL) {
        printf("Error: Input data cannot be NULL.\n");
        return -1;
    }

    // Treat pointers as 2D arrays
    double (*points)[3] = (double (*)[3]) points_flat;
    double (*table)[8] = (double (*)[8]) table_flat;
    double (*uInfluence)[N] = (double (*)[N]) uInfluence_flat;
    double (*vInfluence)[N] = (double (*)[N]) vInfluence_flat;
    double (*wInfluence)[N] = (double (*)[N]) wInfluence_flat;

    // Print table 
    #pragma omp parallel for
    for (int i = 0; i < N; i++) {
        for (int j = 0; j<T; j++) {
            double x1 = table[j][0];
            double y1 = table[j][1];
            double z1 = table[j][2];
            double x2 = table[j][3];
            double y2 = table[j][4];
            double z2 = table[j][5];
            int flagK = 0; 
            double gamma = table[j][7];
            double K;

            double x = points[i][0];
            double y = points[i][1];
            double z = points[i][2];

            double r1 = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1) + (z - z1)*(z - z1));
            if (r1 < CORE){
                continue;
            }
            double r2 = sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2) + (z - z2)*(z - z2));
            if (r2 < CORE){
                continue;
            }

            double crossX = (y - y1)*(z - z2) - (z - z1)*(y - y2);
            double crossY = (z - z1)*(x - x2) - (x - x1)*(z - z2);
            double crossZ = (x - x1)*(y - y2) - (y - y1)*(x - x2);



            double crossMag = sqrt(crossX*crossX + crossY*crossY + crossZ*crossZ);

            double r_0 = sqrt((x1-x2)*(x1-x2) + (y1- y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

            double d = crossMag/r_0;

            if (d < CORE) {
                continue;
            }

            double dot1  = (x2 - x1)*(x - x1) + (y2 - y1)*(y - y1) + (z2 - z1)*(z - z1);
            double dot2  = (x2 - x1)*(x - x2) + (y2 - y1)*(y - y2) + (z2 - z1)*(z - z2);

            

            
            if (flagK == 1){
                 K = 0;
            }
            else{
                 double coef = gamma/(4*PI*crossMag*crossMag);
                 K = coef*(dot1/r1 - dot2/r2);
            }

            double u_induced = crossX*K;
            double v_induced = crossY*K;
            double w_induced = crossZ*K;


            int horse_index =(int)table[j][6];
            if (horse_index < 0 || horse_index >= N) {
                printf("Invalid horse_index: %d at j=%d\n", horse_index, j);
                continue;
            }

            uInfluence[i][horse_index] += u_induced;
            vInfluence[i][horse_index] += v_induced;
            wInfluence[i][horse_index] += w_induced;
        }
    }
    
    return 0;
}