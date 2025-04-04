#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.141592653589793

// Define struct to hold object properties
typedef struct {
    double x1, y1, z1;
    double x2, y2, z2;
} VelocityParams;

// Function to compute cross product components
double crossX(double x, double y, double z, VelocityParams *params) {
    return (y - params->y1) * (z - params->z2) - (z - params->z1) * (y - params->y2);
}

double crossY(double x, double y, double z, VelocityParams *params) {
    return (z - params->z1) * (x - params->x2) - (x - params->x1) * (z - params->z2);
}

double crossZ(double x, double y, double z, VelocityParams *params) {
    return (x - params->x1) * (y - params->y2) - (y - params->y1) * (x - params->x2);
}

// Main velocity function
void velocity_vectorized(int N, double *x, double *y, double *z, double Gamma, VelocityParams *params, double *output) {
    double *crossMag = (double *)malloc(N * sizeof(double));

    // Compute cross product components
    for (int i = 0; i < N; i++) {
        double cx = crossX(x[i], y[i], z[i], params);
        double cy = crossY(x[i], y[i], z[i], params);
        double cz = crossZ(x[i], y[i], z[i], params);

        // Compute cross magnitude
        crossMag[i] = sqrt(cx * cx + cy * cy + cz * cz);
        if (crossMag[i] < 1e-6) crossMag[i] = 1;

        // Placeholder r1, r2, dot1, dot2 calculations (replace with actual functions)
        double r1 = sqrt(x[i] * x[i] + y[i] * y[i]) + 1e-5;
        double r2 = sqrt(z[i] * z[i] + y[i] * y[i]) + 1e-5;
        double dot1 = x[i] * y[i];
        double dot2 = z[i] * y[i];

        // Compute K factor
        double K = (Gamma / (4 * PI * crossMag[i] * crossMag[i])) * (dot1 / r1 - dot2 / r2);
        if (crossMag[i] < 1e-6) K = 0;

        // Store result in output array
        output[i * 3] = K * cx;
        output[i * 3 + 1] = K * cy;
        output[i * 3 + 2] = K * cz;
    }

    free(crossMag);
}