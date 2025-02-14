#include <stdio.h>
#include <math.h>

// Function to compute a 1x3 result from two 3D points
void compute_3D_vector(double x1, double y1, double z1, 
                        double x2, double y2, double z2, 
                        double result[3]) {
    result[0] = x2 - x1;  // Example: Compute difference in X
    result[1] = y2 - y1;  // Example: Compute difference in Y
    result[2] = z2 - z1;  // Example: Compute difference in Z
}