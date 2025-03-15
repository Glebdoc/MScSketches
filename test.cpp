#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <iomanip>
#include <cstring>


std::vector<std::vector<double>> readCSV(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::vector<double> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                row.push_back(std::stod(cell));
            }
            data.push_back(row);
        }
        file.close();
    } else {
        std::cout << "Failed to open file: " << filename << std::endl;
    }
    return data;
}

void saveToFile(const double* data, int rows, int cols, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile) {
        std::cerr << "Error opening file for writing.\n";
        return;
    }

    outFile.write(reinterpret_cast<const char*>(data), rows * cols * sizeof(double));
    outFile.close();
}


void readAndPrintFile(const std::string& filename, int rows, int cols) {
    std::ifstream inFile(filename, std::ios::binary);
    if (!inFile) {
        std::cerr << "Error opening file for reading.\n";
        return;
    }

    double* data = new double[rows * cols]; // Allocate space for the matrix

    inFile.read(reinterpret_cast<char*>(data), rows * cols * sizeof(double));
    inFile.close();

    // Print the data in a readable format
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::cout << std::fixed << std::setprecision(3) << data[i * cols + j] << " ";
        }
        std::cout << "\n";
    }

    delete[] data; // Clean up
}



class Point {

public:
    double x, y, z;
    // constructors:

    // default constructor
    Point() : x(0), y(0), z(0) {}
    // parameterized constructor
    Point(double xVal, double yVal, double zVal) : x(xVal), y(yVal), z(zVal) {}

    void translate(double dx, double dy, double dz) {
        x += dx;
        y += dy;
        z += dz;
    }
};

class Vortex {
private:
    

public:
double Gamma;
    Point p1;
    Point p2;
    int hsN;
    Vortex(const Point& point1, const Point& point2, double GammaVal, int hs)
        : p1(point1), p2(point2), Gamma(GammaVal), hsN(hs) {}

    double crossX(Point externalPoint) {
        double val = (externalPoint.y - p1.y) * (externalPoint.z - p2.z) - (externalPoint.z - p1.z) * (externalPoint.y - p2.y);
        return val;
    }
    double crossY(Point externalPoint) {
        double val = (externalPoint.z - p1.z) * (externalPoint.x - p2.x) - (externalPoint.x - p1.x) * (externalPoint.z - p2.z);
        return val;
    }
    double crossZ(Point externalPoint) {
        double val = (externalPoint.x - p1.x) * (externalPoint.y - p2.y) - (externalPoint.y - p1.y) * (externalPoint.x - p2.x);
        return val;
    }
    double dot1(Point externalPoint) {
        double val = (p2.x - p1.x) * (externalPoint.x - p1.x) + (p2.y - p1.y) * (externalPoint.y - p1.y) + (p2.z - p1.z) * (externalPoint.z - p1.z);
        return val;
    }
    double dot2(Point externalPoint) {
        double val = (p2.x - p1.x) * (externalPoint.x - p2.x) + (p2.y - p1.y) * (externalPoint.y - p2.y) + (p2.z - p1.z) * (externalPoint.z - p2.z);
        return val;
    }
    double r1(Point externalPoint) {
        double val = sqrt(pow(externalPoint.x - p1.x, 2) + pow(externalPoint.y - p1.y, 2) + pow(externalPoint.z - p1.z, 2));
        return val;
    }
    double r2(Point externalPoint) {
        double val = sqrt(pow(externalPoint.x - p2.x, 2) + pow(externalPoint.y - p2.y, 2) + pow(externalPoint.z - p2.z, 2));
        return val;
    }

    void computeVelocity(Point* externalPoints, double (&velocities)[][3], int numPoints, bool reset = false) {
        for (int i = 0; i < numPoints; i++) {
            double crossXVal = crossX(externalPoints[i]);
            double crossYVal = crossY(externalPoints[i]);
            double crossZVal = crossZ(externalPoints[i]);
            double crossVector[3] = { crossXVal, crossYVal, crossZVal };
            double crossMagnitude = sqrt(pow(crossXVal, 2) + pow(crossYVal, 2) + pow(crossZVal, 2));
            double r1Val = r1(externalPoints[i]);
            double r2Val = r2(externalPoints[i]);
            double dot1Val = dot1(externalPoints[i]);
            double dot2Val = dot2(externalPoints[i]);
            double K = Gamma / (4 * M_PI*crossMagnitude*crossMagnitude)*(dot1Val / r1Val - dot2Val / r2Val);

            velocities[i][0] += K * crossVector[0];
            velocities[i][1] += K * crossVector[1];
            velocities[i][2] += K * crossVector[2];

        }
    }

    Point getP1() const { return p1; }
    Point getP2() const { return p2; }
};


int main() {
// read a txt with vortices
std::string filename = "testVortex.csv";
std::vector<std::vector<double>> data = readCSV(filename);

// read a txt with collocation points
std::string filename2 = "testCoordinates.csv";
std::vector<std::vector<double>> collocation_points = readCSV(filename2);

int numPoints = collocation_points.size();
int numVortices = data.size();

// define collocation point array 
Point collocation_points_array[numPoints];

// fill the collocation point array with the collocation points
for (int i = 0; i < numPoints; i++) {
    collocation_points_array[i] = Point(collocation_points[i][0], collocation_points[i][1], collocation_points[i][2]);
}

// create the corresponding arrays for the velocities
double velocities[numPoints][3] = {0};

// define the influence matrices
double uInfluence[numPoints][numPoints];
double vInfluence[numPoints][numPoints];
double wInfluence[numPoints][numPoints];

int hsN = 0;
bool reset = false;

for (int i = 0; i < numVortices; i++) {
    Vortex vortex = Vortex(Point(data[i][0], data[i][1], data[i][2]), Point(data[i][3], data[i][4], data[i][5]), 1, (int)data[i][6]);
    if (vortex.hsN > hsN ) {
        hsN = vortex.hsN;
        for (int j = 0; j < numPoints; j++) {
            uInfluence[hsN-1][j] = velocities[j][0];
            vInfluence[hsN-1][j] = velocities[j][1];
            wInfluence[hsN-1][j] = velocities[j][2];
            }
        std::memset(velocities, 0, sizeof(velocities[0][0]) * numPoints * 3);
        vortex.computeVelocity(collocation_points_array, velocities, numPoints, reset);
    }
    else {
        vortex.computeVelocity(collocation_points_array, velocities, numPoints, reset);
    }

    if (i == numVortices - 1) {
        for (int j = 0; j < numPoints; j++) {
            uInfluence[hsN][j] = velocities[j][0];
            vInfluence[hsN][j] = velocities[j][1];
            wInfluence[hsN][j] = velocities[j][2];
        }
    }
}

    printf("Influence u \n");
    for (int i = 0; i<numPoints; i++){
        for (int j = 0; j<numPoints; j++){
            printf("%2f", uInfluence[i][j]);
        }
            printf("\n");
    }
    printf("Influence v \n");
    for (int i = 0; i<numPoints; i++){
        for (int j = 0; j<numPoints; j++){
            printf("%2f", vInfluence[i][j]);
        }
            printf("\n");
    }
    printf("Influence w \n");
    for (int i = 0; i<numPoints; i++){
        for (int j = 0; j<numPoints; j++){
            printf("%2f  ", wInfluence[i][j]);
        }
            printf("\n");
    }

    saveToFile(&uInfluence[0][0], numPoints, numPoints, "matrix.bin");

    return 0;
}