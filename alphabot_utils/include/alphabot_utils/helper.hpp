#define M_PI 3.14159265359

double to_radians(double theta) {
    return M_PI * theta / 180.0;

}

double to_degrees(double theta) {
    return theta * 180.0 / M_PI;
}
