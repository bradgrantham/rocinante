#include <cstdio>
int main()
{
    float m = -1;
    for(float r1 = 10; r1 < 100 * 1000; r1 *= 1.1) {
        for(float r2 = 10; r2 < 100 * 1000; r2 *= 1.1) {
            float a = 3.3 * (75 + r2) / (75 + r2 + 15000 + r1);
            float b = 3.3 * (r2) / (75 + r2 + 15000 + r1);
            float diff = a - b;
            if(diff > m) {
                m = diff;
                printf("%f, %f, %f, %f, %f\n", r1, r2, a, b, a - b);
            }
        }
    }
}
