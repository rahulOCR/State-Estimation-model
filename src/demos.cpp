#include<sstream>
#include<vector>
#include<fstream>
#include"kalmanfilter.h"
#include<iostream>
using namespace std;

int main()
{
    Kalmanfilter kp;
    double ans = 0;
    double a[20] = {2,4,8,16,32,64,128,256,512,1024};
    //double a[10] = {2,4,6,8,10,12,14,16,18,20};
    for(int i = 0; i < 10; i++)
    {
        double ans = kp.predict(a[i]);
        cout << a[i] << ": " << ans << endl;
    }
    return 0;
}


