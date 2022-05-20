
class Kalmanfilter
{
    private:
        const double R = 40; //noise covariance (it's 10 initially)
        const double H = 1.0; //measurement map scaler
        double Q = 10; //initial estimated covariance
        double P = 0; // initial error covariance
        double predicted = 0; // initial estimated state 
        double k = 0; //initial kalman gain


    public:

        double predict(double measured)
            {
                k = P * H / (H * P * H + R); // update kalman gain
                
                predicted = predicted + k * (measured - H * predicted); // update estimated state

                P = (1 - k * H) * P + Q; // update error covariance

                return predicted;

            }



};
