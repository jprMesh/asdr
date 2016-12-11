#include "kalman_filt.h"

/******************************************************************************/
KalmanFilter::KalmanFilter() {

    /// 12/06/16 MHLI: Jonas if you want you can use memset here, just doing it this way for now.
    int i,j;
    for (i = 0; i < 4; i++) {
        x[i] = 0;
        innovation[i] = 0;
        for (j = 0; j < 4; j++) {
            
            if(i==j){
                A[i][j] = 1;
                P[i][j] = 1; //Should be var_(x or y or vx or vy)
                Q[i][j] = 0.01; //Arbitrarily picked
                H[i][j] = 1; //Arbitrarily picked
                S[i][j] = 1; // Arbitrarily picked
                K[i][j] = 1; //
            }

            else{
                A[i][j] = 0;
                P[i][j] = 0;
                Q[i][j] = 0;
                H[i][j] = 0;
                S[i][j] = 0;
                K[i][j] = 0;
            }
        }
    }
    float x[4]; ///< State, [x;y;Vx;Vy]
    float A[4][4]; ///< Transition matrix, [[1 0 dt 0][0 1 0 dt][ 0 0 1 0][0 0 0 1]]
    float P[4][4]; ///< Covariance. Setting to [[var_x 0 0 0][0 var_y 0 0][ 0 0 var_Vx 0][0 0 0 var_Vy]]
    float Q[4][4]; ///< Process Noise. set to diagonal 0.01.
    float H[4][4]; ///< Sensor noise. Will initialize to identity matrix.
    float innovation[4]; ///< Measurement - Hx
    float S[4][4]; ///< Also initialize to identity matrix.
    float K[4][4]; ///< Kalman Gains. Should probably make constant.
}

/******************************************************************************/
KalmanFilter::~KalmanFilter() {
    
}

/******************************************************************************/
float * KalmanFilter::predict() {

}

/******************************************************************************/
void KalmanFilter::update(float x_in, float y_in, float e_x, float e_y) {

}