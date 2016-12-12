#include "kalman_filt.h"

/******************************************************************************/
KalmanFilter::KalmanFilter() {

    /// 12/06/16 MHLI: Jonas if you want you can use memset here, just doing it this way for now.
    int i,j;
    for (i = 0; i < 4; i++) {
        x[i] = 0;
        innovation[i] = 0;
        for (j = 0; j < 4; j++) {
         
            if(i == j)
                H[i][j] = 1;
            else
                H[i][j] = 0;

                A[i][j] = 0;
                P[i][j] = 0;
                Q[i][j] = 0;
                S[i][j] = 0;
                K[i][j] = 0;
                R[i][j] = 0;
            
        }
    }
    
    //A: Constant V model (sampling fast enough for it)
    A[0][0] = 1;
    A[0][2] = 0.001; //dt
    A[1][1] = 1;
    A[1][3] = 0.001; //dt
    A[2][2] = 1;
    A[3][3] = 1;    

    //P: Arbitrarily picked rn, get value from gps.
    P[0][0] = 2; //Var dx
    P[1][1] = 2; //Var dy
    P[2][2] = 4; //var vx
    P[3][3] = 4; //var vy

    //Q: Assumed that the noise is white.
    //Q = q*[1/4*dt^4 0 0.5*dt^3 0; 
    //      0 1/4*dt^4 0 0.5*dt^3; 
    //      0.5*dt^3 0 dt^2 0; 
    //      0 0.5*dt^3 0 dt^2];

    float q = 1; //Constant
    Q[0][0] = q * 2.5e-13;
    Q[0][2] = q * 5e-10;
    Q[1][1] = q * 2.5e-13;
    Q[1][3] = q * 5e-10;
    Q[2][0] = q * 5e-10;
    Q[2][2] = q * 1e-6;
    Q[3][1] = q * 5e-10;
    Q[3][3] = q * 1e-6;

    //Calculated from matlab trial, may need modificaions.
    K[0][0] = 0.625;
    K[0][2] = -750.002;
    K[1][1] = 0.625;
    K[1][3] = -750.002;
    K[2][0] = 4.375e-4;
    K[2][2] = 1.875;
    K[3][1] = 4.375e-4;
    K[3][3] = 1.875;

   }

/******************************************************************************/
KalmanFilter::~KalmanFilter() {
    
}

/******************************************************************************/
void KalmanFilter::predict() {
    float tempx[4] = {0,0,0,0};
    float tempP[4][4] = {0};
    int i, j, k;



    // X = A*X
    for(i = 0; i < 4; i++) {
        for (j = 0; j< 4; j++){
            tempx[i] += A[i][j] * x[j];
        }
    }
    //Maybe changable to pointer change. (x = tempx)
    for(i = 0; i < 4; i++) {
        x[i] = tempx[i];
    }
    
    //P = F*P*Ft + Q
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            for(k = 0; k < 4; k++){
                tempP[i][j] += A[i][k] * P[k][j] *A[k][i];
            }
            tempP[i][j] += Q[i][j];
        }
    }

}

/******************************************************************************/
void KalmanFilter::update(float pos_x, float pos_y, float e_x, float e_y) {
   float temp[4][4] = {0};
   float mid[4][4] = {0};
   float temp_y[4] = {0};
   float hph,rs;
   int i, j, k;

   //Y = M-Hx 
    innovation[0] = pos_x - x[0];
    innovation[1] = pos_y - x[1];
    innovation[2] = (pos_x - pos_x_last) - x[2];
    innovation[3] = (pos_y - pos_y_last) - x[3];
    //Save current location
    pos_x_last = pos_x;
    pos_y_last = pos_y; 


    // IF K is constant, this isn't necessary.
    //S = HPH^T + RS
    // for(i = 0; i < 4; i++) {
    //     for(j = 0; j < 4; j++) {
    //         hph = 0; 
    //         rs = 0;
    //         for(k = 0; k < 4; k++){
    //             hph += H[i][k] * P[k][j] *H[k][i];
    //             rs += R[i][k] * S[k][j];
    //         }
    //         temp[i][j] = hph + rs;
    //     }
    // }
    // for(i=0;i<4;i++){
    //     for(j=0;j<4;j++){
    //         S[i][j] = temp[i][j];
    //     }
    // }
    //K=PH^TS^-1K //K will be set to a constant

    //X = x+KY
    for(i = 0; i < 4; i++) {
        for (j = 0; j< 4; j++){
            temp_y[i] += K[i][j] * innovation[j];
        }
        x[i] = x[i] + temp_y[i];
    }

    // Mid = I-KH
    for(i = 0; i < 4; i++) {
        for(j = 0; j < 4; j++) {
            for(k = 0; k < 4; k++){
                mid[i][j] += K[i][k] * H[k][j];
            }
            //Incorporate identity matrix here
            if(i==j)
                mid[i][j] = 1-mid[i][j];
            else
                mid[i][j] = 0-mid[i][j];
        }
    }
    
    //P = Mid*P
    for(i = 0; i < 4; i++) {
        for(j = 0; j<4;j++){
            temp[i][j] = 0;
            for(k=0;k<4;k++){
                temp[i][j] += mid[i][k] * P[k][j];
            }
        }
    }

    //Maybe can just be done with pointer change.
    for(i = 0; i < 4; i++) { 
        for(j = 0; j < 4; j++){
            P[i][j] = temp[i][j];
        }
    }

}
/******************************************************************************/
float * KalmanFilter::get_state() {
 return x;
}