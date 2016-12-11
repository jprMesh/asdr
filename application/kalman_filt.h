#ifndef __kf_h__
#define __kf_h__

class KalmanFilter {
public:
    /**
     * @brief Initialize a KF object.
     */
    KalmanFilter();

    /**
     * @brief Stop all functionality and destroy KF object.
     */
    ~KalmanFilter();

    /**
     * @brief Given current state, outputs a buffer with filtered result.
     */
    void predict();


    /**
     * @brief Updates filter
     */
    void update(float,float,float,float);
    
    /**
     * @brief Return current state
     */
    float * get_state();

private:

    float x[4]; ///< State, [x;y;Vx;Vy]
    float A[4][4]; ///< Transition matrix, [[1 0 dt 0][0 1 0 dt][ 0 0 1 0][0 0 0 1]]
    float P[4][4]; ///< Covariance. Setting to [[var_x 0 0 0][0 var_y 0 0][ 0 0 var_Vx 0][0 0 0 var_Vy]]
    float R[4][4]; ///< Sensor Noise
    float Q[4][4]; ///< Process Noise. set to diagonal 0.01.
    float H[4][4]; ///< rolls forward, initialized to I
    float innovation[4]; ///< Measurement - Hx
    float S[4][4]; ///< Also initialize to identity matrix.
    float K[4][4]; ///< Kalman Gains. Should probably make constant.

    float pos_x_last; ///< Xpos, saved for velocity calc
    float pos_y_last; ///< ypos, saved for velocity calc
};

#endif //__kf_H__