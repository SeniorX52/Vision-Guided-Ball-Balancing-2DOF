package com.android.ball_balancing_system;

public class KalmanFilter {
    private Matrix state;       //[x,y,vx,vy]
    private double DT = 0.1;
    private Matrix covariance;
    private final Matrix F;
    private final Matrix H;
    private final Matrix Q;
    private final Matrix R;


    public KalmanFilter(double initialX, double initialY) {
        this(initialX, initialY, 0.01, 0.1); // Calls the new constructor with default values
    }

    // Customizable constructor
    public KalmanFilter(double initialX, double initialY, double qScale, double rScale) {
        this.state = new Matrix(new double[][]{{initialX}, {initialY}, {0}, {0}});
        this.covariance = Matrix.identity(4).times(1000);
        this.F = new Matrix(new double[][]{
                {1, 0, DT, 0},
                {0, 1, 0, DT},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        });
        this.H = new Matrix(new double[][]{
                {1, 0, 0, 0},
                {0, 1, 0, 0}
        });
        this.Q = Matrix.identity(4).times(qScale); // Process noise scaling
        this.R = new Matrix(new double[][]{
                {rScale, 0},
                {0, rScale}
        }); // Measurement noise scaling
    }


    public void enta_shayf_eh() {
        state = F.times(state);
        covariance = F.times(covariance).times(F.transpose()).plus(Q);
    }

    public void khod_faltar(double measuredX, double measuredY) {
        Matrix z = new Matrix(new double[][]{{measuredX}, {measuredY}});
        Matrix y = z.minus(H.times(state));
        Matrix S = H.times(covariance).times(H.transpose()).plus(R);
        Matrix K = covariance.times(H.transpose()).times(S.inverse());
        state = state.plus(K.times(y));
        covariance = (Matrix.identity(4).minus(K.times(H))).times(covariance);
    }

    public double getX() { return state.get(0, 0); }
    public double getY() { return state.get(1, 0); }
    public double getVx() { return state.get(2, 0); }
    public double getVy() { return state.get(3, 0); }
}
