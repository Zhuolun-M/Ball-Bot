package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ballbot2 {
    public static double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix, int r1, int c1, int c2) {
        double[][] product = new double[r1][c2];
        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }

        return product;
    }
    public static double[][] transpose(double A[][], int length)
    {
        for (int i = 0; i < length; i++)
            for (int j = i + 1; j < length; j++) {
                double temp = A[i][j];
                A[i][j] = A[j][i];
                A[j][i] = temp;
            }
        return A;
    }
    ElapsedTime timer_1 = new ElapsedTime();
    public double[] getVector(double pitch, double roll){
        double[] vector = new double[3];
        vector[0] = Math.cos((Math.PI/2) - roll) * Constants.IMUhtBall;
        vector[1] = Math.cos((Math.PI/2) - pitch) * Constants.IMUhtBall;
        vector[2] = Math.sqrt(Constants.IMUhtBall) - (Math.sqrt(vector[1]*vector[1] + vector[0]*vector[0]));
        return vector;
    }
    public double[] Motor_output(double pitch, double roll, double yaw, double xvelocity, double yvelocity, double zvelocity) {
        double[] ballVelocity = new double[3];
        ballVelocity[0] = xvelocity;
        ballVelocity[1] = yvelocity;
        ballVelocity[2] = 0;


        double Construction_Matrix[][] = {{0, -0.139, 0.0139}, {0.0121, 0.0070, 0.0139}, {-0.0121, 0.0070, 0.0139}};
        double Construction_2[][] = {{0, 1, 0}, {-1, 0, 0}, {0, 0, -0.008}};
        double Construction_3[][] = {{0, 1, 0}, {-1, 0, 0}, {0, 0, -119}};
        double Tilt_Matrix[][] = {{Math.cos(pitch), Math.sin(pitch) * Math.sin(roll), -Math.cos(roll) * Math.sin(pitch)},
                                    {0, Math.cos(roll), Math.sin(roll)},
                                    {Math.sin(pitch), -Math.sin(roll) * Math.cos(pitch), Math.cos(pitch) * Math.cos(pitch)}};
        double Kinematic_Matrix[][] = multiplyMatrices(Tilt_Matrix, Construction_3, 3, 3, 3);
        double Velocity_Matrix[] = new double[3];
        Velocity_Matrix[0] = Kinematic_Matrix[0][0] * xvelocity + Kinematic_Matrix[0][1] * yvelocity + Kinematic_Matrix[0][2] * zvelocity;
        Velocity_Matrix[1] = Kinematic_Matrix[1][0] * xvelocity + Kinematic_Matrix[1][1] * yvelocity + Kinematic_Matrix[1][2] * zvelocity;
        Velocity_Matrix[2] = Kinematic_Matrix[2][0] * xvelocity + Kinematic_Matrix[2][1] * yvelocity + Kinematic_Matrix[2][2] * zvelocity;
        return Velocity_Matrix;
    }
}
