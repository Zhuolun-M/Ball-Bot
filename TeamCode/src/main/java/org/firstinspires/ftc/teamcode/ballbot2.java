package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ballbot2 {
    public static int[][] multiplyMatrices(int[][] firstMatrix, int[][] secondMatrix, int r1, int c1, int c2) {
        int[][] product = new int[r1][c2];
        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }

        return product;
    }
    ElapsedTime timer_1 = new ElapsedTime();
    public double[] getVector(double pitch, double roll){
        double[] vector = new double[3];
        vector[0] = Math.cos((Math.PI/2) - roll) * Constants.IMUhtBall;
        vector[1] = Math.cos((Math.PI/2) - pitch) * Constants.IMUhtBall;
        vector[2] = Math.sqrt(Constants.IMUhtBall) - (Math.sqrt(vector[1]*vector[1] + vector[0]*vector[0]));
        return vector;
    }
    public double[] Motor_output(double pitch, double roll, double yaw, double xvelocity, double yvelocity){
        double[] ballVelocity = new double[3];
        ballVelocity[0] = xvelocity;
        ballVelocity[1] = yvelocity;
        ballVelocity[2] = 0;

        double TorqueX = Math.cos((Math.PI / 2) - roll) * Constants.g * (Constants.IMUhtBall/2);
        double TorqueY =  Math.cos((Math.PI / 2) - pitch) * Constants.g * (Constants.IMUhtBall/2);
        double TorqueZ =  Math.cos((Math.PI / 2) - yaw) * Constants.g * (Constants.IMUhtBall/2);
        double frictionTorque = (Constants.Ballr * Constants.Omnir * Constants.xyInertia * Math.sin(yaw));

        double TKyz = 0.5 * Constants.BallMass * Math.pow((Constants.Omnir * roll) , 2) + (0.5 * Constants.BallInertia) * Math.pow(roll,2) ;
        double TKxy = 0.5 * Constants.BallInertia * Math.pow(roll ,2);
        double RotX = (Constants.Ballr / Constants.Omnir) * (roll * 0.25) - roll;
        double TWyz = 0.5 * (Math.pow(Constants.Ballr * roll, 2)) + 2 * (Constants.Ballr + Constants.Omnir) * Math.cos(roll) * roll * Constants.Ballr * roll + Math.pow(Constants.Ballr + Constants.Omnir, 2) * roll * roll + 0.5 * Constants.yzInertia * Math.pow(RotX, 2);
        double TWxy = 0.5 * Constants.xyInertia * Math.pow(RotX, 2);
        double[] TorqueEffect = new double[2];
        TorqueEffect[0] = TorqueX * Constants.Ballr / Constants.Omnir;
        TorqueEffect[1] = -(1 + Constants.Ballr / Constants.Omnir) * TorqueX;
        double[] CtrTorque = new double[2];
        CtrTorque[0] = 0;
        CtrTorque[1] = TorqueX;
        double [] NonPotentialxy = new double[2];
        NonPotentialxy[0] = TorqueEffect[0] + CtrTorque[0];
        NonPotentialxy[1] = TorqueEffect[1] + CtrTorque[1];
        double [] NonPotentialyz = new double[2];
        NonPotentialyz[0] = -frictionTorque + (Constants.Ballr / Constants.Omnir) * Math.sin(yaw) * Constants.Omnir * yaw / 2;
        NonPotentialyz[1] = -(Constants.Ballr / Constants.Omnir) * Math.sin(yaw) * yaw / 2 +  yaw / 2 ;

        double [] FinalVector = new double [3];
        FinalVector[0] = (1/3) * ((TWyz - TKyz) + (TWxy - TKyz) + (2/Math.cos(yaw))*(TorqueX * Math.cos((Math.PI / 2 ) - yaw) - TorqueY * Math.sin((Math.PI / 2 ) - yaw)));
        FinalVector[1] = (1/3) * ((TWyz - TKyz) + (TWxy - TKyz) + (1/Math.cos(yaw))*((Math.sin((Math.PI / 2 )- yaw) * (-Math.sqrt(3)*TorqueX + TorqueY))) - Math.cos((Math.PI / 2 )- yaw)* (TorqueX + Math.sqrt(3)*TorqueY));
        FinalVector[2] = (1/3) * ((TWyz - TKyz) + (TWxy - TKyz) + (1/Math.cos(yaw))*((Math.sin((Math.PI / 2 )- yaw) * (Math.sqrt(3)*TorqueX + TorqueY))) - Math.cos((Math.PI / 2 )- yaw)* (-TorqueX + Math.sqrt(3)*TorqueY));
        return FinalVector;
    }

}
