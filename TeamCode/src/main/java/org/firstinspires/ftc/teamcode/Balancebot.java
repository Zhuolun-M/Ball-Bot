package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Balancebot {

    public DcMotor LeftMotor;
    public DcMotor RightMotor;
    public IMU imu;
    ElapsedTime time1 = new ElapsedTime();
//2.75, 2.416, 1.5, 2.5, 3.0, 2.85, 2.8, 2.6, 2.55, 2.65, 2.4, 2.5, 2.3, 2.45, 2.3
    public double Kp = 2.75;
    // try 2.35 tomorrow
    public double Ki = 5.95;
    //4.55, 0.28, 0.2, 0.26, 0.25, 0.255, 5.75
    public double Kd = 0.1055;
    //0.0635, 0.775, 0.0995
    public double Kf = 0.75;
    public double Int_sum = 0.0;
    public double prev_error = 0.0;

    HardwareMap maps;

    public void init(HardwareMap m){
        maps = m;
        LeftMotor = m.get(DcMotor.class, "lf");
        RightMotor = m.get(DcMotor.class,"rg");
        imu = m.get(IMU.class, "imu");

        LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftMotor.setPower(0.0);
        RightMotor.setPower(0.0);

        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double PID_Ctrl(double tp, double p){
        double error = (tp - p);

        Int_sum += error * time1.seconds();
        double deriv = (error - prev_error)/time1.seconds();
        prev_error = error;
        time1.reset();

        double motor_power = Kp * error + Ki * Int_sum + Kd * deriv;
        return motor_power;
    }

}
