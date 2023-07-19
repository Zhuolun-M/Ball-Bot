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

    public double Kp = 8.0;
    public double Ki = 0.0;
    public double Kd = 0.0;
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

        Int_sum += error*time1.seconds();
        double deriv = (error - prev_error) / time1.seconds();
        prev_error = error;
        time1.reset();

        double motor_power = Kp * error + Ki * Int_sum + Kd * deriv;
        return motor_power;
    }

}
