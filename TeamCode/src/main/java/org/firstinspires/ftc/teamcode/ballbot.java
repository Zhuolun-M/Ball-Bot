package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ballbot {
    //Declared 3 motors to be used
    public DcMotor Motor1;
    public DcMotor Motor2;
    public DcMotor Motor3;
    public IMU imu = null;

    ElapsedTime timer = new ElapsedTime();

    //Motor 1 PID values
    public double Int_Sum_1 = 0;
    public double Kp_1 = 0.5;
    public double Ki_1 = 0.2;
    public double Kd_1 = 0.1;
    public double prev_error_1 = 0;

    //Motor 2 PID values
    public double Int_Sum_2 = 0;
    public double Kp_2 = 0.5;
    public double Ki_2 = 0.2;
    public double Kd_2 = 0.1;
    public double prev_error_2 = 0;

    //Motor 3 PID values
    public double Int_Sum_3 = 0;
    public double Kp_3 = 0.5;
    public double Ki_3 = 0.2;
    public double Kd_3 = 0.1;
    public double prev_error_3 = 0;



    HardwareMap map;

    public void init(HardwareMap m){
        //Declare variable map, which we map our motor ports to
        map = m;
        Motor1 = m.get(DcMotor.class, "m1");
        Motor2 = m.get(DcMotor.class, "m2");
        Motor3 = m.get(DcMotor.class, "m3");
        imu = m.get(IMU.class,"imu");
        //Sets initial direction of the motors to forward
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initial power is zero, absence of motion
        Motor1.setPower(0.0);
        Motor2.setPower(0.0);
        Motor3.setPower(0.0);
        //Sets the motors to use encoder values to run the motors for precise adjustment
        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public double PIDctrl_1 (double p, double r){
        double error = p + r;
        double p_to_r = Math.abs(error - Math.abs(p))/Math.abs(error-Math.abs(r));

        Int_Sum_1 += error*timer.seconds();
        double deriv = (error - prev_error_1) / timer.seconds();
        prev_error_1 = error;

        timer.reset();

        double motor_power = Kp_1 * error + Ki_1 * Int_Sum_1 + Kd_1 * deriv;

        return motor_power;
    }
    public double PIDctrl_2 (double p, double r){
        double error = p + r;
        double p_to_r = Math.abs(error - Math.abs(p))/Math.abs(error-Math.abs(r));

        Int_Sum_2 += error*timer.seconds();
        double deriv = (error - prev_error_2) / timer.seconds();
        prev_error_2 = error;

        timer.reset();

        double motor_power = Kp_2 * error + Ki_2 * Int_Sum_2 + Kd_2 * deriv;

        return motor_power;
    }
    public double PIDctrl_3 (double p, double r){
        double error = p + r;
        double p_to_r = Math.abs(error - Math.abs(p))/Math.abs(error-Math.abs(r));

        Int_Sum_3 += error*timer.seconds();
        double deriv = (error - prev_error_3) / timer.seconds();
        prev_error_3 = error;

        timer.reset();

        double motor_power = Kp_3 * error + Ki_3 * Int_Sum_3 + Kd_3 * deriv;

        return motor_power;
    }

}
