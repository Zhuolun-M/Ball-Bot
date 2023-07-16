package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ballbot {
    //Declared 3 motors to be used
    public DcMotor Motor1;
    public DcMotor Motor2;
    public DcMotor Motor3;
    public IMU imu = null;
    //public BNO055IMU imu2 = null;

    ElapsedTime timer_1 = new ElapsedTime();
    ElapsedTime timer_2 = new ElapsedTime();
    ElapsedTime timer_3 = new ElapsedTime();

    //Motor 1 PID values
    public double Int_Sum_1 = 0;
    public double Kp_1 = 3.0;
    public double Ki_1 = 0.9;
    public double Kd_1 = 0.01;
    public double prev_error_1 = 0;

    //Motor 2 PID values
    public double Int_Sum_2 = 0;
    public double Kp_2 = 3.0;
    public double Ki_2 = 1.0;
    public double Kd_2 = 0.01;
    public double prev_error_2 = 0;

    //Motor 3 PID values
    public double Int_Sum_3 = 0;
    public double Kp_3 = 3.0;
    public double Ki_3 = 1.0;
    public double Kd_3 = 0.01;
    public double prev_error_3 = 0;

    public double a = 0.8;
    public double prevFilter = 0.0;
    public double currFilter = 0.0;
    public double max_int_sum = 0.25;



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
        Motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initial power is zero, absence of motion
        Motor1.setPower(0.0);
        Motor2.setPower(0.0);
        Motor3.setPower(0.0);
        //Sets the motors to without encoders
        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        BNO055IMU.Parameters parm = new BNO055IMU.Parameters();
        parm.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parm.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parm.calibrationDataFile = "BNO055IMUCalibration.json";
        parm.loggingEnabled = true;
        parm.loggingTag = "IMU";
        parm.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu2.initialize(parm);
        */
    }
    public double PIDctrl (double tp, double tr, double p, double r, double Kp, double Ki, double Kd, double Int_Sum, double prev_error, ElapsedTime timer){
        double error = (tr - r);
        double p_to_r = Math.abs(error - Math.abs(p))/Math.abs(error-Math.abs(r));

        Int_Sum += error*timer.seconds();
        double deriv = (error - prev_error) / timer.seconds();
        prev_error = error;
        timer.reset();
        double motor_power = Kp * error + Ki * Int_Sum + Kd * deriv;

        return motor_power;
    }

    public double PIDctrl_1 (double tp, double tr, double p, double r, double Kp, double Ki, double Kd, double Int_Sum, double prev_error, ElapsedTime timer){
        double error = (tp - p);
        double p_to_r = Math.abs(error - Math.abs(p))/Math.abs(error-Math.abs(r));
        double error_change = error - prev_error;

        currFilter = (a * prev_error) + (1-a) * error_change;
        prevFilter = currFilter;

        Int_Sum += error*timer.seconds();
        double deriv = currFilter / timer.seconds();
        if (Int_Sum > max_int_sum){
            Int_Sum = max_int_sum;
        }
        if (Int_Sum < -max_int_sum){
            Int_Sum = -max_int_sum;
        }
        timer.reset();

        double motor_power = Kp * error + Ki * Int_Sum + Kd * deriv;
        prev_error = error;
        return motor_power;
    }


}