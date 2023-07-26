package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "2-Wheel Balance")
public class BalancebotControl extends LinearOpMode{
    Balancebot robot = new Balancebot();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RevHubOrientationOnRobot.LogoFacingDirection LD = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection UD = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot RO = new RevHubOrientationOnRobot(LD, UD);
        robot.imu.initialize(new IMU.Parameters(RO));

        robot.time1.reset();
        double power = 0.0;
        double setPoint = -0.0;

        robot.imu.resetYaw();

        waitForStart();

        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", LD, UD);

            YawPitchRollAngles ori = robot.imu.getRobotYawPitchRollAngles();
            AngularVelocity ang = robot.imu.getRobotAngularVelocity(AngleUnit.RADIANS);

            if (Math.abs(ori.getPitch(AngleUnit.DEGREES)) > 45){
                robot.LeftMotor.setPower(0.0);
                robot.RightMotor.setPower(0.0);
            }
            else {
                power = robot.PID_Ctrl(setPoint, ori.getPitch(AngleUnit.RADIANS)) + Math.abs(robot.Kf * Math.sin(ori.getPitch(AngleUnit.RADIANS)));
                robot.LeftMotor.setPower(power);
                robot.RightMotor.setPower(power);
            }
            if (power > 1.0){
                power = 1.0;
            }
            else if (power < -1.0){
                power = -1.0;
            }

            telemetry.addData("YAW (Z)", "%.2f Rad.", ori.getYaw(AngleUnit.RADIANS));
            telemetry.addData("PITCH (X)","%.2f Deg.", ori.getPitch(AngleUnit.DEGREES));
            telemetry.addData("PITCH (X) Velocity","%.2f Rad/Sec", ang.xRotationRate);
            telemetry.addData("setPoint", "%.2f", setPoint);
            telemetry.addData("Motor Power", "%.2f", power);
            telemetry.update();

        }
    }
}
