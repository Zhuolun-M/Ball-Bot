package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Autonomous(name = "BallBotControl", group = "Sensor")
public class ballControl extends LinearOpMode {
    ballbot robot = new ballbot();
    ballbot2 methods = new ballbot2();
    double[] meth = new double[3];
    double[] methv = new double[3];
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // This runs the init method from ballbot onto robot so the variables are no longer null values
        //Here we declare two  variables that the orientation of the hub is facing upward as shown by the logo and
        // that the direction of the USB port is facing forward.
        RevHubOrientationOnRobot.LogoFacingDirection LD = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection UD = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot RO = new RevHubOrientationOnRobot(LD, UD);
        robot.imu.initialize(new IMU.Parameters(RO));
        // Here we initialize the variable imu by using the RO variable
        robot.timer_1.reset();
        robot.timer_2.reset();
        robot.timer_3.reset();
        double ticks_1 = 0.0;
        double ticks_2 = 0.0;
        double ticks_3 = 0.0;
        ElapsedTime timer = new ElapsedTime();
        double prevx = 0;
        double prevy = 0;
        waitForStart();
        // We will wait for the drive station to be activated
        robot.imu.resetYaw();
        //Each time we start our robot we will reset the YAW to make sure that where the robot starts
        // that location becomes the new origin
        while(opModeIsActive()){
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", LD, UD);
            // This telemetry tells the drive the orientation the Hub is supposed to be in.
            YawPitchRollAngles ori = robot.imu.getRobotYawPitchRollAngles();
            AngularVelocity ang = robot.imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (timer.seconds() > 0.1){
                timer.reset();
            }
            methv = methods.getVector(ori.getPitch(AngleUnit.RADIANS), ori.getRoll(AngleUnit.RADIANS), timer, prevx, prevy);
            meth = methods.Motor_output(ori.getPitch(AngleUnit.RADIANS), ori.getRoll(AngleUnit.RADIANS), ori.getYaw(AngleUnit.RADIANS), methv[0], methv[1], ang.zRotationRate);
            if (gamepad1.a){
                robot.Motor1.setTargetPosition(0);
                robot.Motor2.setTargetPosition(0);
                robot.Motor3.setTargetPosition(0);
            }
            else {
                robot.Motor1.setTargetPosition((int)ticks_1);
                robot.Motor2.setTargetPosition((int)ticks_1);
                robot.Motor3.setTargetPosition((int)ticks_1);
                ticks_1 = meth[0] * Constants.ticksperrev;
                ticks_2 = meth[1] * Constants.ticksperrev;
                ticks_3 = meth[2] * Constants.ticksperrev;
            }
            telemetry.addData("YAW (Z)", "%.2f Rad.", ori.getYaw(AngleUnit.RADIANS));
            telemetry.addData("PITCH (X)","%.2f Rad.", ori.getPitch(AngleUnit.RADIANS));
            telemetry.addData("ROLL (Y)", "%.2f Rad.", ori.getRoll(AngleUnit.RADIANS));
            telemetry.addData("YAW (Z) Velocity", "%.2f Rad/Sec", ang.zRotationRate);
            telemetry.addData("PITCH (X) Velocity","%.2f Rad/Sec", ang.xRotationRate);
            telemetry.addData("ROLL (Y) Velocity", "%.2f Rad/Sec", ang.yRotationRate);
            telemetry.addData("Motor1 ticks", "%.2f", ticks_1);
            telemetry.addData("Motor2 ticks", "%.2f", ticks_2);
            telemetry.addData("Motor3 ticks", "%.2f", ticks_3);
            telemetry.update();
            // We will add Data to the telemetry on the values of the Yaw, Pitch, Roll, and their respective angular velocites
            // We will also update all the telemetry at the same time for the driver

            sleep(5);
            // Updates the code every 0.005 seconds, or, 200 times per second

        }
    }
}