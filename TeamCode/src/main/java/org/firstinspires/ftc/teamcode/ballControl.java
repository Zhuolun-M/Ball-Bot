package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    // Using the ballbot class we will create a variable we will use later.
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // This runs the init method from ballbot onto robot so the variables are no longer null values
        //        //Here we declare two  variables
        //        // that the orientation of the hub is facing upward as shown by the logo and that the direction of the
        // USB port is facing forward.
        RevHubOrientationOnRobot.LogoFacingDirection LD = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection UD = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot RO = new RevHubOrientationOnRobot(LD, UD);
        robot.imu.initialize(new IMU.Parameters(RO));
        // Here we initialize the variable imu by using the RO variable
        robot.timer_1.reset();
        robot.timer_2.reset();
        robot.timer_3.reset();
        double power_1 = 0.0;
        double power_2 = 0.0;
        double power_3 = 0.0;
        ElapsedTime timer = new ElapsedTime();
        double prevx = 0;
        double prevy = 0;
        waitForStart();
        // We will wait for the drivestation to be activated
        robot.imu.resetYaw();
        //Each time we start our robot we will reset the YAW to make sure that where the robot starts
        // that location becomes the new origin
        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", LD, UD);
            // This telemetry tells the drive the orientation the Hub is supposed to be in.
            YawPitchRollAngles ori = robot.imu.getRobotYawPitchRollAngles();
            // The YawPitchRollAngles class is extended from the IMU class in order to contain the readings of
            // Yaw, Pitch, Roll, and Angles of the IMU sensor in the hub.
            // We will create a variable called ori to access those readings.
            AngularVelocity ang = robot.imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            // This line will instead contain the Angular Velocities of the change in Yaw, Pitch, and Roll,
            // instead of the values themselves
            /*if(gamepad1.a){
                robot.Motor1.setPower(1.0);
            }
            else{
                robot.Motor1.setPower(0.0);
            }*/
            /*if(robot.timer.seconds() > 0.1){
                robot.timer.reset();
                robot.Int_Sum_1 = 0;
                robot.Int_Sum_2 = 0;
                robot.Int_Sum_3 = 0;
            }*/
            if (timer.seconds() > 0.1){
                timer.reset();
            }
            methv = methods.getVector(ori.getPitch(AngleUnit.RADIANS), ori.getRoll(AngleUnit.RADIANS), timer, prevx, prevy);
            meth = methods.Motor_output(ori.getPitch(AngleUnit.RADIANS), ori.getRoll(AngleUnit.RADIANS), ori.getYaw(AngleUnit.RADIANS), methv[0], methv[1], ang.zRotationRate);
            if (gamepad1.a){
                robot.Motor1.setPower(0.0);
                robot.Motor2.setPower(0.0);
                robot.Motor3.setPower(0.0);
            }
            else {
                power_1 = meth[0] * 0.05;
                        //* robot.PIDctrl(0, 0, ori.getPitch(AngleUnit.RADIANS), ori.getRoll(AngleUnit.RADIANS), robot.Kp_1, robot.Ki_1, robot.Kd_1, robot.Int_Sum_1, robot.prev_error_1, robot.timer_1);
                power_2 = meth[1] * 0.05;
                //* robot.PIDctrl(0, 0, ori.getPitch(AngleUnit.RADIANS), ori.getRoll(AngleUnit.RADIANS), robot.Kp_2, robot.Ki_2, robot.Kd_2, robot.Int_Sum_2, robot.prev_error_2, robot.timer_2);
                power_3 = meth[2] * 0.05;
                //* robot.PIDctrl(0, 0, ori.getPitch(AngleUnit.RADIANS), ori.getRoll(AngleUnit.RADIANS), robot.Kp_3, robot.Ki_3, robot.Kd_3, robot.Int_Sum_3, robot.prev_error_3, robot.timer_3);
                robot.Motor1.setPower(power_1);
                robot.Motor2.setPower(power_2);
                robot.Motor3.setPower(power_3);
            }
            telemetry.addData("YAW (Z)", "%.2f Rad.", ori.getYaw(AngleUnit.RADIANS));
            telemetry.addData("PITCH (X)","%.2f Rad.", ori.getPitch(AngleUnit.RADIANS));
            telemetry.addData("ROLL (Y)", "%.2f Rad.", ori.getRoll(AngleUnit.RADIANS));
            telemetry.addData("YAW (Z) Velocity", "%.2f Rad/Sec", ang.zRotationRate);
            telemetry.addData("PITCH (X) Velocity","%.2f Rad/Sec", ang.xRotationRate);
            telemetry.addData("ROLL (Y) Velocity", "%.2f Rad/Sec", ang.yRotationRate);
            telemetry.addData("Motor1 Power", "%.2f", power_1);
            telemetry.addData("Motor2 Power", "%.2f",power_2);
            telemetry.addData("Motor3 Power", "%.2f",power_3);
            telemetry.update();
            // We will add Data to the telemetry on the values of the Yaw, Pitch, Roll, and their respective angular velocites
            // We will also update all the telemtry at the same time for the driver

            sleep(20);
            // Updates the code every 0.005 seconds
            // Updates the code every 0.02 seconds
            //robot.timer_1.reset();

        }
    }
}