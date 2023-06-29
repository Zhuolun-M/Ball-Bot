package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "LiveReadingOfIMU", group = "Sensor")
public class readIMU extends LinearOpMode {
    ballbot robot = new ballbot();
    // We name an IMU class IMU which we will then initialize later.

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // This calls the IMU of that is located on the expansion/control hub which should be
        // named IMU.
        //Here we declare two  variables
        // that the orientation of the hub is facing upward as shown by the logo and that the direction of the
        // USB port is facing forward.
        RevHubOrientationOnRobot.LogoFacingDirection LD = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection UD = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot RO = new RevHubOrientationOnRobot(LD, UD);

        robot.imu.initialize(new IMU.Parameters(RO));
        // Here we initialize the variable imu by using the RO variable

        waitForStart();
        // We will wait for the drivestation to be activated

        robot.imu.resetYaw();
        //Each time we start our robot we will reset the YAW to make sure that where the robot starts
        // that location becomes the new origin


        while(opModeIsActive()){
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", LD, UD);
            // This telemetry tells the drive the orientation the Hub is supposed to be in.

            YawPitchRollAngles ori = robot.imu.getRobotYawPitchRollAngles();
            // The YawPitchRollAngles class is extended from the IMU class in order to contain the readings of
            // Yaw, Pitch, Roll, and Angles of the IMU sensor in the hub.
            // We will create a variable called ori to access those readings.
            AngularVelocity ang = robot.imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            // This line will instead contain the Angular Velocities of the change in Yaw, Pitch, and Roll,
            // instead of the values themselves

            if(gamepad1.a){
                robot.Motor1.setPower(1.0);
            }
            else{
                robot.Motor1.setPower(0.0);
            }

            telemetry.addData("YAW (Z)", "%.2f Deg.", ori.getYaw(AngleUnit.DEGREES));
            telemetry.addData("PITCH (X)","%.2f Deg.", ori.getPitch(AngleUnit.DEGREES));
            telemetry.addData("ROLL (Y)", "%.2f Deg.", ori.getRoll(AngleUnit.DEGREES));
            telemetry.addData("YAW (Z) Velocity", "%.2f Deg/Sec", ang.zRotationRate);
            telemetry.addData("PITCH (X) Velocity","%.2f Deg/Sec", ang.xRotationRate);
            telemetry.addData("ROLL (Y) Velocity", "%.2f Deg/Sec", ang.yRotationRate);
            telemetry.update();
            // We will add Data to the telemetry on the values of the Yaw, Pitch, Roll, and their respective angular velocites
            // We will also update all the telemtry at the same time for the driver

            sleep(250);
            // Updates the code every 0.25 seconds
        }
    }
}
