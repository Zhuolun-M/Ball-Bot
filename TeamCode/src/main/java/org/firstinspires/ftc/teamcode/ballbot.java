package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ballbot {
    //Declared 3 motors to be used
    public DcMotor Motor1;
    public DcMotor Motor2;
    public DcMotor Motor3;

    HardwareMap map;

    public void init(HardwareMap m){
        //Declare variable map, which we map our motor ports to
        map = m;
        Motor1 = m.get(DcMotor.class, "m1");
        Motor2 = m.get(DcMotor.class, "m2");
        Motor3 = m.get(DcMotor.class, "m3");
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

}
