package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    public static final double ARM_INTAKE = 0.5;
    public static final double ARM_LEFT_OUTTAKE = 0;
    public static final double ARM_RIGHT_OUTTAKE = 1.0;
    public Servo arm;

    public void init(HardwareMap ahwMap) {
        arm = ahwMap.servo.get("turret_servo");

        //set arm at intake position
        arm.setPosition(ARM_INTAKE);
    }

    public void setPosition(double position) {
        arm.setPosition(position);
    }

    public void moveArm(LinearOpMode activeOpMode, boolean operatorY, boolean operatorB, boolean operatorX){
        if(operatorY) {
            setPosition(ARM_INTAKE);
        }
        else if (operatorB){
            setPosition(ARM_RIGHT_OUTTAKE);
        }
        else if (operatorX){
            setPosition(ARM_LEFT_OUTTAKE);
        }
    }
}
