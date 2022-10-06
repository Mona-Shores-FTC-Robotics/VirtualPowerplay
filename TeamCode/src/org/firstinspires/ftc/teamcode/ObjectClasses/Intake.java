package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public static final double INTAKE_POWER = 1;

    public boolean intakeState;
    public Servo intake1;
    public Servo intake2;

    public void init(HardwareMap ahwMap) {
        intake1 = ahwMap.servo.get("intake1_servo");
        intake2 = ahwMap.servo.get("intake2_servo");

        //set intake positions
        intake1.setPosition(0);
        intake2.setPosition(0);
        intakeState = false;

    }

    public void toggleIntake() {

        if (intakeState) {
            intake1.setPosition(0);
            intake2.setPosition(0);
            intakeState= false;
        }
        else if(!intakeState)
        {
            intake1.setPosition(1);
            intake2.setPosition(1);
            intakeState= true;
        }
    }
}
