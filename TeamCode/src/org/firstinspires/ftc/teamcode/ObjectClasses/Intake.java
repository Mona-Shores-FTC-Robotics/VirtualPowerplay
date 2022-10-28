package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public static final double INTAKE_POWER = 1;

    public intakeState currentIntakeState;
    public Servo intake1;
    public Servo intake2;

    public enum intakeState {INTAKE_ON, INTAKE_OFF}

    public void init(HardwareMap ahwMap) {
        intake1 = ahwMap.servo.get("intake1_servo");
        intake2 = ahwMap.servo.get("intake2_servo");

        //make sure intake is off at init
        intake1.setPosition(.5);
        intake2.setPosition(.5);
        currentIntakeState = intakeState.INTAKE_OFF;
    }

    public void toggleIntake() {

        if (currentIntakeState == intakeState.INTAKE_ON) {
            intake1.setPosition(.5);
            intake2.setPosition(.5);
            currentIntakeState = intakeState.INTAKE_OFF;
        } else if (currentIntakeState == intakeState.INTAKE_OFF) {
            intake1.setPosition(1);
            intake2.setPosition(0);
            currentIntakeState = intakeState.INTAKE_ON;
        }
    }

    public void CheckIntake(Boolean currentButtonPress, Boolean previousButtonPress) {
        if (currentButtonPress && !previousButtonPress) {
            toggleIntake();
        }
    }
}
