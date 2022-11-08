package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    public static final double INTAKE_POWER = 1;

    public intakeState currentIntakeState;
    public Servo intake1;
    public Servo intake2;

    public enum intakeState {INTAKE_ON, INTAKE_OFF}
    public ElapsedTime afterIntakeOnDelayPeriod = new ElapsedTime();




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
        //When you press and release the button, toggle the intake
        if (currentButtonPress && !previousButtonPress) {
            toggleIntake();
        }
    }

    public void AdvancedCheckIntake(Boolean currentButtonPress, Boolean previousButtonPress) {

        //Keep resetting the delay period as long as the button is pressed
        if (currentButtonPress){
            afterIntakeOnDelayPeriod.reset();
        }

        //When you press and release the button, toggle the intake
        if (currentButtonPress && !previousButtonPress) {
            toggleIntake();
        }
        //When you release the button, reset the delay period one final time after which the intake will automatically toggle
        else if (!currentButtonPress && previousButtonPress){
            afterIntakeOnDelayPeriod.reset();
        }
        //If the intake has been on longer than 1 second after the operator released the button, turn it off
        else if (currentIntakeState == intakeState.INTAKE_ON && afterIntakeOnDelayPeriod.seconds() > 1) {
           toggleIntake();
        }
    }
}
