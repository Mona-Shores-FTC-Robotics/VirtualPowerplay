package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public Boolean coneSensor;
    Claw claw;
    LinearOpMode activeOpMode;

    public Intake(Claw m_claw, LinearOpMode mode) {

        claw = m_claw;
        activeOpMode = mode;
    }

    public void init(HardwareMap ahwMap) {
        intake1 = ahwMap.servo.get("intake1_servo");
        intake2 = ahwMap.servo.get("intake2_servo");

        //make sure intake is off at init
        intake1.setPosition(.5);
        intake2.setPosition(.5);
        currentIntakeState = intakeState.INTAKE_OFF;
    }

    public void turnIntakeOff() {
        intake1.setPosition(.5);
        intake2.setPosition(.5);
        currentIntakeState = intakeState.INTAKE_OFF;
    }

    public void turnIntakeOn() {
        intake1.setPosition(1);
        intake2.setPosition(0);
        currentIntakeState = intakeState.INTAKE_ON;
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
        //When you press the button, turn the intake on
        if (currentButtonPress && !previousButtonPress) {
            claw.setEasyIntake();
            turnIntakeOn();
        }
        //When you release the button, turn the intake off
        else if (!currentButtonPress && previousButtonPress){
            turnIntakeOff();
            claw.closeClaw();
        }
    }

    //If we add a cone sensor, this code should replace the CheckIntake
    public void SuperAdvancedCheckIntake(Boolean currentButtonPress, Boolean previousButtonPress) {
        //When you press and release the button, toggle the intake
        if (currentButtonPress && !previousButtonPress) {
            turnIntakeOn();
        }
        //When you release the button, reset the delay period one final time after which the intake will automatically toggle
        else if ((!currentButtonPress && previousButtonPress) || coneSensor){
            turnIntakeOff();
        }
    }


    public void AutoDeliverIntakeToggle() {
        if (currentIntakeState == intakeState.INTAKE_OFF) {
            intake1.setPosition(1);
            intake2.setPosition(0);
            currentIntakeState = intakeState.INTAKE_ON;
            afterIntakeOnDelayPeriod.reset();
        } else if (currentIntakeState == intakeState.INTAKE_ON && (afterIntakeOnDelayPeriod.seconds() > 1)) {
            intake1.setPosition(.5);
            intake2.setPosition(.5);
            currentIntakeState = intakeState.INTAKE_OFF;
        }
    }

}
