package org.firstinspires.ftc.teamcode.ObjectClasses;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    public static final double CLAW_OPEN_POWER = .32;
    public static final double CLAW_CLOSED_POWER = .75;
    public static final double CLAW_EASY_INTAKE = .5;
    public Servo claw;
    public enum clawStates {CLAW_OPEN, CLAW_CLOSED, CLAW_EASY_INTAKE}
    public clawStates currentClawState;

    public ElapsedTime afterClawOpensDelayPeriod = new ElapsedTime();




    public void init(HardwareMap ahwMap) {
        claw = ahwMap.servo.get("claw_servo");

        //set arm at intake position
        claw.setPosition(CLAW_CLOSED_POWER);
        currentClawState = clawStates.CLAW_CLOSED;
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POWER);
        currentClawState = clawStates.CLAW_CLOSED;
    }

    public void setEasyIntake() {
        claw.setPosition(CLAW_EASY_INTAKE);
        currentClawState = clawStates.CLAW_EASY_INTAKE;
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POWER);
        currentClawState = clawStates.CLAW_OPEN;
    }

    //Toggles claw on and off depending on currentClawState
    public void toggleClaw() {
        if (currentClawState == clawStates.CLAW_OPEN || currentClawState == clawStates.CLAW_EASY_INTAKE) {
            claw.setPosition(CLAW_CLOSED_POWER);
            currentClawState = clawStates.CLAW_CLOSED;
        }
        else if (currentClawState == clawStates.CLAW_CLOSED) {
            claw.setPosition(CLAW_OPEN_POWER);
            currentClawState = clawStates.CLAW_OPEN;
        }
    }

    //Toggles claw on and off depending on currentClawState, but centers/lowers arm after closing claw
    public void smartToggleClaw(Arm servoarm) {
        if (currentClawState == clawStates.CLAW_OPEN || currentClawState == clawStates.CLAW_EASY_INTAKE) {
            //If the claw was open, close it
            claw.setPosition(CLAW_CLOSED_POWER);
            currentClawState = clawStates.CLAW_CLOSED;
        } else if (currentClawState == clawStates.CLAW_CLOSED) {
            //If the claw was closed, open it
            claw.setPosition(CLAW_OPEN_POWER);
            currentClawState = clawStates.CLAW_OPEN;
        }
    }


    public void CheckClaw(boolean currentButton , boolean lastButton, Arm servoarm) {
        //Keep resetting the delay period as long as the button is pressed
        if (currentButton){
            afterClawOpensDelayPeriod.reset();
        }

        //toggle the claw open or closed when button is pressed
        if (currentButton && !lastButton) {
            smartToggleClaw(servoarm);
        }

        //reset the delay period one final time after the button is released
        else if (!currentButton && lastButton) {
            afterClawOpensDelayPeriod.reset();
        }
        //if the claw is open and the delay period has passed,
        //  1) set the claw to easy intake position (which also centers and lowers lift)
        else if (currentClawState == clawStates.CLAW_OPEN && afterClawOpensDelayPeriod.seconds() > 1){
            setEasyIntake();
        }
    }




    public void AutoDeliverClawTogggle() {
        if (currentClawState == clawStates.CLAW_CLOSED) {
            //If the claw was closed, open it
            claw.setPosition(CLAW_OPEN_POWER);
            currentClawState = clawStates.CLAW_OPEN;
            afterClawOpensDelayPeriod.reset();
        }
    }

}
