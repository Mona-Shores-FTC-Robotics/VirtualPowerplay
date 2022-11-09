package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    public static final double CLAW_OPEN_POWER = 0;
    public static final double CLAW_CLOSED_POWER = .25;
    public Servo claw;
    public enum clawStates {CLAW_OPEN, CLAW_CLOSED}
    public clawStates currentClawState;

    public ElapsedTime afterClawOpensDelayPeriod = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        claw = ahwMap.servo.get("claw_servo");

        //set arm at intake position
        claw.setPosition(CLAW_CLOSED_POWER);
        currentClawState = clawStates.CLAW_CLOSED;
    }

    public void toggleClaw() {
        if (currentClawState == clawStates.CLAW_OPEN) {
            claw.setPosition(CLAW_CLOSED_POWER);
            currentClawState = clawStates.CLAW_CLOSED;
        }
        else if (currentClawState == clawStates.CLAW_CLOSED) {
            claw.setPosition(CLAW_OPEN_POWER);
            currentClawState = clawStates.CLAW_OPEN;
        }
    }

    public void CheckClaw(boolean currentButton , boolean lastButton, Arm servoarm) {
        if (currentButton && !lastButton) {
            //open and close the claw
           toggleClaw();
        }
    }

    public void AdvancedCheckClaw(boolean currentButton , boolean lastButton, Arm servoarm) {
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
        //if the claw is open and the delay period has passed, close the claw (including centering the arm and lowering the lift)
        else if (currentClawState == clawStates.CLAW_OPEN && afterClawOpensDelayPeriod.seconds() > 1){
            smartToggleClaw(servoarm);
        }
    }

    public void smartToggleClaw(Arm servoarm) {
        if (currentClawState == clawStates.CLAW_OPEN) {
            //If the claw was open, close it
            claw.setPosition(CLAW_CLOSED_POWER);
            currentClawState = clawStates.CLAW_CLOSED;
            //Center the arm, which also lowers the lift to the cone intake height after 1 second
            servoarm.setArmState(Arm.armState.ARM_CENTER);
        } else if (currentClawState == clawStates.CLAW_CLOSED) {
            //If the claw was closed, open it
            claw.setPosition(CLAW_OPEN_POWER);
            currentClawState = clawStates.CLAW_OPEN;
        }
    }


    public void AutonomousCheckClaw(boolean currentButton , boolean lastButton) {
        if (currentButton && !lastButton) {
            //open and close the claw
            toggleClaw();
        }
    }





}
