package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    public static final double CLAW_OPEN_POWER = .4;
    public static final double CLAW_CLOSED_POWER = .5;
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

    public void CheckClaw(boolean currentButton , boolean lastButton, Arm servoarm, Lift lift) {
        if (currentButton && !lastButton) {
            //open and close the claw
           smartToggleClaw(servoarm, lift);
        } else if (currentClawState == clawStates.CLAW_OPEN && afterClawOpensDelayPeriod.seconds() > 1)
        {
            smartToggleClaw(servoarm, lift);
        }
    }

    public void smartToggleClaw(Arm servoarm, Lift lift) {
        if (currentClawState == clawStates.CLAW_OPEN) {
            claw.setPosition(CLAW_CLOSED_POWER);
            currentClawState = clawStates.CLAW_CLOSED;
            servoarm.setArmState(Arm.armState.ARM_CENTER);
            lift.StartLifting(GameConstants.ONE_CONE_INTAKE_HEIGHT_MM);
        }
        else if (currentClawState == clawStates.CLAW_CLOSED) {
            claw.setPosition(CLAW_OPEN_POWER);
            currentClawState = clawStates.CLAW_OPEN;
            afterClawOpensDelayPeriod.reset();
        }
    }



}
