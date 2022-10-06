package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public static final double CLAW_OPEN_POWER = .4;
    public static final double CLAW_CLOSED_POWER = .5;
    public Servo claw;
    public enum clawStates {CLAW_OPEN, CLAW_CLOSED}
    public clawStates currentClawState;

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
}
