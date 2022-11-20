package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ButtonConfig {

    public static boolean confirmStartingPositionSelection = false;

    LinearOpMode activeOpMode;

    public static StartingPosition currentStartPosition;

    public enum StartingPosition {
        LEFT_SIDE,
        RIGHT_SIDE,
        NOT_SET_YET
    }

    public static int startPositionMultiplier; //1 is LEFT and -1 RIGHT

    public ButtonConfig(LinearOpMode activeOpMode) {
        this.activeOpMode = activeOpMode;
    }

    public void init() {
        ButtonConfig.currentStartPosition = StartingPosition.RIGHT_SIDE;
        ButtonConfig.startPositionMultiplier = -1;
        ButtonConfig.confirmStartingPositionSelection = false;
    }

    public void ConfigureStartingPosition(  Boolean leftButton, Boolean previousLeftButton,
                                            Boolean rightButton, Boolean previousRightButton,
                                            Boolean confirmButton, Boolean previousConfirmButton) {

        if (!confirmStartingPositionSelection && !activeOpMode.isStarted()) {

            if (leftButton && !previousLeftButton) {
               currentStartPosition = StartingPosition.LEFT_SIDE;
               startPositionMultiplier = 1;
            }
            if (rightButton && !previousRightButton) {
                currentStartPosition = StartingPosition.RIGHT_SIDE;
                startPositionMultiplier = -1;
            }
            if (confirmButton && !previousConfirmButton && currentStartPosition!= StartingPosition.NOT_SET_YET) {
                confirmStartingPositionSelection = true;
            }
        } else

        if (confirmButton && !previousConfirmButton) {
            confirmStartingPositionSelection = false;
        }
    }

    public void ConfigureMultiplier(LinearOpMode activeOpMode, DriveTrain MecDrive) {

        if (activeOpMode.gamepad1.left_stick_y > .25 && MecDrive.multiplier > MecDrive.MINMULT) {
            MecDrive.multiplier = (MecDrive.multiplier * 10 - 1) / 10;

            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        } else {
            if (activeOpMode.gamepad1.left_stick_y < -.25 && MecDrive.multiplier < MecDrive.MAXMULT) {
                MecDrive.multiplier = (MecDrive.multiplier * 10 + 1) / 10;
            }

            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        activeOpMode.telemetry.addData("Drive Multiplier", MecDrive.multiplier);
    }

    public Gamepad copy(Gamepad gamepad) {
        Gamepad pad = new Gamepad();
        pad.a = gamepad.a;
        pad.b = gamepad.b;
        pad.x = gamepad.x;
        pad.y = gamepad.y;

        pad.right_bumper = gamepad.right_bumper;
        pad.left_bumper = gamepad.left_bumper;


        pad.right_trigger = gamepad.right_trigger;
        pad.left_trigger = gamepad.left_trigger;

        pad.left_stick_x = gamepad.left_stick_x;
        pad.left_stick_y = gamepad.left_stick_y;
        pad.left_stick_button = gamepad.left_stick_button;

        pad.right_stick_x = gamepad.right_stick_x;
        pad.right_stick_y = gamepad.right_stick_y;
        pad.right_stick_button = gamepad.right_stick_button;

        pad.start = gamepad.start;
        pad.back = gamepad.back;

        pad.dpad_up = gamepad.dpad_up;
        pad.dpad_left = gamepad.dpad_left;
        pad.dpad_right = gamepad.dpad_right;
        pad.dpad_down = gamepad.dpad_down;

        return pad;
    }

}
