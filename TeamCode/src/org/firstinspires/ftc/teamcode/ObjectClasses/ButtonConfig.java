package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ButtonConfig {

    private boolean confirmAllianceColorSelection = false;
    private boolean confirmStartingPositionSelection = false;
    boolean toggleReadyB = false;

    LinearOpMode activeOpMode;
    public AllianceColor currentAllianceColor;
    public int allianceColorMultiplier;//-1 = RED 1 = blue

    public StartPosition currentStartPosition;
    public int startPositionMultiplier; //1 is row 2 and -1 is row 5

    // alliance color times starting location
    public int allianceColorAndLocationFactor;

    public ButtonConfig(LinearOpMode activeOpMode) {
        this.activeOpMode = activeOpMode;
    }

    public void init() {

        currentAllianceColor = AllianceColor.NOT_SET_YET;
        allianceColorMultiplier = 1; //1 = Blue // -1 = Red

        currentStartPosition = StartPosition.NOT_SET_YET;
        startPositionMultiplier = 1; //1 = close to audience A2/F2 // -1 =opposite side as audience A5/F5

        allianceColorAndLocationFactor = allianceColorMultiplier*startPositionMultiplier;
    }

    public void ConfigureAllianceColor() {

        while (!confirmAllianceColorSelection) {
            activeOpMode.telemetry.addLine("Select Alliance Color with D-pad");
            activeOpMode.telemetry.addData("Current Alliance Color", currentAllianceColor);
            activeOpMode.telemetry.addData("Press B", "Press B to confirm selection");
            activeOpMode.telemetry.update();

            if (activeOpMode.gamepad1.dpad_up) {
                currentAllianceColor = AllianceColor.RED;
                allianceColorMultiplier = -1;
            }
            if (activeOpMode.gamepad1.dpad_down) {
                currentAllianceColor = AllianceColor.BLUE;
                allianceColorMultiplier = 1;
            }
            allianceColorAndLocationFactor = allianceColorMultiplier * startPositionMultiplier;

            boolean G1b = activeOpMode.gamepad1.b;

            if (G1b == false) {
                toggleReadyB = true;
            }

            if (G1b && toggleReadyB && currentAllianceColor != AllianceColor.NOT_SET_YET) {
                toggleReadyB = false;
                confirmAllianceColorSelection = true;
                activeOpMode.telemetry.addData(" Alliance Color Selected: ", currentAllianceColor);
                activeOpMode.telemetry.update();
            }
        }
    }


    public void ConfigureStartingPosition() {

        while (!confirmStartingPositionSelection) {

            activeOpMode.telemetry.addLine("Select Starting Position with D-pad");
            activeOpMode.telemetry.addData("Current Starting Position ", currentStartPosition);
            activeOpMode.telemetry.addData("Press B", "Press B to confirm selection");
            activeOpMode.telemetry.update();

            if (activeOpMode.gamepad1.dpad_up) {
               currentStartPosition = StartPosition.ROW_5;
               startPositionMultiplier = -1;
            }
            if (activeOpMode.gamepad1.dpad_down) {
                    currentStartPosition = StartPosition.ROW_2;
                    startPositionMultiplier = 1;
            }
            allianceColorAndLocationFactor = allianceColorMultiplier * startPositionMultiplier;

            boolean G1b = activeOpMode.gamepad1.b;

            if (G1b == false) {
                toggleReadyB = true;
            }

            if (G1b && toggleReadyB && currentStartPosition != StartPosition.NOT_SET_YET) {
                toggleReadyB = false;
                confirmStartingPositionSelection = true;
                activeOpMode.telemetry.addData(" Alliance ColorSelected: ", currentAllianceColor);
                activeOpMode.telemetry.addData(" Starting Position Selected: ", currentStartPosition);
                activeOpMode.telemetry.update();
            }

        }
    }

    public void ConfigureMultiplier(LinearOpMode activeOpMode, DriveTrain MecDrive) {

        if (activeOpMode.gamepad1.left_stick_y > .25 && MecDrive.multiplier > MecDrive.MINMULT) {
            MecDrive.multiplier = (MecDrive.multiplier * 10 - 1) / 10;

            try {
                sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        } else {
            if (activeOpMode.gamepad1.left_stick_y < -.25 && MecDrive.multiplier < MecDrive.MAXMULT) {
                MecDrive.multiplier = (MecDrive.multiplier * 10 + 1) / 10;
            }

            try {
                sleep(400);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        activeOpMode.telemetry.addData("Drive Multiplier", MecDrive.multiplier);
        activeOpMode.telemetry.update();
    }

    public enum StartPosition {
        ROW_2,
        ROW_5,
        NOT_SET_YET
    }

    public enum AllianceColor {
        BLUE,
        RED,
        NOT_SET_YET
    }

}
