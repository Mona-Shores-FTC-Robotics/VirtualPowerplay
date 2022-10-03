package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.TeleOp_Iterative_Turret_Bot;

public class ButtonConfig {

    public String startingLocationString;
    public String allianceColorString;

    //-1 = RED 1 = blue
    public int allianceColor;

    //1 is row 2 and -1 is row 5
    public int startLocation;

    // alliance color times starting location
    public int allianceLocationFactor;

    public ButtonConfig() {
    }

    public void init() {
        startingLocationString = "Row 2";
        allianceColorString = "Blue";
        allianceColor = 1; //1 = Blue // -1 = Red
        startLocation = 1; //1 = close to audience A2/F2 // -1 =opposite side as audience A5/F5
        allianceLocationFactor = allianceColor*startLocation;
    }

    public void ConfigureAllianceColor(LinearOpMode activeOpMode) {
        if (activeOpMode.gamepad1.dpad_up)
            {
                if (allianceColorString.equals("Blue")) {
                    allianceColorString = "Red";
                    allianceColor = -1;
                } else if (allianceColorString.equals("Red")) {
                    allianceColorString = "Blue";
                    allianceColor = 1;
                }
                allianceLocationFactor = allianceColor*startLocation;
                activeOpMode.sleep(250);
            }
    }

    public void ConfigureStartingLocation(LinearOpMode activeOpMode) {
        if (activeOpMode.gamepad1.dpad_down) {
                if (startingLocationString.equals("Row 2")) {
                    startingLocationString = "Row 5";
                    startLocation = -1;
                } else if (startingLocationString.equals("Row 5")) {
                    startingLocationString = "Row 2";
                    startLocation = 1;
                }
            allianceLocationFactor = allianceColor*startLocation;
            activeOpMode.sleep(250);
            }
    }

    public void ConfigureMultiplier(TeleOp_Iterative_Turret_Bot activeOpMode, DriveTrain MecDrive) {
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
}
