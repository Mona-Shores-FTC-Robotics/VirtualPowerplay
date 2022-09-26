package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ButtonConfig {

    public String startingLocationString;
    public String allianceColorString;

    /* Constructor */
    public ButtonConfig() {

    }

    public void init() {
        startingLocationString = "Row 2";
        allianceColorString = "Blue";
    }

    public void ConfigureAllianceColor(LinearOpMode activeOpMode) {
        if (activeOpMode.gamepad1.dpad_up)
            {
                if (allianceColorString.equals("Blue")) {
                    allianceColorString = "Red";
                } else if (allianceColorString.equals("Red")) {
                    allianceColorString = "Blue";
                }

              activeOpMode.sleep(250);
            }
    }

    public void ConfigureStartingLocation(LinearOpMode activeOpMode) {
        if (activeOpMode.gamepad1.dpad_down) {
                if (startingLocationString.equals("Row 2")) {
                    startingLocationString = "Row 5";
                } else if (startingLocationString.equals("Row 5")) {
                    startingLocationString = "Row 2";
                }
            activeOpMode.sleep(250);
            }
    }
}
