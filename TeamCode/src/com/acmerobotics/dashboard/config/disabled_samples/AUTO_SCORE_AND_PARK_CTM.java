package com.acmerobotics.dashboard.config.disabled_samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;

@Autonomous(name = "AUTO_SCORE_AND_PARK_CTM")
@Disabled
public class AUTO_SCORE_AND_PARK_CTM extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig(this);

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);

        ButtonConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);
        Signal = 1;
        while (!isStarted()) {
            ButtonConfig.ConfigureAllianceColor();
            ButtonConfig.ConfigureStartingPosition();
            telemetry.addData("Alliance Color ", ButtonConfig.currentAllianceColor);
            telemetry.addData("Starting Location ", ButtonConfig.currentStartPosition);
            telemetry.update();
            //Use Webcam to find out Signal and store in Signal variable

        }

        //Autonomous Routine Example
        //backup to make sure that its square to the wall

        MecDrive.encoderDrive(.4, -10, -10, this);
        //we go forward until we are at the ground junction
        MecDrive.encoderDrive(.4, 110, 110, this);

        if (Signal == 1) {
            //strafe until in parking zone 1
            MecDrive.strafeDrive(.3, -25, -25, this);
            MecDrive.strafeDrive(.3, -25, -25, this);
            MecDrive.encoderDrive(.4, -5, -5, this);

        } else if (Signal == 2) {
            MecDrive.strafeDrive(.3, -25, -25, this);
            MecDrive.strafeDrive(.3, 25, 25, this);
            MecDrive.encoderDrive(.4, -5, -5, this);


        } else if (Signal == 3) {
            //strafe until in parking zone 3
            MecDrive.strafeDrive(.3, -25, -25, this);
            MecDrive.strafeDrive(.3, 75, 75, this);
            MecDrive.encoderDrive(.4, -5, -5, this);


        }

    }
}





