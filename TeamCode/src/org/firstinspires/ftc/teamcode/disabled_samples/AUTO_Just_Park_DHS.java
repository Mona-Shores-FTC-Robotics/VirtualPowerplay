package org.firstinspires.ftc.teamcode.disabled_samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;

@Disabled
@Autonomous(name = "AUTO_Just_Park_DHS")
public class AUTO_Just_Park_DHS extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();
    int Signal;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        ButtonConfig.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sleep(1000);

        while (!isStarted()) {
            ButtonConfig.ConfigureAllianceColor(this);
            ButtonConfig.ConfigureStartingLocation( this);
            telemetry.addData("Alliance Color ", ButtonConfig.allianceColorString);
            telemetry.addData("Starting Location ", ButtonConfig.startingLocationString);
            telemetry.update();
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 1;
        }

        if (opModeIsActive()) {
            //Backup into wall
            MecDrive.encoderDrive(.5, -10, -10, this);

            //Drive straight toward the Signal Cone
            MecDrive.encoderDrive(.8, 105, 105, this);

            if (Signal == 1)
            {
                //strafe left to park
               MecDrive.strafeDrive(.6, 50, 50, this);
            }
            else if (Signal == 2)
            {
                //stay put, you are already at the parking spot
            }
            else if (Signal == 3)
            {
                //strafe right to park
                MecDrive.strafeDrive(.6, -50, -50, this);
            }
        }
    }
}