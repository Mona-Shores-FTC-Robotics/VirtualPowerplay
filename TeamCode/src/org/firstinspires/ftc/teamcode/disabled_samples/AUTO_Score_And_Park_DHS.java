package org.firstinspires.ftc.teamcode.disabled_samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;

@Autonomous(name = "AUTO_Score_And_Park_DHS")
@Disabled
public class AUTO_Score_And_Park_DHS extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();
    int Signal;

    int FULL_TILE_DIST = 50;
    int HALF_TILE_DIST = FULL_TILE_DIST/2;
    int QUARTER_TILE_DIST = HALF_TILE_DIST/2;
    int EIGHTH_TILE_DIST = QUARTER_TILE_DIST/2;

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

            //Use Webcam to find out Signal and store in Signal variable
            Signal = 2;

            telemetry.addData("Alliance Color String ", ButtonConfig.allianceColorString);
            telemetry.addData("Alliance Color ", ButtonConfig.allianceColor);
            telemetry.addData("Starting Location String ", ButtonConfig.startingLocationString);
            telemetry.addData("Starting Location ", ButtonConfig.startLocation);
            telemetry.addData("Alliance Location Factor ", ButtonConfig.allianceLocationFactor);
            telemetry.addData("Signal ", Signal);
            telemetry.update();
        }

        if (opModeIsActive()) {
            //Backup into wall to straighten the robot
            MecDrive.encoderDrive(.5, -10, -10, this);

            //Drive Forward
            MecDrive.encoderDrive(.7, (FULL_TILE_DIST*2)+EIGHTH_TILE_DIST, (FULL_TILE_DIST*2)+EIGHTH_TILE_DIST, this);

            //Strafe in Front of High Pole
            MecDrive.strafeDrive(.7, -(HALF_TILE_DIST * ButtonConfig.allianceLocationFactor), -(HALF_TILE_DIST * ButtonConfig.allianceLocationFactor), this);

            //Place Cone on High Pole
            //PLACEHOLDER CODE FOR PLACING CONE
            MecDrive.encoderDrive(.5, 3, 3, this);
            sleep(500);
            MecDrive.encoderDrive(.5, -3, -3, this);

            //Park after placing cone
            if (Signal == 1) {
                MecDrive.strafeDrive(.5, (-FULL_TILE_DIST + (HALF_TILE_DIST * ButtonConfig.allianceLocationFactor)), (-FULL_TILE_DIST + (HALF_TILE_DIST * ButtonConfig.allianceLocationFactor)), this);
            } else if (Signal == 2) {
                MecDrive.strafeDrive(.5, (HALF_TILE_DIST* ButtonConfig.allianceLocationFactor), (HALF_TILE_DIST * ButtonConfig.allianceLocationFactor), this);
            } else if (Signal == 3) {
               MecDrive.strafeDrive(.5, (FULL_TILE_DIST + (HALF_TILE_DIST * ButtonConfig.allianceLocationFactor)), (FULL_TILE_DIST + (HALF_TILE_DIST * ButtonConfig.allianceLocationFactor)), this);
            }
        }
    }
}