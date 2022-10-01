package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;



@Autonomous(name = "AUTO_JUST_PARK_MJL")
public class AUTO_JUST_PARK_MJL extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();

    // int code to get Full tile distance its easier

    public static int FULL_TILE_DISTANCE = 50;
    public static int HALF_TILE_DISTANCE = FULL_TILE_DISTANCE /2;
    public static int QUARTER_TILE_DISTANCE = HALF_TILE_DISTANCE /2;
    public static int EIGHTH_TILE_DISTANCE = QUARTER_TILE_DISTANCE /2;


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

        while (!isStarted()) {
            ButtonConfig.ConfigureAllianceColor(this);
            ButtonConfig.ConfigureStartingLocation( this);
            telemetry.addData("Alliance Color ", ButtonConfig.allianceColorString);
            telemetry.addData("Starting Location ", ButtonConfig.startingLocationString);
            telemetry.update();
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 3;
        }

        //Autonomous Routine Example using FullTileDistance thing
        MecDrive.encoderDrive(.4, -10, -10, this);
        MecDrive.encoderDrive(.4, FULL_TILE_DISTANCE*2+EIGHTH_TILE_DISTANCE, FULL_TILE_DISTANCE*2+EIGHTH_TILE_DISTANCE, this);

        //Just_PARK drive code
        if(Signal == 1) {
            MecDrive.strafeDrive(.3, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);
        }
        else if (Signal == 2) {
        }
         else if (Signal == 3){
                MecDrive.strafeDrive(.3, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);
        }

    }
}



