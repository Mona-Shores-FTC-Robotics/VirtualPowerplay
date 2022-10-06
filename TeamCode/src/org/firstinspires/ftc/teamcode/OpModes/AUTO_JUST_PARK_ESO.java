package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;


@Autonomous(name = "AUTO_JUST_PARK_ESO")
public class AUTO_JUST_PARK_ESO extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();

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

        //Autonomous Routine Example

        //MecDrive.encoderDrive(.4, -10, -10, this);
        //MecDrive.encoderDrive(.4, 55, 55, this);
        //MecDrive.strafeDrive(.3, -20, -20, this);
        //MecDrive.turn(45,this);
        //MecDrive.turnTo(-20,this);

        MecDrive.encoderDrive(.4, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE, this);

        if(Signal == 1){
            MecDrive.encoderDrive(.4, FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, this);
            MecDrive.strafeDrive(.4, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);
        }else if(Signal == 2){
            MecDrive.encoderDrive(.4, FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, this);
        }else if(Signal == 3){
            MecDrive.encoderDrive(.4, FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, this);
            MecDrive.strafeDrive(.4, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);
        }

    }
}



