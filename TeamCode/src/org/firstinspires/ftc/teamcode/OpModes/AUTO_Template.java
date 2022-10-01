package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;

@Autonomous(name = "AUTO_Template")
public class AUTO_Template extends LinearOpMode {

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
            Signal = 1;
        }

        //Autonomous Routine Example

        MecDrive.encoderDrive(LOW_SPEED, -10, -10, this);
        MecDrive.encoderDrive(MED_SPEED, ((FULL_TILE_DISTANCE*2)+EIGHTH_TILE_DISTANCE), ((FULL_TILE_DISTANCE*2)+EIGHTH_TILE_DISTANCE), this);
        MecDrive.strafeDrive(LOW_SPEED, -20, -20, this);
        MecDrive.turn(45,this);
        MecDrive.turnTo(-20,this);
    }
}



