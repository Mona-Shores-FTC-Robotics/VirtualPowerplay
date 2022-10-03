package com.acmerobotics.dashboard.config.disabled_samples;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;


@Autonomous(name = "AUTO_OTS STRAFE AND SCORE 6 AND PARK")
@Disabled
public class AUTO_OTS_STRAFE_AND_SCORE_SIX_SMOOTH_AND_PARK extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();
    public final ElapsedTime runtime = new ElapsedTime();

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
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 1;

            ButtonConfig.ConfigureAllianceColor(this);
            ButtonConfig.ConfigureStartingLocation( this);
            telemetry.addData("Alliance Color ", ButtonConfig.allianceColorString);
            telemetry.addData("Starting Location ", ButtonConfig.startingLocationString);
            telemetry.update();

        }
        runtime.reset();
        //Autonomous Routine Example

        MecDrive.strafeDrive(LOW_SPEED, 10, 10, this);
        MecDrive.calibrateGyro(this);

        //strafe to line up with the cone stack
        MecDrive.strafeDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE*2 + EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE*2 + EIGHTH_TILE_DISTANCE), this);

        //drive toward middle of field
        MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);

        //strafe to the high pole
        MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);

        // Deliver initial cone
        int cones = 1;
        sleep(1000);

        //strafe away from the high pole
        MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);

        while (cones < 5 && getRuntime() < 24)
        {
            //Drive to pickup cone from cone stack
            MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE, this);

            //pickup cone
            sleep(1000);

            //Drive toward middle of field
            MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE), -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE), this);

            //Strafe to the high pole
            MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);

            //drop off cone
            cones++;
            telemetry.addData("Cones", cones);
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
            sleep(1000);

            //strafe away from the high pole
            MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
        }

        //Park code
        if(Signal == 1) {
            MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);
        }
        else if (Signal == 2) {

            MecDrive.encoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE, HALF_TILE_DISTANCE, this);
        }
        else if (Signal == 3){
            MecDrive.encoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE, this);
        }

        sleep (5000);

    }
}



