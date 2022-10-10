package com.acmerobotics.dashboard.config.disabled_samples;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
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


@Autonomous(name = "DHS Strafe and Score with Smooth Path (TIME BASED)")
@Disabled
public class AUTO_DHS_STRAFE_AND_SCORE_SMOOTH_PATH extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig(this);
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

            ButtonConfig.ConfigureAllianceColor();
            ButtonConfig.ConfigureStartingPosition();
            telemetry.addData("Alliance Color ", ButtonConfig.currentAllianceColor);
            telemetry.addData("Starting Location ", ButtonConfig.currentStartPosition);
            telemetry.update();

        }
        runtime.reset();
        //Autonomous Routine Example

        MecDrive.strafeDrive(LOW_SPEED, 10, 10, this);
        MecDrive.calibrateGyro(this);

        //strafe to middle height pole
        MecDrive.strafeDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), this);

        //backup to deliver to medium height pole
        MecDrive.encoderDrive(HIGH_SPEED, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE, this);
        int cones = 1;
        MecDrive.encoderDrive(HIGH_SPEED, QUARTER_TILE_DISTANCE, QUARTER_TILE_DISTANCE, this);

        //strafe to line up with the cone stack
        MecDrive.strafeDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE-2), -(HALF_TILE_DISTANCE-2), this);

        while (cones < 4)
        {
            //Drive to pickup cone from cone stack
            MecDrive.drive = 1;
            MecDrive.MecanumDrive();
            sleep(925);

            MecDrive.strafe = 0;
            MecDrive.drive = 0;
            MecDrive.turn = 0;
            MecDrive.MecanumDrive();

            //pickup cone
            sleep(1000);

            //Drive toward middle of field
            MecDrive.drive = -1;
            MecDrive.MecanumDrive();
            sleep(700);

            //Smooth path toward contested high pole
            MecDrive.strafe = -.5;
            MecDrive.drive = -.9;
            MecDrive.turn = -.7;
            MecDrive.MecanumDrive();
            sleep(1300);

            MecDrive.strafe = 0;
            MecDrive.drive = 0;
            MecDrive.turn = 0;
            MecDrive.MecanumDrive();

            //dropoff cone
            cones++;
            telemetry.addData("Cones", cones);
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
            sleep(800);

            MecDrive.strafe = .5;
            MecDrive.drive = .9;
            MecDrive.turn = .7;
            MecDrive.MecanumDrive();
            sleep(1300);

            MecDrive.strafe = 0;
            MecDrive.drive = 0;
            MecDrive.turn = 0;
            MecDrive.MecanumDrive();
        }

        //Park code
        if(Signal == 1) {
            MecDrive.encoderDrive(MED_SPEED, -(FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE), -(FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE), this);
        }
        else if (Signal == 2) {

            MecDrive.encoderDrive(MED_SPEED, -(QUARTER_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), this);
        }
        else if (Signal == 3){
            MecDrive.encoderDrive(MED_SPEED, HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, this);
        }

        sleep (5000);

    }
}



