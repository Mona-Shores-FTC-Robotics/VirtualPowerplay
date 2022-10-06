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


@Autonomous(name = "AUTO OTS SCORE TERMINAL AND STRAFE W/ COLOR SENSOR, SCORE 6")
@Disabled
public class AUTO_OTS_TERMINAL_COLOR_SENSOR_SCORE_6 extends LinearOpMode {

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

        //deliver initial cone
        MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);
        int cones = 1;

        MecDrive.strafeDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);
        MecDrive.colorStrafe(HIGH_SPEED, 1 , this);

        //Straighten up the Robot
        MecDrive.turnTo(0, this);

        //Drive to pickup cone
        MecDrive.encoderDrive(HIGH_SPEED, QUARTER_TILE_DISTANCE, QUARTER_TILE_DISTANCE, this);

        //pickup cone
        sleep(300);

        //Drive to high junction
        MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE), -(FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE), this);

        //dropoff cone
        cones++;
        telemetry.addData("# of Cones Delivered: ", cones);
        telemetry.addData("Run Time: ", getRuntime());
        telemetry.update();
        sleep(300);

        while (cones < 6 && getRuntime() < 26)
        {
            //Drive to pickup cone
            MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE, FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE, this);

            //pickup cone
            sleep(300);

            //Drive to high junction
            MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE), -(FULL_TILE_DISTANCE+QUARTER_TILE_DISTANCE), this);

            //dropoff cone
            cones++;
            telemetry.addData("# of Cones Delivered: ", cones);
            telemetry.addData("Run Time: ", getRuntime());
            telemetry.update();
            sleep(300);

        }
        //Park code
        if(Signal == 1) {
            MecDrive.encoderDrive(MED_SPEED, -(HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), -(HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), this);
        }
        else if (Signal == 2) {
        }
        else if (Signal == 3){
            MecDrive.strafeDrive(MED_SPEED, (HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), (HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), this);
        }

        telemetry.addData("# of Cones Delivered: ", cones);
        telemetry.addData("Run Time: ", getRuntime());
        telemetry.update();

        MecDrive.turnTo(90, this);
        MecDrive.encoderDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);



        sleep (5000);

    }
}



