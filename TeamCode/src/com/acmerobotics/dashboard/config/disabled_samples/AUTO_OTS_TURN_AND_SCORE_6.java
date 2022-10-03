package com.acmerobotics.dashboard.config.disabled_samples;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;


@Autonomous(name = "AUTO_OTS TURN AND SCORE 6")
@Disabled
public class AUTO_OTS_TURN_AND_SCORE_6 extends LinearOpMode {

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

        MecDrive.encoderDrive(LOW_SPEED, -30, -30, this);
        MecDrive.calibrateGyro(this);

        MecDrive.encoderDrive(HIGH_SPEED, ((FULL_TILE_DISTANCE*2)+QUARTER_TILE_DISTANCE), ((FULL_TILE_DISTANCE*2)+QUARTER_TILE_DISTANCE), this);
        MecDrive.turnTo(80, this);

        // drive to high pole
        MecDrive.encoderDrive(MED_SPEED, QUARTER_TILE_DISTANCE, QUARTER_TILE_DISTANCE, this);

        //deliver initial cone
        sleep(1000);
        int cones = 1;

        while (cones <=6 && getRuntime() <24)
        {

            //Drive to stack line
            MecDrive.colorDrive(-MED_SPEED, ButtonConfig.allianceColor, this);

            //STRAIGHTEN
            MecDrive.turnTo(90, this);

            //Drive to pickup cone
            MecDrive.encoderDrive(MED_SPEED, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE, this);

            //pickup cone

            MecDrive.encoderDrive(MED_SPEED, QUARTER_TILE_DISTANCE, QUARTER_TILE_DISTANCE, this);

            MecDrive.turnTo(80, this);

            //Drive to high junction
            MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);

            //dropoff cone
            cones++;
            telemetry.addData("Cones", cones);
            telemetry.update();
            sleep(1000);
        }

    }
}



