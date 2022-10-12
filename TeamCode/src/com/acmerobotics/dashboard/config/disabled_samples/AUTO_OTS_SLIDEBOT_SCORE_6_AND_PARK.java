package com.acmerobotics.dashboard.config.disabled_samples;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;


@Autonomous(name = "AUTO_OTS SLIDE BOT SCORE, CONE STACK, AND PARK")
@Disabled
public class AUTO_OTS_SLIDEBOT_SCORE_6_AND_PARK extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    public final ElapsedTime runtime = new ElapsedTime();

    public static final double SLIDE_INTAKE = .5;
    public static final double SLIDE_LEFT_OUTTAKE = 0;
    public static final double SLIDE_RIGHT_OUTTAKE = 360;

    @Override

    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        Servo slide = hardwareMap.servo.get("slide_servo");
        Servo lift = hardwareMap.servo.get("elevation_servo");

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

        //start with a cone in proper turret orientation for scoring, but at lower lift
        slide.setPosition(SLIDE_LEFT_OUTTAKE);
        lift.setPosition(GROUND_CONE_JUNCTION_SCORE_HEIGHT_MM);

        //align to the wall and calibrate gyro
        MecDrive.strafeDrive(LOW_SPEED, 10, 10, this);
        MecDrive.calibrateGyro(this);

        //strafe to line up with the cone stack
        MecDrive.strafeDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE), this);

        //drive toward middle of field
        MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);

        //rotate turret to deliver to High Junction
        //this code won't work if high junction is on the left.
        slide.setPosition(SLIDE_LEFT_OUTTAKE);

        //raise lift to height to deliver to High Junction
        lift.setPosition(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM);

        //strafe to the high pole
        MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);

        // Deliver initial cone
        // release cone
        int coneDeliveryTracker = 1;

        //strafe away from the high pole
        MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);

        int coneStackTracker = 5;

        while (coneStackTracker > 0 && (getRuntime() < 25.0))
        {
            //move turret to pickup position
            slide.setPosition(SLIDE_INTAKE);

            //lower lift to correct cone stack intake height
            switch (coneStackTracker) {
                case 5: {
                    lift.setPosition(FIVE_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }
                case 4: {
                    lift.setPosition(FOUR_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }

                case 3: {
                    lift.setPosition(THREE_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }

                case 2: {
                    lift.setPosition(TWO_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }
                case 1: {
                    lift.setPosition(ONE_CONE_INTAKE_HEIGHT_MM);
                    break;
                }
            }

            telemetry.addData("# of Cones Left on Stack", coneStackTracker);
            telemetry.addData("Current Lift Height", lift.getPosition());
            telemetry.addData("# of Cones Delivered During Auto", coneDeliveryTracker);
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();

            //Drive to pickup cone from cone stack
            MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE, this);

            //lower lift by set amount based on current lift position
            lift.setPosition(lift.getPosition()-CONE_INTAKE_HEIGHT_CHANGE_MM);

            //activate intake to grab cone
            coneStackTracker = coneStackTracker - 1;

            //raise lift by set amount based on current lift position
            lift.setPosition(lift.getPosition()+CONE_INTAKE_HEIGHT_CHANGE_MM);

            //Drive toward middle of field
            MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE), -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE), this);

            //move turret to deliver position
            slide.setPosition(SLIDE_LEFT_OUTTAKE);

            //raise lift to height to deliver to High Junction
            lift.setPosition(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM);

            //Strafe to the high pole
            MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);

            //drop off cone
            coneDeliveryTracker = coneDeliveryTracker +1;

            //strafe away from the high pole
            MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
        }

        lift.setPosition(ONE_CONE_INTAKE_HEIGHT_MM);

        telemetry.addData("# of Cones Left on Stack", coneStackTracker);
        telemetry.addData("Current Lift Height", lift.getPosition());
        telemetry.addData("# of Cones Delivered During Auto", coneDeliveryTracker);
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();

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

        MecDrive.turnTo(90, this);
        MecDrive.encoderDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);

    }
}


