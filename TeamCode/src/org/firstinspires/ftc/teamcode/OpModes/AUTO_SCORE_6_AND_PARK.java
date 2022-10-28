package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gyro;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;


import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.*;


@Autonomous(name = "AUTO_OTS_SCORE_6_AND_PARK")
public class AUTO_SCORE_6_AND_PARK extends LinearOpMode {

    int Signal;
    DriveTrain MecDrive = new DriveTrain(this);
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    Arm ServoArm = new Arm();
    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    Lift Lift = new Lift(this);
    Gyro Gyro = new Gyro(this);

    public final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        ServoArm.init(hardwareMap);
        ServoIntake.init(hardwareMap);
        ServoClaw.init(hardwareMap);
        Lift.init(hardwareMap);
        Gyro.init(hardwareMap);

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
        }

        runtime.reset();

        //start with a cone for scoring at intake position with lift low
        ServoArm.setPosition(ARM_CENTER_INTAKE);

        //align to the wall and calibrate gyro
        MecDrive.startEncoderDrive(MED_SPEED, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE);
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }

        Gyro.calibrateGyro();

        //drive to line up with the cone stack
        MecDrive.startEncoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE), (FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE));
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }
        MecDrive.turnTo(-90 * ButtonConfig.startPositionMultiplier * ButtonConfig.allianceColorMultiplier, Gyro);

        //drive toward middle of field
        MecDrive.startEncoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE));
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }

        //rotate turret to deliver to High Junction
        //this code won't work if high junction is on the left.
        if(ButtonConfig.allianceColorAndLocationFactor == 1){
        ServoArm.setPosition(ARM_LEFT_OUTTAKE);}
        else if (ButtonConfig.allianceColorAndLocationFactor == -1){
        ServoArm.setPosition(ARM_RIGHT_OUTTAKE);}

        //raise lift to height to deliver to High Junction
        Lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM);
        while (opModeIsActive() && Lift.alreadyLifting == true) {
            Lift.ContinueLifting();
        }

        //strafe to the high pole
        MecDrive.startStrafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier * ButtonConfig.allianceColorMultiplier),
                                              -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier * ButtonConfig.allianceColorMultiplier));
        while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
            MecDrive.ContinueStrafing();
        }

        // Open claw to release cone
        ServoClaw.toggleClaw();
        int coneDeliveryTracker = 1;

        //Give cone a moment to release;
        sleep(400);

        //Close claw for intaking next cone
        ServoClaw.toggleClaw();

        //strafe away from the high pole
        MecDrive.startStrafeDrive(HIGH_SPEED,   (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor),
                                                (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor));
        while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
            MecDrive.ContinueStrafing();
        }

        int coneStackTracker = 5;

        while (coneStackTracker > 0 && (getRuntime() < 25.0))
        {
            //move turret to pickup position
            ServoArm.setPosition(ARM_CENTER_INTAKE);

            //lower lift to correct cone stack intake height
            switch (coneStackTracker) {
                case 5: {
                    Lift.StartLifting(FIVE_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }
                case 4: {
                    Lift.StartLifting(FOUR_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }
                case 3: {
                    Lift.StartLifting(THREE_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }
                case 2: {
                    Lift.StartLifting(TWO_CONE_STACK_INTAKE_HEIGHT_MM);
                    break;
                }
                case 1: {
                    Lift.StartLifting(ONE_CONE_INTAKE_HEIGHT_MM);
                    break;
                }
            }

            while (opModeIsActive() && Lift.alreadyLifting == true) {
                Lift.ContinueLifting();
            }

            telemetry.addData("# of Cones Left on Stack", coneStackTracker);
            telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
            telemetry.addData("# of Cones Delivered During Auto", coneDeliveryTracker);
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();

            //Drive to pickup cone from cone stack
            MecDrive.startEncoderDrive(HIGH_SPEED,FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE,
                                        FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE);
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                MecDrive.ContinueStrafing();
            }

            //lower lift by set amount based on current lift position
            Lift.StartLifting(Lift.liftMotor.getCurrentPosition()-CONE_INTAKE_HEIGHT_CHANGE_MM);
            while (opModeIsActive() && Lift.alreadyLifting == true) {
                Lift.ContinueLifting();
            }

            //turn on intake to grab cone
            ServoIntake.toggleIntake();

            //wait for the intake to work
            sleep(150);

            coneStackTracker = coneStackTracker - 1;

            //turn off intake now that cone is grabbed
            ServoIntake.toggleIntake();

            //raise lift by set amount based on current lift position
            Lift.StartLifting(Lift.liftMotor.getCurrentPosition()+CONE_INTAKE_HEIGHT_CHANGE_MM);
            while (opModeIsActive() && Lift.alreadyLifting == true) {
                Lift.ContinueLifting();
            }

            //Drive toward middle of field
            MecDrive.startEncoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE));
            while (opModeIsActive() && MecDrive.alreadyDriving == true) {
                MecDrive.ContinueDriving();
            }

            //move turret to deliver position
            if(ButtonConfig.allianceColorAndLocationFactor == 1){
                ServoArm.setPosition(ARM_LEFT_OUTTAKE);}
            else if (ButtonConfig.allianceColorAndLocationFactor == -1){
                ServoArm.setPosition(ARM_RIGHT_OUTTAKE);}

            //raise lift to height to deliver to High Junction
            Lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM);
            while (opModeIsActive() && Lift.alreadyLifting == true) {
                Lift.ContinueLifting();
            }

            //Strafe to the high pole
            MecDrive.startStrafeDrive(HIGH_SPEED,   -(QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor),
                                                    -(QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor));
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                MecDrive.ContinueStrafing();
            }

            //drop off cone
            ServoClaw.toggleClaw();
            coneDeliveryTracker = coneDeliveryTracker +1;

            //wait for cone to be released
            sleep(350);

            //close claw for next intake
            ServoClaw.toggleClaw();

            //strafe away from the high pole
            MecDrive.startStrafeDrive(HIGH_SPEED,   (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor),
                                                    (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor));
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                MecDrive.ContinueStrafing();
            }
        }

        Lift.StartLifting(ONE_CONE_INTAKE_HEIGHT_MM);
        while (opModeIsActive() && Lift.alreadyLifting == true) {
            Lift.ContinueLifting();
        }
        ServoArm.setPosition(ARM_CENTER_INTAKE);

        //Park code
        if(Signal == 3) {
            MecDrive.startEncoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE));
        }
        else if (Signal == 2) {

            MecDrive.startEncoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE, HALF_TILE_DISTANCE);
        }
        else if (Signal == 1){
            MecDrive.startEncoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE);
        }
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }

        MecDrive.turnTo(180, Gyro);
        MecDrive.startEncoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE, HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE);
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }

        telemetry.addData("# of Cones Left on Stack", coneStackTracker);
        telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
        telemetry.addData("# of Cones Delivered During Auto", coneDeliveryTracker);
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();
        sleep(2000);

    }
}



