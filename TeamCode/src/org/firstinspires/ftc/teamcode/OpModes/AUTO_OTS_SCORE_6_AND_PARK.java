package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;


import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.*;


@Autonomous(name = "AUTO_OTS_SCORE_6_AND_PARK")
public class AUTO_OTS_SCORE_6_AND_PARK extends LinearOpMode {

    int Signal;
    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    Arm ServoArm = new Arm();
    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    Lift Lift = new Lift();

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
        Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM,this);

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
        ServoArm.setPosition(ARM_INTAKE);

        //align to the wall and calibrate gyro
        MecDrive.encoderDrive(MED_SPEED, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE, this);
        MecDrive.calibrateGyro(this);

        //drive to line up with the cone stack
        MecDrive.encoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE), (FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE), this);
        MecDrive.turnTo(-90 * ButtonConfig.startPositionMultiplier * ButtonConfig.allianceColorMultiplier, this);

        //drive toward middle of field
        MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);

        //rotate turret to deliver to High Junction
        //this code won't work if high junction is on the left.
        if(ButtonConfig.allianceColorAndLocationFactor == 1){
        ServoArm.setPosition(ARM_LEFT_OUTTAKE);}
        else if (ButtonConfig.allianceColorAndLocationFactor == -1){
        ServoArm.setPosition(ARM_RIGHT_OUTTAKE);}

        //raise lift to height to deliver to High Junction
        Lift.moveLift(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, this);

        //strafe to the high pole
        MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier * ButtonConfig.allianceColorMultiplier), -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier * ButtonConfig.allianceColorMultiplier), this);

        // Open claw to release cone
        ServoClaw.toggleClaw();
        int coneDeliveryTracker = 1;

        //Give cone a moment to release;
        sleep(100);

        //Close claw for intaking next cone
        ServoClaw.toggleClaw();

        //strafe away from the high pole
        MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), this);

        int coneStackTracker = 5;

        while (coneStackTracker > 0 && (getRuntime() < 25.0))
        {
            //move turret to pickup position
            ServoArm.setPosition(ARM_INTAKE);

            //lower lift to correct cone stack intake height
            switch (coneStackTracker) {
                case 5: {
                    Lift.moveLift(FIVE_CONE_STACK_INTAKE_HEIGHT_MM, this);
                    break;
                }
                case 4: {
                    Lift.moveLift(FOUR_CONE_STACK_INTAKE_HEIGHT_MM, this);
                    break;
                }

                case 3: {
                    Lift.moveLift(THREE_CONE_STACK_INTAKE_HEIGHT_MM, this);
                    break;
                }

                case 2: {
                    Lift.moveLift(TWO_CONE_STACK_INTAKE_HEIGHT_MM, this);
                    break;
                }
                case 1: {
                    Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM, this);
                    break;
                }
            }

            telemetry.addData("# of Cones Left on Stack", coneStackTracker);
            telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
            telemetry.addData("# of Cones Delivered During Auto", coneDeliveryTracker);
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();

            //Drive to pickup cone from cone stack
            MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE, this);

            //lower lift by set amount based on current lift position
            Lift.moveLift(Lift.liftMotor.getCurrentPosition()-CONE_INTAKE_HEIGHT_CHANGE_MM, this);

            //turn on intake to grab cone
            ServoIntake.toggleIntake();

            //wait for the intake to work
            sleep(150);

            coneStackTracker = coneStackTracker - 1;

            //turn off intake now that cone is grabbed
            ServoIntake.toggleIntake();

            //raise lift by set amount based on current lift position
            Lift.moveLift(Lift.liftMotor.getCurrentPosition()+CONE_INTAKE_HEIGHT_CHANGE_MM, this);

            //Drive toward middle of field
            MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE+EIGHTH_TILE_DISTANCE), this);

            //move turret to deliver position
            if(ButtonConfig.allianceColorAndLocationFactor == 1){
                ServoArm.setPosition(ARM_LEFT_OUTTAKE);}
            else if (ButtonConfig.allianceColorAndLocationFactor == -1){
                ServoArm.setPosition(ARM_RIGHT_OUTTAKE);}

            //raise lift to height to deliver to High Junction
            Lift.moveLift(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, this);

            //Strafe to the high pole
            MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), -(QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), this);

            //drop off cone
            ServoClaw.toggleClaw();
            coneDeliveryTracker = coneDeliveryTracker +1;

            //wait for cone to be released
            sleep(150);

            //close claw for next intake
            ServoClaw.toggleClaw();

            //strafe away from the high pole
            MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), (QUARTER_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), this);
        }

        Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM, this);
        ServoArm.setPosition(ARM_INTAKE);

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

        MecDrive.turnTo(180, this);
        MecDrive.encoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE, HALF_TILE_DISTANCE+QUARTER_TILE_DISTANCE, this);

        telemetry.addData("# of Cones Left on Stack", coneStackTracker);
        telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
        telemetry.addData("# of Cones Delivered During Auto", coneDeliveryTracker);
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();
        sleep(10000);

    }
}



