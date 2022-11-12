package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_CENTER_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_RIGHT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.CONE_CLEARANCE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.CONE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.SIXTEENTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.AprilTagVision;
import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gyro;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;

@Autonomous(name = "AUTO_SCORE_6_AND_PARK_START_SIDEWAYS")
public class AUTO_SCORE_6_AND_PARK_START_SIDEWAYS extends LinearOpMode {

    int Signal;
    DriveTrain MecDrive = new DriveTrain(this);
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    AprilTagVision Vision = new AprilTagVision();
    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    Lift Lift = new Lift(this);
    Arm ServoArm = new Arm(Lift);
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
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        while (!isStarted()) {
            //save current and previous gamepad values for one loop
            previousGamepad1 = ButtonConfig.copy(currentGamepad1);
            currentGamepad1 = ButtonConfig.copy(gamepad1);

            previousGamepad2 = ButtonConfig.copy(currentGamepad2);
            currentGamepad2 = ButtonConfig.copy(gamepad2);

            //Use Webcam to find out Signal using April Tags
            //Vision.CheckForAprilTags(this);

            // User sets starting location left or right, and confirms selection with a button press
            // LEFT is a multiplier of 1, RIGHT is a multiplier of -1
            ButtonConfig.ConfigureStartingPosition( currentGamepad1.dpad_left, previousGamepad1.dpad_left,
                    currentGamepad1.dpad_right, previousGamepad1.dpad_right,
                    currentGamepad1.b,          previousGamepad1.b);

            telemetry.addData("Signal is ", Vision.currentSignal);
            telemetry.addLine(" ");
            telemetry.addLine("Select Starting Position with D-pad");
            telemetry.addData("Current Starting Position ", ButtonConfig.currentStartPosition);
            if (ButtonConfig.confirmStartingPositionSelection == false) {
                telemetry.addData("Unlocked", "Press CIRCLE to lock selection");
            } else {
                telemetry.addData("Locked", "Press CIRCLE to unlock selection");
            }
            telemetry.update();

            //let the operator control the intake and claw during init
            ServoClaw.CheckClaw(currentGamepad2.a, previousGamepad2.a);
            ServoIntake.CheckIntake(currentGamepad2.x, previousGamepad2.x);

            sleep(20);
        }

        runtime.reset();

        int coneDeliveryTracker = 0;
        int coneStackTracker = 5;

        telemetry.addData("Cones:", "Stack(%s)/Delivered(%s)", coneStackTracker, coneDeliveryTracker);
        telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();

        //Strafe past line up with the cone stack
        MecDrive.startStrafeDrive(HIGH_SPEED,   -(FULL_TILE_DISTANCE*2 + EIGHTH_TILE_DISTANCE)* ButtonConfig.startPositionMultiplier,
                                                -(FULL_TILE_DISTANCE*2 + EIGHTH_TILE_DISTANCE) * ButtonConfig.startPositionMultiplier);
        while (opModeIsActive() && MecDrive.alreadyStrafing) {
            MecDrive.ContinueStrafing();
        }

        MecDrive.startStrafeDrive(MED_SPEED,    (SIXTEENTH_TILE_DISTANCE)* ButtonConfig.startPositionMultiplier,
                                                (SIXTEENTH_TILE_DISTANCE) * ButtonConfig.startPositionMultiplier);
        while (opModeIsActive() && MecDrive.alreadyStrafing) {
            MecDrive.ContinueStrafing();
        }


        MecDrive.turnTo(0, Gyro);
        while (opModeIsActive() && MecDrive.alreadyTurning == true) {
            MecDrive.ContinueTurning(Gyro);
        }

        //drive toward middle of field
        Lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL);
        MecDrive.startEncoderDrive(HIGH_SPEED, (HALF_TILE_DISTANCE), (HALF_TILE_DISTANCE));
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
            Lift.ContinueLifting();
        }

        //rotate turret to deliver to High Junction
        if(ButtonConfig.startPositionMultiplier == -1){
            ServoArm.setPosition(ARM_RIGHT_OUTTAKE);}
        else if (ButtonConfig.startPositionMultiplier == 1){
            ServoArm.setPosition(ARM_LEFT_OUTTAKE);}

        //strafe to the high pole to deliver to High Junction
        MecDrive.startStrafeDrive(HIGH_SPEED,   -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier),
                                                -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier));
        while (opModeIsActive() && (MecDrive.alreadyStrafing )) {
            MecDrive.ContinueStrafing();
        }

        // Open claw to release cone
        ServoClaw.toggleClaw();

        sleep(250);

        coneDeliveryTracker = 1;
        coneStackTracker = 5;

        telemetry.addData("Cones:", "Stack(%s)/Delivered(%s)", coneStackTracker, coneDeliveryTracker);
        telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();

        //strafe away from the high pole
        MecDrive.startStrafeDrive(HIGH_SPEED,   (QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier),
                                                (QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier));
        while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
            MecDrive.ContinueStrafing();
        }

        while (coneStackTracker > 1 && runtime.seconds() < 23)
        {
            //close claw for next intake
            ServoClaw.toggleClaw();

            //move turret to pickup position
            ServoArm.setPosition(ARM_CENTER_INTAKE);

            //lower lift to correct cone stack intake height
            switch (coneStackTracker) {
                case 5: {
                    Lift.StartLifting(FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL);
                    break;
                }
                case 4: {
                    Lift.StartLifting(FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL);
                    break;
                }
                case 3: {
                    Lift.StartLifting(THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL);
                    break;
                }
                case 2: {
                    Lift.StartLifting(TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL);
                    break;
                }
                case 1: {
                    Lift.StartLifting(ONE_CONE_INTAKE_HEIGHT_ENC_VAL);
                    break;
                }
            }
            //Drive near cone stack while setting lift to correct height
            MecDrive.startEncoderDrive(HIGH_SPEED,  -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE),
                                                    -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE));
            while (opModeIsActive() && (Lift.alreadyLifting || MecDrive.alreadyDriving)) {
                MecDrive.ContinueDriving();
                Lift.ContinueLifting();
                telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
                telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.update();
            }

            //turn on intake to grab cone
            ServoIntake.toggleIntake();

            //Drive into cone with intake on
            MecDrive.startEncoderDrive(LOW_SPEED,   -(SIXTEENTH_TILE_DISTANCE+EIGHTH_TILE_DISTANCE),
                                                    -(SIXTEENTH_TILE_DISTANCE+EIGHTH_TILE_DISTANCE));
            while (opModeIsActive() && (MecDrive.alreadyDriving)) {
                MecDrive.ContinueDriving();
                telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
                telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.update();
            }

            sleep(150);

            coneStackTracker = coneStackTracker - 1;

            telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
            telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();

            //turn off intake now that cone is grabbed
            ServoIntake.toggleIntake();

            //raise lift by set amount based on current lift position
            Lift.StartLifting(Lift.liftMotor.getCurrentPosition()+ CONE_HEIGHT_ENC_VAL + CONE_CLEARANCE_HEIGHT_ENC_VAL);
            while (opModeIsActive() && Lift.alreadyLifting == true) {
                Lift.ContinueLifting();
                telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
                telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.update();
            }

            //Drive away from cone stack a little bit
            MecDrive.startEncoderDrive(LOW_SPEED, (EIGHTH_TILE_DISTANCE), (EIGHTH_TILE_DISTANCE));
            while (opModeIsActive() && MecDrive.alreadyDriving == true) {
                MecDrive.ContinueDriving();
            }

            //Turn To Absolute 0
            MecDrive.turnTo(0, Gyro);

            //Drive toward middle of field
            MecDrive.startEncoderDrive(MED_SPEED, (FULL_TILE_DISTANCE+HALF_TILE_DISTANCE), (FULL_TILE_DISTANCE+HALF_TILE_DISTANCE));
            while (opModeIsActive() && MecDrive.alreadyDriving == true) {
                MecDrive.ContinueDriving();
            }

            //move turret to deliver position
            if(ButtonConfig.startPositionMultiplier == -1){
                ServoArm.setPosition(ARM_RIGHT_OUTTAKE);}
            else if (ButtonConfig.startPositionMultiplier == 1){
                ServoArm.setPosition(ARM_LEFT_OUTTAKE);}

            //Strafe to the high pole while raising lift to height to deliver to High Junction
            MecDrive.startStrafeDrive(MED_SPEED,   -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier),
                                                    -(QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier));
            Lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL);
            while (opModeIsActive() && (Lift.alreadyLifting || MecDrive.alreadyStrafing)) {
                Lift.ContinueLifting();
                MecDrive.ContinueStrafing();
                telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
                telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.update();
            }

            //drop off cone
            ServoClaw.toggleClaw();
            coneDeliveryTracker = coneDeliveryTracker +1;

            telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
            telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();

            //strafe away from the high pole
            MecDrive.startStrafeDrive(HIGH_SPEED,   (QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier),
                                                    (QUARTER_TILE_DISTANCE * ButtonConfig.startPositionMultiplier));
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                MecDrive.ContinueStrafing();
            }

            telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
            telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }

        ServoArm.setPosition(ARM_CENTER_INTAKE);
        ServoClaw.toggleClaw();
        Lift.StartLifting(ONE_CONE_INTAKE_HEIGHT_ENC_VAL);

        //Park code
        if (Vision.currentSignal == AprilTagVision.Signal.LEFT) {
            MecDrive.startEncoderDrive(HIGH_SPEED, ((FULL_TILE_DISTANCE * ButtonConfig.startPositionMultiplier)-HALF_TILE_DISTANCE), ((FULL_TILE_DISTANCE* ButtonConfig.startPositionMultiplier) - HALF_TILE_DISTANCE));

            while (opModeIsActive() && (MecDrive.alreadyDriving || Lift.alreadyLifting)) {
                MecDrive.ContinueDriving();
                Lift.ContinueLifting();
            }
        } else if (Vision.currentSignal == AprilTagVision.Signal.MIDDLE) {
            MecDrive.startEncoderDrive(HIGH_SPEED, -HALF_TILE_DISTANCE, -HALF_TILE_DISTANCE);

            while (opModeIsActive() && (MecDrive.alreadyDriving || Lift.alreadyLifting)) {
                MecDrive.ContinueDriving();
                Lift.ContinueLifting();
            }
        } else if (Vision.currentSignal == AprilTagVision.Signal.RIGHT) {
            MecDrive.startEncoderDrive(HIGH_SPEED, ((-FULL_TILE_DISTANCE * ButtonConfig.startPositionMultiplier) - HALF_TILE_DISTANCE), ((-FULL_TILE_DISTANCE * ButtonConfig.startPositionMultiplier) - HALF_TILE_DISTANCE));

            while (opModeIsActive() && (MecDrive.alreadyDriving || Lift.alreadyLifting)) {
                MecDrive.ContinueDriving();
                Lift.ContinueLifting();
            }

            if (ButtonConfig.currentStartPosition == org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig.StartingPosition.LEFT_SIDE) {
                MecDrive.turnTo(90, Gyro);
            } else if (ButtonConfig.currentStartPosition == org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig.StartingPosition.RIGHT_SIDE) {
                MecDrive.turnTo(-90, Gyro);
            }

            while (opModeIsActive() && (MecDrive.alreadyPIDTurning)) {
                MecDrive.ContinueTurning(Gyro);
            }

            MecDrive.startEncoderDrive(HIGH_SPEED, -HALF_TILE_DISTANCE + QUARTER_TILE_DISTANCE, -HALF_TILE_DISTANCE + QUARTER_TILE_DISTANCE);
            while (opModeIsActive() && MecDrive.alreadyDriving == true) {
                MecDrive.ContinueDriving();
            }
        }

        telemetry.addData("Cones:", "Stack %s / Delivered %s", coneStackTracker, coneDeliveryTracker);
        telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();
        sleep(2000);

    }
}



