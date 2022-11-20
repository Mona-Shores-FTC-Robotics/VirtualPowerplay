package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE_STRAFE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE_DRIVE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE_DRIVE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE_STRAFE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.SIXTEENTH_TILE_DISTANCE_DRIVE;

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


@Autonomous(name = "AUTO_SCORE_ONE_AND_PARK")
public class AUTO_SCORE_ONE_AND_PARK extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain(this);
    ButtonConfig BConfig = new ButtonConfig(this);

    AprilTagVision Vision = new AprilTagVision();
    Claw ServoClaw = new Claw();
    Intake ServoIntake = new Intake(ServoClaw, this);
    Lift Lift = new Lift(this);
    Arm ServoArm = new Arm(Lift, ServoIntake, ServoClaw, this);
    Gyro Gyro = new Gyro(this);

    private final ElapsedTime runtime = new ElapsedTime();

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        ServoIntake.init(hardwareMap);
        ServoClaw.init(hardwareMap);
        Lift.init(hardwareMap);
        ServoArm.init(hardwareMap);

        Vision.init(hardwareMap);
        BConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        while (!isStarted()) {
            //save current and previous gamepad values for one loop
            previousGamepad1 = BConfig.copy(currentGamepad1);
            currentGamepad1 = BConfig.copy(gamepad1);

            previousGamepad2 = BConfig.copy(currentGamepad2);
            currentGamepad2 = BConfig.copy(gamepad2);

            //Use Webcam to find out Signal using April Tags and save in currentSignal
            Vision.CheckForAprilTags(this);

            // User sets starting location left or right, and confirms selection with a button press
            // LEFT is a multiplier of 1, RIGHT is a multiplier of -1
            BConfig.ConfigureStartingPosition(currentGamepad1.dpad_left, previousGamepad1.dpad_left,
                    currentGamepad1.dpad_right, previousGamepad1.dpad_right,
                    currentGamepad1.b, previousGamepad1.b);

            telemetry.addData("Signal", "Signal(%s), Number(%s)", Vision.currentSignal, Vision.currentSignalNumber);
            telemetry.addLine(" ");
            telemetry.addLine("Select Starting Position with D-pad");
            telemetry.addData("Current Starting Position ", ButtonConfig.currentStartPosition);
            if (ButtonConfig.confirmStartingPositionSelection == false) {
                telemetry.addData("Unlocked", "Press B to lock selection");
            } else {
                telemetry.addData("Locked", "Press B to unlock selection");
            }
            telemetry.update();

            sleep(20);
        }

        runtime.reset();
        Gyro.init(hardwareMap);

        telemetry.addData("Signal is ", Vision.currentSignal);
        telemetry.addData("Selected Starting Position ", ButtonConfig.currentStartPosition);
        telemetry.update();

        //Drive Forward
        Lift.StartLifting(MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, ServoArm);
        MecDrive.startEncoderDrive(.4, (FULL_TILE_DISTANCE_DRIVE * 2) + EIGHTH_TILE_DISTANCE_STRAFE+SIXTEENTH_TILE_DISTANCE_DRIVE);
        while (opModeIsActive() && (Lift.alreadyLifting || MecDrive.alreadyDriving)) {
            MecDrive.ContinueDriving();
            Lift.ContinueLifting();
        }

        //Rotate
        //turn to 90
        MecDrive.turnToPID(-90* ButtonConfig.startPositionMultiplier, Gyro);
        while (opModeIsActive() && (MecDrive.alreadyPIDTurning)) {
            MecDrive.ContinuePIDTurning(Gyro);
        }

        //Drive in Front of High Pole
        MecDrive.startEncoderDrive(.4, HALF_TILE_DISTANCE_DRIVE);
        Lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, ServoArm);
        while (opModeIsActive() && (Lift.alreadyLifting || MecDrive.alreadyDriving)) {
            MecDrive.ContinueDriving();
            Lift.ContinueLifting();
        }

        //Strafe close to High Pole
        MecDrive.startStrafeDrive(LOW_SPEED, -(QUARTER_TILE_DISTANCE_STRAFE+EIGHTH_TILE_DISTANCE_STRAFE) * ButtonConfig.startPositionMultiplier);
        if ((ButtonConfig.currentStartPosition == ButtonConfig.StartingPosition.RIGHT_SIDE)) {
            ServoArm.setArmState(Arm.armState.ARM_RIGHT);
        } else ServoArm.setArmState(Arm.armState.ARM_LEFT);
        while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
            MecDrive.ContinueStrafing();
        }

        sleep(300);

        Lift.StartLifting(Lift.liftMotor.getCurrentPosition()-400, ServoArm);
        while (opModeIsActive() && (Lift.alreadyLifting)) {
            Lift.ContinueLifting();
        }

        //Open claw to drop cone
        ServoClaw.openClaw();

        sleep(300);

        Lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL+100, ServoArm);
        while (opModeIsActive() && (Lift.alreadyLifting)) {
            Lift.ContinueLifting();
        }

        //Strafe away from High Pole
        MecDrive.startStrafeDrive(LOW_SPEED, (QUARTER_TILE_DISTANCE_STRAFE) * ButtonConfig.startPositionMultiplier);
        while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                MecDrive.ContinueStrafing();
        }

        //turn to 90
        MecDrive.turnToPID(-90* ButtonConfig.startPositionMultiplier, Gyro);
        while (opModeIsActive() && (MecDrive.alreadyPIDTurning)) {
            MecDrive.ContinuePIDTurning(Gyro);
        }

        //close the claw
        ServoClaw.closeClaw();
        ServoArm.setArmState(Arm.armState.ARM_CENTER);

        Lift.StartLifting(FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, ServoArm);
        //Park after placing cone
        if (Vision.currentSignal == AprilTagVision.Signal.LEFT) {
            MecDrive.startEncoderDrive(LOW_SPEED,
                    -(FULL_TILE_DISTANCE_DRIVE * ButtonConfig.startPositionMultiplier) - HALF_TILE_DISTANCE_DRIVE);
        } else if (Vision.currentSignal == AprilTagVision.Signal.MIDDLE) {
            MecDrive.startEncoderDrive(LOW_SPEED,
                    (HALF_TILE_DISTANCE_DRIVE)*ButtonConfig.startPositionMultiplier);
        } else if (Vision.currentSignal == AprilTagVision.Signal.RIGHT) {
            MecDrive.startEncoderDrive(LOW_SPEED,
                    (FULL_TILE_DISTANCE_DRIVE * ButtonConfig.startPositionMultiplier) - HALF_TILE_DISTANCE_DRIVE);
        }
        while (opModeIsActive() && (Lift.alreadyLifting || MecDrive.alreadyDriving == true)) {
            MecDrive.ContinueDriving();
            Lift.ContinueLifting();
        }

            telemetry.addData("Signal is ", Vision.currentSignal);
            telemetry.addData("Selected Starting Position ", ButtonConfig.currentStartPosition);
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }



