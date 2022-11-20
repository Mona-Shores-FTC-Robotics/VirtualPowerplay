package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.CONE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE_DRIVE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE_STRAFE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE_DRIVE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ObjectClasses.AprilTagVision;
import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;


@Autonomous(name = "AUTO_JUST_PARK")
public class AUTO_JUST_PARK extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain(this);
    AprilTagVision Vision = new AprilTagVision();
    ButtonConfig BConfig = new ButtonConfig(this);
    Claw ServoClaw = new Claw();
    Intake ServoIntake = new Intake(ServoClaw, this);
    Lift Lift = new Lift(this);
    Arm ServoArm = new Arm(Lift, ServoIntake, ServoClaw, this);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        Vision.init(hardwareMap);
        Lift.init(hardwareMap);
        ServoIntake.init(hardwareMap);
        ServoClaw.init(hardwareMap);
        ServoArm.init(hardwareMap);
        BConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(500);

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
            BConfig.ConfigureStartingPosition( currentGamepad1.dpad_left, previousGamepad1.dpad_left,
                    currentGamepad1.dpad_right, previousGamepad1.dpad_right,
                    currentGamepad1.b,          previousGamepad1.b);

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

        telemetry.addData("Signal is ", Vision.currentSignal);
        telemetry.addData("Selected Starting Position ", ButtonConfig.currentStartPosition);
        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();

        //Drive forward 2 tiles plus a little bit more to get into position for deciding where to park
        Lift.StartLifting(CONE_HEIGHT_ENC_VAL, ServoArm);
        MecDrive.startEncoderDrive(MED_SPEED, FULL_TILE_DISTANCE_DRIVE * 2 + QUARTER_TILE_DISTANCE_DRIVE);
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }

        //Decide where to park

        //if current Signal is the LEFT april tag then park on robot's left
        if (Vision.currentSignal == AprilTagVision.Signal.LEFT) {
            //Park on left
            MecDrive.startStrafeDrive(MED_SPEED, -FULL_TILE_DISTANCE_STRAFE);
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                MecDrive.ContinueStrafing();
            }
        }

        //if current Signal is the MIDDLE april tag then park in middle
        else if (Vision.currentSignal == AprilTagVision.Signal.MIDDLE) {
            //Park in middle
            }

        //if current Signal is the RIGHT april tag then park on robot's right
        else if (Vision.currentSignal == AprilTagVision.Signal.RIGHT) {
            //Park on right
            MecDrive.startStrafeDrive(MED_SPEED, FULL_TILE_DISTANCE_STRAFE);
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                    MecDrive.ContinueStrafing();
            }

            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
            }
        }
    }




