package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;


@Autonomous(name = "AUTO_JUST_PARK")
public class AUTO_JUST_PARK extends LinearOpMode {

    // Variable to store the Signal

    DriveTrain MecDrive = new DriveTrain();
    //AprilTagVision Vision = new AprilTagVision();
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    Arm ServoArm = new Arm();
    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    Lift Lift = new Lift();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        //Vision.init(hardwareMap);
        ButtonConfig.init();

        ServoArm.init(hardwareMap);
        ServoIntake.init(hardwareMap);
        ServoClaw.init(hardwareMap);
        //Lift.init(hardwareMap);
        //Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM, this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(500);

        while (!isStarted()) {
            //Use Webcam to find out Signal using April Tags
            //Vision.CheckForAprilTags(this);

            // Let the user set alliance color and starting location variables for use in code
            ButtonConfig.ConfigureAllianceColor();
            ButtonConfig.ConfigureStartingPosition();
            telemetry.addData("Alliance Color ", ButtonConfig.currentAllianceColor);
            telemetry.addData("Starting Position ", ButtonConfig.currentStartPosition);
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
            sleep(20);
        }

        resetStartTime();
        currentSignal = Signal.RIGHT;
        //Vision.SetSignal(this);
        telemetry.addData("Signal is ", currentSignal);
        telemetry.addData("Selected Alliance Color ", ButtonConfig.currentAllianceColor);
        telemetry.addData("Selected Starting Position ", ButtonConfig.currentStartPosition);
        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();

        //Drive backwards into wall to make sure we are aligned
        MecDrive.startEncoderDrive(MED_SPEED, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE, this);
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving(this);
        }

        //Drive forward 2 tiles plus a little bit more to get into position for deciding where to park
        MecDrive.startEncoderDrive(MED_SPEED, FULL_TILE_DISTANCE * 2 + SIXTEENTH_TILE_DISTANCE, FULL_TILE_DISTANCE * 2 + SIXTEENTH_TILE_DISTANCE, this);
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving(this);
        }

        //Decide where to park
        //if current Signal is the LEFT april tag then park on robot's left
        if (currentSignal == Signal.LEFT) {
            //Park on left
            MecDrive.startStrafeDrive(MED_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                MecDrive.ContinueStrafing(this);
            }
        }

        //if current Signal is the MIDDLE april tag then park in middle
        else if (currentSignal == Signal.MIDDLE) {
            //Park in middle
            }

        //if current Signal is the RIGHT april tag then park on robot's right
        else if (currentSignal == Signal.RIGHT) {
            //Park on right
            MecDrive.startStrafeDrive(MED_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);
            while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
                    MecDrive.ContinueStrafing(this);
            }

            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
            }
        }
    }




