package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FIVE_CONE_STACK_INTAKE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FOUR_CONE_STACK_INTAKE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.SIXTEENTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.THREE_CONE_STACK_INTAKE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.TWO_CONE_STACK_INTAKE_HEIGHT_MM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;


@Autonomous(name = "AUTO_CTM_SCORE_6_AND_PARK")
public class AUTO_CTM_SCORE_6_AND_PARK extends LinearOpMode {

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

        //start with a cone for scoring at intake position with lift low
        ServoArm.setPosition(ARM_INTAKE);
        Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM,this);

        ButtonConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        while (!isStarted()) {
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 2;

            ButtonConfig.ConfigureAllianceColor();
            ButtonConfig.ConfigureStartingPosition();

        }

        runtime.reset();

        //align to the wall and calibrate gyro
        MecDrive.encoderDrive(HIGH_SPEED, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE, this);
        MecDrive.calibrateGyro(this);

        //drive to line up with the cone stack
        MecDrive.encoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE), (FULL_TILE_DISTANCE*2 + SIXTEENTH_TILE_DISTANCE), this);
        MecDrive.turnTo(-90, this);

        //drive toward middle of field
        MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);

        //rotate turret to deliver to High Junction
        //this code won't work if high junction is on the left.
        ServoArm.setPosition(ARM_LEFT_OUTTAKE);

        //raise lift to height to deliver to High Junction
        Lift.moveLift(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, this);

        //strafe to the high pole
        MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);

        // Open claw to release cone
        ServoClaw.toggleClaw();
        int coneDeliveryTracker = 1;

        //Give cone a moment to release;
        sleep(100);

        //Close claw for intaking next cone
        ServoClaw.toggleClaw();

        //loop
        int coneStackTracker = 5;
        while (coneStackTracker > 0 && (getRuntime() < 30.0))
        {
            //move turret to pickup position
            ServoArm.setPosition(ARM_INTAKE);

            //strafe away from the high pole
            MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
            //move to cone stack
            MecDrive.encoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE), (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE), this);
            //moving lift down
            Lift.moveLift(FIVE_CONE_STACK_INTAKE_HEIGHT_MM, this);
            //move turret to pickup position
            ServoArm.setPosition(ARM_INTAKE);
            sleep(100);
            coneStackTracker--;

            //go to high junction
        MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE), -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE), this);

            //rotate turret to deliver to High Junction
            //this code won't work if high junction is on the left.
            ServoArm.setPosition(ARM_LEFT_OUTTAKE);

            //raise lift to height to deliver to High Junction
            Lift.moveLift(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, this);

            //strafe to the high pole
            MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);


            // Open claw to release cone
            ServoClaw.toggleClaw();
            coneDeliveryTracker++;

            //Give cone a moment to release;
            sleep(100);

            //Close claw for intaking next cone
            ServoClaw.toggleClaw();


            //Loop code for picking up and delivering cones from stack
        }



        //Park code
        if(Signal == 1) {
            MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);
            MecDrive.strafeDrive(HIGH_SPEED, (HALF_TILE_DISTANCE), (HALF_TILE_DISTANCE), this);
        }
        else if (Signal == 2) {
            MecDrive.strafeDrive(HIGH_SPEED, (HALF_TILE_DISTANCE), (HALF_TILE_DISTANCE), this);
            MecDrive.encoderDrive(HIGH_SPEED, (HALF_TILE_DISTANCE), (HALF_TILE_DISTANCE), this);
        }
        else if (Signal == 3) {
            MecDrive.encoderDrive(HIGH_SPEED, (HALF_TILE_DISTANCE + FULL_TILE_DISTANCE), (HALF_TILE_DISTANCE + FULL_TILE_DISTANCE), this);
            MecDrive.strafeDrive(HIGH_SPEED, (HALF_TILE_DISTANCE), (HALF_TILE_DISTANCE), this);
        }

        //Code after Parking

        telemetry.addData("# of Cones Left on Stack", coneStackTracker);
        telemetry.addData("Current Lift Height", Lift.liftMotor.getCurrentPosition());
        telemetry.addData("# of Cones Delivered During Auto", coneDeliveryTracker);
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();
        sleep(6000);

    }
}



