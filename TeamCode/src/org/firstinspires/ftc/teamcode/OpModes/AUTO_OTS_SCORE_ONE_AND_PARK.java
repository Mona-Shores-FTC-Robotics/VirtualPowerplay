package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
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
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.currentSignal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;


@Autonomous(name = "AUTO_OTS_SCORE_ONE_AND_PARK")
public class AUTO_OTS_SCORE_ONE_AND_PARK extends LinearOpMode {

    int Signal;
    DriveTrain MecDrive = new DriveTrain();
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
            telemetry.addData("Alliance Color ", ButtonConfig.currentAllianceColor);
            telemetry.addData("Starting Position ", ButtonConfig.currentStartPosition);
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
            sleep(20);
        }

        resetStartTime();

        //Vision.SetSignal(this);
        telemetry.addData("Signal is ", Signal);
        telemetry.addData("Selected Alliance Color ", ButtonConfig.currentAllianceColor);
        telemetry.addData("Selected Starting Position ", ButtonConfig.currentStartPosition);
        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();

        //align to the wall and calibrate gyro
        MecDrive.encoderDrive(MED_SPEED, -QUARTER_TILE_DISTANCE, -QUARTER_TILE_DISTANCE, this);
        MecDrive.calibrateGyro(this);

        //Drive Forward
        MecDrive.encoderDrive(MED_SPEED, (FULL_TILE_DISTANCE*2)+EIGHTH_TILE_DISTANCE, (FULL_TILE_DISTANCE*2)+EIGHTH_TILE_DISTANCE, this);

        //Strafe in Front of High Pole
        MecDrive.strafeDrive(MED_SPEED, -(HALF_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), -(HALF_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), this);

        //Drive close to High Pole
        MecDrive.encoderDrive(MED_SPEED, EIGHTH_TILE_DISTANCE, EIGHTH_TILE_DISTANCE, this);

        //Open claw to drop cone
        ServoClaw.toggleClaw();
        sleep(250);
        ServoClaw.toggleClaw();

        //Back away from High Pole
        MecDrive.encoderDrive(MED_SPEED, -EIGHTH_TILE_DISTANCE, -EIGHTH_TILE_DISTANCE, this);

        //Park after placing cone
        if (Signal == 1) {
            MecDrive.strafeDrive(.5, (-FULL_TILE_DISTANCE + (HALF_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor)), (-FULL_TILE_DISTANCE + (HALF_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor)), this);
        } else if (Signal == 2) {
            MecDrive.strafeDrive(.5, (HALF_TILE_DISTANCE* ButtonConfig.allianceColorAndLocationFactor), (HALF_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor), this);
        } else if (Signal == 3) {
            MecDrive.strafeDrive(.5, (FULL_TILE_DISTANCE + (HALF_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor)), (FULL_TILE_DISTANCE + (HALF_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor)), this);
        }

        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();
        sleep(6000);

    }
}



