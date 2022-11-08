package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_CENTER_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_RIGHT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.MED_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig.*;

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

import java.awt.Button;


@Autonomous(name = "AUTO_SCORE_ONE_AND_PARK")
public class AUTO_SCORE_ONE_AND_PARK extends LinearOpMode {

    int Signal;
    DriveTrain MecDrive = new DriveTrain(this);
    ButtonConfig ButtonConfig = new ButtonConfig(this);

    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    Lift Lift = new Lift(this);
    Arm ServoArm = new Arm(Lift);
    Gyro Gyro = new Gyro(this);

    private final ElapsedTime runtime = new ElapsedTime();

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

        //start with a cone for scoring at intake position with lift low
        ServoArm.setArmState(Arm.armState.ARM_CENTER);
        Lift.StartLifting(ONE_CONE_INTAKE_HEIGHT_ENC_VAL);
        while (opModeIsActive() && Lift.alreadyLifting == true) {
            Lift.ContinueLifting();
        }

        ButtonConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        while (!isStarted()) {
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 3;
            ButtonConfig.ConfigureAllianceColor();
            ButtonConfig.ConfigureStartingPosition();
            telemetry.addData("Alliance Color ", ButtonConfig.currentAllianceColor);
            telemetry.addData("Starting Position ", ButtonConfig.currentStartPosition);
            telemetry.update();
            sleep(20);
        }

        runtime.reset();

        //Vision.SetSignal(this);
        telemetry.addData("Signal is ", Signal);
        telemetry.addData("Selected Alliance Color ", ButtonConfig.currentAllianceColor);
        telemetry.addData("Selected Starting Position ", ButtonConfig.currentStartPosition);
        telemetry.update();

        //Drive Forward
        MecDrive.startEncoderDrive(MED_SPEED, (FULL_TILE_DISTANCE*2)+EIGHTH_TILE_DISTANCE, (FULL_TILE_DISTANCE*2)+EIGHTH_TILE_DISTANCE);
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }

        //Rotate
        if ((ButtonConfig.currentAllianceColor == AllianceColor.BLUE && ButtonConfig.currentStartPosition == StartPosition.ROW_2) ||
            (ButtonConfig.currentAllianceColor == AllianceColor.RED && ButtonConfig.currentStartPosition == StartPosition.ROW_5))
            {
                MecDrive.turnPID(90, Gyro);
                while (opModeIsActive() && MecDrive.alreadyPIDTurning == true) {
                    MecDrive.ContinuePIDTurning(Gyro);
                }
            }
        else
            {
                MecDrive.turnPID(-90, Gyro);
                while (opModeIsActive() && MecDrive.alreadyPIDTurning == true) {
                    MecDrive.ContinuePIDTurning(Gyro);
                }
            }

        //Drive in Front of High Pole
        MecDrive.startEncoderDrive(MED_SPEED, HALF_TILE_DISTANCE, HALF_TILE_DISTANCE);
        Lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL);
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
            Lift.ContinueLifting();
        }

        //Strafe close to High Pole
        MecDrive.startStrafeDrive(MED_SPEED, EIGHTH_TILE_DISTANCE* ButtonConfig.allianceColorAndLocationFactor, EIGHTH_TILE_DISTANCE*ButtonConfig.allianceColorAndLocationFactor);
        if ((ButtonConfig.currentAllianceColor == AllianceColor.BLUE && ButtonConfig.currentStartPosition == StartPosition.ROW_2) ||
                (ButtonConfig.currentAllianceColor == AllianceColor.RED && ButtonConfig.currentStartPosition == StartPosition.ROW_5)) {
            ServoArm.setPosition(ARM_RIGHT_OUTTAKE);
        } else ServoArm.setPosition(ARM_LEFT_OUTTAKE);
        while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
            MecDrive.ContinueStrafing();
        }

        //Open claw to drop cone
        ServoClaw.toggleClaw();
        sleep (400);

        //Strafe away from High Pole
        MecDrive.startStrafeDrive(MED_SPEED, -EIGHTH_TILE_DISTANCE* ButtonConfig.allianceColorAndLocationFactor,
                                            -EIGHTH_TILE_DISTANCE* ButtonConfig.allianceColorAndLocationFactor);
        while (opModeIsActive() && MecDrive.alreadyStrafing == true) {
            MecDrive.ContinueStrafing();
        }

        //close the claw
        ServoClaw.toggleClaw();
        ServoArm.setPosition(ARM_CENTER_INTAKE);
        Lift.StartLifting(FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL);

        //Park after placing cone
        if (Signal == 1) {
            MecDrive.startEncoderDrive(.5, ((FULL_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor)-HALF_TILE_DISTANCE), ((FULL_TILE_DISTANCE* ButtonConfig.allianceColorAndLocationFactor) - HALF_TILE_DISTANCE));
        } else if (Signal == 2) {
            MecDrive.startEncoderDrive(.5, -HALF_TILE_DISTANCE, -HALF_TILE_DISTANCE);
        } else if (Signal == 3) {
            MecDrive.startEncoderDrive(.5, ((-FULL_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor) -HALF_TILE_DISTANCE), ((-FULL_TILE_DISTANCE * ButtonConfig.allianceColorAndLocationFactor) -HALF_TILE_DISTANCE));
        }
        while (opModeIsActive() && MecDrive.alreadyDriving == true) {
            MecDrive.ContinueDriving();
        }


        telemetry.addData("Signal is ", Signal);
        telemetry.addData("Selected Alliance Color ", ButtonConfig.currentAllianceColor);
        telemetry.addData("Selected Starting Position ", ButtonConfig.currentStartPosition);
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.update();
        sleep(6000);

    }
}



