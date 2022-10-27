package com.acmerobotics.dashboard.config.disabled_samples;


import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_RIGHT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.CONE_INTAKE_HEIGHT_CHANGE_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.W_3_JUNCTION;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.X_2_JUNCTION;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;

import java.awt.Robot;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */

@TeleOp(name = "Teleop Mode w/ Turret Bot STATEMACHINE", group = "Turret Bot")
@Disabled
public class TeleOp_Linear_Turret_Bot_STATEMACHINE extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    Arm ServoArm = new Arm();
    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    org.firstinspires.ftc.teamcode.ObjectClasses.Lift Lift = new Lift();

    public DriveTrain.robotState currentRobotState;

    private final ElapsedTime runtime = new ElapsedTime();

    private int teleopConeDeliveryTracker = 0;

    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        ServoArm.init(hardwareMap);
        ServoIntake.init(hardwareMap);
        ServoClaw.init(hardwareMap);
        Lift.init(hardwareMap);
        Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM,this);

        currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;
        boolean aToggleReady = false;
        boolean xToggleReady = false;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Press Start When Ready", "");

        while (!isStarted()) {
            ButtonConfig.ConfigureMultiplier(this, MecDrive);
        }

        runtime.reset();

        while (opModeIsActive()) {

                MecDrive.drive = -gamepad1.left_stick_y; //-1.0 to 1.0
                MecDrive.strafe = gamepad1.left_stick_x; //-1.0 to 1.0
                MecDrive.turn = gamepad1.right_stick_x; //-1.0 to 1.0

                MecDrive.MecanumDrive();

                boolean G1a = gamepad1.a;
                boolean G1x = gamepad1.x;

                if (G1a == false) {
                    aToggleReady = true;
                }

                if (G1x == false) {
                    xToggleReady = true;
                }

                if (G1a && aToggleReady) {
                    aToggleReady = false;
                    ServoArm.setPosition(ARM_LEFT_OUTTAKE);
                    sleep(100);
                }

                if (gamepad1.y) {
                    ServoArm.setPosition(ARM_INTAKE);
                    sleep(100);
                }
                if (gamepad1.b) {
                    ServoArm.setPosition(ARM_RIGHT_OUTTAKE);
                    sleep(100);
                }

                if (gamepad1.right_trigger > 0) {
                    ServoIntake.toggleIntake();
                    sleep(100);
                }

                if (G1x && xToggleReady) {
                    xToggleReady = false;
                    ServoClaw.toggleClaw();
                    sleep(100);
                }

                if (gamepad1.left_bumper) {
                    //Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM, this);
                }

                if (gamepad1.right_bumper) {
                    //Lift.moveLift(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, this);
                }

                if (gamepad1.dpad_up) {
                    MecDrive.turnTo(0, this);
                    MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);
                }
                if (gamepad1.dpad_down) {
                    MecDrive.turnTo(0, this);
                    MecDrive.encoderDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);
                }
                if (gamepad1.dpad_left) {
                    MecDrive.turnTo(0, this);
                    MecDrive.strafeDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);
                }
                if (gamepad1.dpad_right) {
                    MecDrive.turnTo(0, this);
                    MecDrive.strafeDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);
                }

                if (gamepad1.right_stick_button) {
                    currentRobotState = DriveTrain.robotState.AUTOMATIC_TASK;
                    Auto_Deliver(X_2_JUNCTION);
                    currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("Motors", "leftfront(%.2f), rightfront (%.2f)", MecDrive.leftFrontPower, MecDrive.rightFrontPower);
                telemetry.addData("Motors", "leftback (%.2f), rightback (%.2f)", MecDrive.leftBackPower, MecDrive.rightBackPower);
                telemetry.addData("# of Cones Delivered", teleopConeDeliveryTracker);
                telemetry.update();
            }


        MecDrive.drive = 0;
        MecDrive.strafe = 0;
        MecDrive.turn = 0;

        MecDrive.MecanumDrive();
    }



    public void Auto_Deliver(int deliveryDestination) {



        MecDrive.turnTo(0, this);

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            ServoArm.setPosition(ARM_INTAKE);
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            MecDrive.encoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE + QUARTER_TILE_DISTANCE), (FULL_TILE_DISTANCE + QUARTER_TILE_DISTANCE), this);
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            //lower lift by set amount based on current lift position
            Lift.moveLift(Lift.liftMotor.getCurrentPosition() - CONE_INTAKE_HEIGHT_CHANGE_MM, this);
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        //activate intake to grab cone

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            //raise lift by set amount based on current lift position
            Lift.moveLift(Lift.liftMotor.getCurrentPosition() + CONE_INTAKE_HEIGHT_CHANGE_MM, this);
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            //raise lift to height to deliver to High Junction
            Lift.moveLift(MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_MM, this);
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            //Drive toward middle of field
            if (deliveryDestination == W_3_JUNCTION) {
                MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
            } else if (deliveryDestination == X_2_JUNCTION) {
                MecDrive.encoderDrive(HIGH_SPEED, -((FULL_TILE_DISTANCE * 2) + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), -((FULL_TILE_DISTANCE * 2) + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
            }
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            //raise lift to height to deliver to High Junction
            Lift.moveLift(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, this);
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            if (deliveryDestination == W_3_JUNCTION) {
                //move turret to deliver position
                ServoArm.setPosition(ARM_RIGHT_OUTTAKE);
            } else if (deliveryDestination == X_2_JUNCTION) {
                ServoArm.setPosition(ARM_LEFT_OUTTAKE);
            }
        } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
            if (deliveryDestination == W_3_JUNCTION) {
                //Strafe left to the W 3 HIGH JUNCTION
                MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
            } else if (deliveryDestination == X_2_JUNCTION) {
                //Strafe right to the W 3 HIGH JUNCTION
                MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);
            } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

            //drop off cone
            ServoClaw.toggleClaw();
            sleep(200);
            ServoClaw.toggleClaw();


        if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {

                if (deliveryDestination == W_3_JUNCTION) {
                    //strafe away from the W3 JUNCTION
                    MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);
                } else if (deliveryDestination == X_2_JUNCTION) {
                    //strafe away from the X2 JUNCTION
                    MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
                }
            } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

            if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
                //lower lift for next cone
                Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM, this);
            } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

            if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
                //rotate turret for next cone
                ServoArm.setPosition(ARM_INTAKE);
            } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

            if (!gamepad1.b && currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK && opModeIsActive()) {
                if (deliveryDestination == W_3_JUNCTION) {
                    //Drive to Alliance Station
                    MecDrive.encoderDrive(HIGH_SPEED, (HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), (HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
                } else if (deliveryDestination == X_2_JUNCTION) {
                    MecDrive.encoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
                }
            } else currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        }
    }

}





