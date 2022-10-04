package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.CONE_INTAKE_HEIGHT_CHANGE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.W_3_JUNCTION;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.X_2_JUNCTION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */

@TeleOp(name = "Teleop Mode w/ Turret WITH CANCEL", group = "TurretBot")
public class TeleOp_Linear_Turret_Bot_WITH_CANCEL extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain();
    org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig ButtonConfig = new ButtonConfig();
    private final ElapsedTime runtime = new ElapsedTime();
    public static final double TURRET_INTAKE = 0.0;
    public static final double TURRET_INTAKE2 = 180.0 / 360.0;
    public static final double TURRET_LEFT_OUTTAKE = 270.0 / 360.0;
    public static final double TURRET_RIGHT_OUTTAKE = 90.0 / 360.0;
    private Servo turret;
    private Servo lift;
    private ColorSensor colorSensor;
    private int teleopConeDeliveryTracker = 0;

    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        turret = hardwareMap.servo.get("turret_servo");
        lift = hardwareMap.servo.get("elevation_servo");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

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

            // MecDrive.drive = -gamepad1.left_stick_y; //-1.0 to 1.0
            //MecDrive.strafe = gamepad1.right_trigger-gamepad1.left_trigger; //-1.0 to 1.0 // right trigger strafe right, left trigger strafe left
            //MecDrive.turn  =  gamepad1.right_stick_x; //-1.0 to 1.0

            MecDrive.MecanumDrive();

            if (gamepad1.x) {
                turret.setPosition(TURRET_LEFT_OUTTAKE);
            } else if (gamepad1.b) {
                turret.setPosition(TURRET_RIGHT_OUTTAKE);
            }

            if (gamepad1.y) {
                auto_deliver(W_3_JUNCTION);
                teleopConeDeliveryTracker = teleopConeDeliveryTracker + 1;
            } else if (gamepad1.a) {
                auto_deliver(X_2_JUNCTION);
                teleopConeDeliveryTracker = teleopConeDeliveryTracker + 1;
            }

            if (gamepad1.left_bumper) {
                lift.setPosition(ONE_CONE_INTAKE_HEIGHT);
            }

            if (gamepad1.right_bumper) {
                lift.setPosition(HIGH_CONE_JUNCTION_SCORE_HEIGHT);
            }

            if (gamepad1.dpad_up){
                MecDrive.turnTo(0, this);
                MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this );
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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Motors", "leftfront(%.2f), rightfront (%.2f)", MecDrive.leftFrontPower, MecDrive.rightFrontPower);
            telemetry.addData("Motors", "leftback (%.2f), rightback (%.2f)", MecDrive.leftBackPower, MecDrive.rightBackPower);
            telemetry.addData("Color", "R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("# of Cones Delivered", teleopConeDeliveryTracker);
            telemetry.update();
        }

        MecDrive.drive = 0;
        MecDrive.strafe = 0;
        MecDrive.turn = 0;

        MecDrive.MecanumDrive();
    }

    public void auto_deliver(int deliveryDestination) {

        boolean stopExecution = false;

        MecDrive.turnTo(0, this);

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
            turret.setPosition(TURRET_INTAKE2);
        } else stopExecution = true;

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
            MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE + QUARTER_TILE_DISTANCE), -(FULL_TILE_DISTANCE + QUARTER_TILE_DISTANCE), this);
        } else stopExecution = true;

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
            //lower lift by set amount based on current lift position
            lift.setPosition(lift.getPosition() - CONE_INTAKE_HEIGHT_CHANGE);
        } else stopExecution = true;

        //activate intake to grab cone

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
            //raise lift by set amount based on current lift position
            lift.setPosition(lift.getPosition() + CONE_INTAKE_HEIGHT_CHANGE);
        } else stopExecution = true;

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
            //raise lift to height to deliver to High Junction
            lift.setPosition(MEDIUM_CONE_JUNCTION_SCORE_HEIGHT);
        } else stopExecution = true;

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
            //Drive toward middle of field
            if (deliveryDestination == W_3_JUNCTION) {
                MecDrive.encoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
            } else if (deliveryDestination == X_2_JUNCTION) {
                MecDrive.encoderDrive(HIGH_SPEED, ((FULL_TILE_DISTANCE * 2) + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), ((FULL_TILE_DISTANCE * 2) + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
            }
        } else stopExecution = true;

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
            //raise lift to height to deliver to High Junction
            lift.setPosition(HIGH_CONE_JUNCTION_SCORE_HEIGHT);
        } else stopExecution = true;

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {

            if (deliveryDestination == W_3_JUNCTION) {
                //move turret to deliver position
                turret.setPosition(TURRET_LEFT_OUTTAKE);
            } else if (deliveryDestination == X_2_JUNCTION) {
                turret.setPosition(TURRET_RIGHT_OUTTAKE);
            }
        } else stopExecution = true;

        if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {

            if (deliveryDestination == W_3_JUNCTION) {
                //Strafe left to the W 3 HIGH JUNCTION
                MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);
            } else if (deliveryDestination == X_2_JUNCTION) {
                //Strafe right to the W 3 HIGH JUNCTION
                MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
            } else stopExecution = true;

            //drop off cone

            if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {

                if (deliveryDestination == W_3_JUNCTION) {
                    //strafe away from the W3 JUNCTION
                    MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
                } else if (deliveryDestination == X_2_JUNCTION) {
                    //strafe away from the X2 JUNCTION
                    MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);
                }
            } else stopExecution = true;

            if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
                //lower lift for next cone
                lift.setPosition(ONE_CONE_INTAKE_HEIGHT);
            } else stopExecution = true;

            if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
                //rotate turret for next cone
                turret.setPosition(TURRET_INTAKE2);
            } else stopExecution = true;

            if (!gamepad1.right_bumper && !stopExecution && opModeIsActive()) {
                if (deliveryDestination == W_3_JUNCTION) {
                    //Drive to Alliance Station
                    MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), -(HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
                } else if (deliveryDestination == X_2_JUNCTION) {
                    MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), this);
                }
            } else stopExecution = true;

        }
    }

}






