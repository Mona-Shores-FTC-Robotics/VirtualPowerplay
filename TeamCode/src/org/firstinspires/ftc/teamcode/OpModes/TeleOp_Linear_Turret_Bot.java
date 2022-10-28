package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gyro;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;
import org.firstinspires.ftc.teamcode.ObjectClasses.TurnPIDController;

@TeleOp(name = "Teleop Mode w/ Turret Bot", group = "Turret Bot")
public class TeleOp_Linear_Turret_Bot extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain(this);
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    Arm ServoArm = new Arm();
    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    Lift Lift = new Lift(this);
    Gyro Gyro = new Gyro(this);

    private final ElapsedTime runtime = new ElapsedTime();

    private int teleopConeDeliveryTracker = 0;

    public void runOpMode() {

        telemetry.addData("Status", "Initializing Hardware");
        telemetry.update();

        MecDrive.init(hardwareMap);
        ServoArm.init(hardwareMap);
        ServoIntake.init(hardwareMap);
        ServoClaw.init(hardwareMap);
        Lift.init(hardwareMap);
        Gyro.init(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Gyro.calibrateGyro();

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData("Status", "Configuring Buttons");
            ButtonConfig.ConfigureMultiplier(this, MecDrive);
            ButtonConfig.ConfigureLiftMultiplier(this, Lift);
            telemetry.addData("Status", "Press START once multipliers are set");
            telemetry.update();
        }

        runtime.reset();

        while (opModeIsActive()) {

            //Store the previous loop's gamepad values.
            previousGamepad1 = ButtonConfig.copy(currentGamepad1);
            previousGamepad2 = ButtonConfig.copy(currentGamepad2);

            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = ButtonConfig.copy(gamepad1);
            currentGamepad2 = ButtonConfig.copy(gamepad2);

            ServoClaw.CheckClaw(currentGamepad2.a, previousGamepad2.a);

            ServoIntake.CheckIntake(currentGamepad2.x, previousGamepad2.x);

            Lift.CheckLift(     currentGamepad2.left_bumper, previousGamepad2.left_bumper,
                                currentGamepad2.right_bumper, previousGamepad2.right_bumper,
                                currentGamepad2.left_stick_y);

            ServoArm.CheckArm(  currentGamepad2.dpad_left, previousGamepad2.dpad_left,
                                currentGamepad2.dpad_down, previousGamepad2.dpad_down,
                                currentGamepad2.dpad_right, previousGamepad2.dpad_right);

            MecDrive.CheckDriveControls( currentGamepad1, previousGamepad1, Lift, ServoArm, ServoClaw, ServoIntake, Gyro);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Run Time:","%s", runtime);
            telemetry.addData("Motors", "leftfront(%.2f), rightfront (%.2f)", MecDrive.leftFrontPower*MecDrive.multiplier, MecDrive.rightFrontPower*MecDrive.multiplier);
            telemetry.addData("Motors", "leftback (%.2f), rightback (%.2f)", MecDrive.leftBackPower*MecDrive.multiplier, MecDrive.rightBackPower*MecDrive.multiplier);
            telemetry.addData("Encoders" , "leftfront(%s), rightfront(%s)", MecDrive.LFDrive.getCurrentPosition(), MecDrive.RFDrive.getCurrentPosition());
            telemetry.addData("Encoders", "leftback(%s), rightback(%s)", MecDrive.LBDrive.getCurrentPosition(), MecDrive.RBDrive.getCurrentPosition());
            telemetry.addData("Speed", "(%.4f)", MecDrive.ramp);

            telemetry.addData("Lift", "Position(%s), Target(%s)", Lift.liftMotor.getCurrentPosition(), Lift.newLiftTarget);
            telemetry.addData("Arm Position", ServoArm.currentArmState);
            telemetry.addData("Claw Position", ServoClaw.currentClawState);
            telemetry.addData("Intake State", ServoIntake.currentIntakeState);
            telemetry.addData("Absolute Gyro Angle", (int) Gyro.getAbsoluteAngle());
            telemetry.addData("Target PID Angle", (int) MecDrive.pid.targetPIDAngle);
            telemetry.addData("PID Angle Left to Turn", (int) MecDrive.pid.pidAngleLeftToTurn);
            telemetry.addData("Degrees Left to Turn:", "(%.2f)", abs(MecDrive.degreesLeftToTurn));

            telemetry.addData("# of Cones Delivered", teleopConeDeliveryTracker);
            telemetry.update();
        }
        MecDrive.drive = 0;
        MecDrive.strafe = 0;
        MecDrive.turn = 0;
        MecDrive.MecanumDrive();
    }
}






