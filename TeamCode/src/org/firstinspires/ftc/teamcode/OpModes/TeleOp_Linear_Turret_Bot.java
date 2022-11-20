package org.firstinspires.ftc.teamcode.OpModes;

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

@TeleOp(name = "TeleOp Mode", group = "Turret Bot")
public class TeleOp_Linear_Turret_Bot extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain(this);
    ButtonConfig ButtonConfig = new ButtonConfig(this);

    Claw ServoClaw = new Claw();
    Intake ServoIntake = new Intake(ServoClaw, this);
    Lift Lift = new Lift(this);
    Arm ServoArm = new Arm(Lift, ServoIntake, ServoClaw, this);
    Gyro Gyro = new Gyro(this);
    //PipeVision AutoVision = new PipeVision(this, MecDrive);

    private final ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        telemetry.addData("Status", "Initializing Hardware");
        telemetry.update();

        MecDrive.init(hardwareMap);
        ServoArm.init(hardwareMap);
        ServoIntake.init(hardwareMap);
        ServoClaw.init(hardwareMap);
        Lift.init(hardwareMap);
        Gyro.init(hardwareMap);
       //AutoVision.init(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1;
        Gamepad previousGamepad2;

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData("Status", "Configuring Buttons");
            ButtonConfig.ConfigureMultiplier(this, MecDrive);
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

            //------------------------------------------------------//
            //--------------------RUMBLE FEATURES-------------------//
            //------------------------------------------------------//

            /**
            Alert driver 5 seconds until END GAME
             */
            if (runtime.seconds() > 84 && runtime.seconds() < 85) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }

            /**
            Alert driver 5 seconds until GAME END
             */

            if (runtime.seconds() > 114 && runtime.seconds() < 115) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }

            //------------------------------------------------------//
            //--------------------OPERATOR CONTROLS-----------------//
            //------------------------------------------------------//

            /**
            Y button opens claw on press
            One second after release, claw closes and arm automatically centers
             */

            ServoClaw.CheckClaw(currentGamepad2.y, previousGamepad2.y, ServoArm);

            /**
            X button turns Intake ON
            Intake shuts OFF upon release of X button
            */

            ServoIntake.CheckIntake(currentGamepad2.x, previousGamepad2.x);

            /**
            Left/Right/Up D-pad moves arm, raising lift to safe level first, if needed
            Down D-pad moves arm to center intake position and lowers lift after short delay

            Isaac's Operator Bumper Request
              left bumper:
                    // Opens Claw
                    // Centers Arm
                    // Lowers Lift
                    // Sets claw to "easy" intake position
                    // Turns Intake On

             right bumper:
                    // Turns Intake Off
                    // Closes Claw
                    // Raises lift to safe height for rotation
                    // Rotates Arm to Front (if B move arm based on start position)
                    // Raises lift to High Pole height
            */

            ServoArm.CheckArm(              currentGamepad2.dpad_left, previousGamepad2.dpad_left,
                                            currentGamepad2.dpad_down, previousGamepad2.dpad_down,
                                            currentGamepad2.dpad_right, previousGamepad2.dpad_right,
                                            currentGamepad2.dpad_up, previousGamepad2.dpad_up,
                                            currentGamepad2.left_bumper, previousGamepad2.left_bumper,
                                            currentGamepad2.right_bumper, previousGamepad2.right_bumper,
                                            currentGamepad2.b, currentGamepad2.a);

            /**
            Left Trigger, lowers lift by one Junction Height Level (Intake, Ground, Low, Medium, High)
            Right Trigger, raises lift by one Junction height level (Intake, Ground, Low, Medium, High)
            Left Trigger w/ Modifier(B) pressed, lowers lift to next  Cone Stack Intake Height Level (1, 2, 3, 4, 5 cone stack)
            Right Trigger w/ Modifier(B) pressed, raises lift to next Cone Stack Intake Height Level (1, 2, 3, 4, 5 cones stack)
            Left Stick up/down - raise/lower lift, stop when stick is zeroed
            */

            Lift.CheckLift(                 currentGamepad2.left_trigger, previousGamepad2.left_trigger,
                                            currentGamepad2.right_trigger, previousGamepad2.right_trigger,
                                            currentGamepad2.b,
                                            currentGamepad2.left_stick_y,
                                            ServoArm);

            /** Unused Operator Gamepad Elements:
                A Button
                B Button (but this is used as a modifier for the lift controls) - be careful about using buttons for two things
                Start Button (be careful using this, especially for automated functions because
                            you have to press it to select the controller which can be annoying)
                Back Button

                Right Stick (up/down)
                Right Stick (left/right)
                Left Stick (left/right)

                Left Stick Button
                Right Stick Button
                touchpad controls
            */

            //------------------------------------------------------//
            //--------------------DRIVER CONTROLS-------------------//
            //------------------------------------------------------//

            //Driver manual controls - if any of these are non-zero, all automatic tasks are halted
            MecDrive.CheckManualDriveControls(  currentGamepad1.left_stick_y, previousGamepad1.left_stick_y,
                                                currentGamepad1.left_stick_x, previousGamepad1.left_stick_x,
                                                currentGamepad1.right_stick_x,
                                                currentGamepad1.left_trigger, currentGamepad1.right_trigger);

            //Driver D-PAD controls
            MecDrive.CheckDpadDriveControls(    currentGamepad1.dpad_up, currentGamepad1.dpad_right, currentGamepad1.dpad_down, currentGamepad1.dpad_left,
                                                previousGamepad1.dpad_up, previousGamepad1.dpad_right, previousGamepad1.dpad_down, previousGamepad1.dpad_left,
                                                currentGamepad1.b);

            //Driver bumper controls for rotating
            MecDrive.CheckSquareTurning(currentGamepad1.left_bumper, previousGamepad1.left_bumper,
                    currentGamepad1.right_bumper, previousGamepad1.right_bumper,
                    Gyro);

            //Driver control to move set distance away from alliance substation
            MecDrive.CheckAutoAwayFromAllianceSubstation(currentGamepad1.x, previousGamepad1.x);

            //Driver control to use vision to center on pipe by strafing
            //MecDrive.CheckVisionStrafing(currentGamepad1.y, previousGamepad1.y);

            //Driver control to automatically pickup and deliver a cone
            //MecDrive.CheckAutoDeliver(  currentGamepad1.back, previousGamepad1.back,
            //                            currentGamepad1.right_stick_button, previousGamepad1.right_stick_button);

            //Automated tasks (driving, turning, strafing, vision strafing, auto deliver)
            MecDrive.ContinueAutomaticTasks(Gyro, ServoArm, Lift, ServoClaw, ServoIntake);

            MecDrive.CheckNoManualDriveControls(currentGamepad1.left_stick_y, currentGamepad1.left_stick_x, currentGamepad1.right_stick_x,
                    currentGamepad1.left_trigger, currentGamepad1.right_trigger);

            /** Unused Drive Gamepad Elements:
             A Button
             Back Button
             Right Stick (up/down)

             Left Stick Button
             Right Stick Button
             touchpad controls
             */


            //------------------------------------------------------//
            //------------------TELEMETRY---------------------------//
            //------------------------------------------------------//

            telemetry.addData("Run Time:","%s", runtime.seconds());

            telemetry.addData("Lift", "Position(%s), Target(%s)", Lift.liftMotor.getCurrentPosition(), Lift.getLiftTarget());
            telemetry.addData("Arm Position", ServoArm.currentArmState);
            telemetry.addData("Arm Position", MecDrive.currentAutoArmState);
            telemetry.addData("Claw Position", ServoClaw.currentClawState);

            telemetry.addData("PID Degree Error", "%.3f", MecDrive.pid.degree_error);
            telemetry.addData("PID Percent Error", "%.3f", MecDrive.pid.percent_error);
            telemetry.addData("PID Motor Power Output", "%.3f", MecDrive.pid.output);
            telemetry.addData("Kf", "%.3f", MecDrive.pid.m_kF);
            //telemetry.addData("Lift Limit Switch State", Lift.LimitSwitchIsPressed());
            telemetry.addData("Color", "R %d  G %d  B %d", MecDrive.colorSensor.red(), MecDrive.colorSensor.green(), MecDrive.colorSensor.blue());

            telemetry.addData("Encoders", "LF(%s), RF(%s)", MecDrive.LFDrive.getCurrentPosition(), MecDrive.RFDrive.getCurrentPosition());
            telemetry.addData("Encoders", "LB(%s), RB(%s)", MecDrive.LBDrive.getCurrentPosition(), MecDrive.RBDrive.getCurrentPosition());

            //telemetry.addData("Automatic Deliver State", "(%s)", MecDrive.currentAutomaticTask);

            telemetry.update();
        }
        MecDrive.drive = 0;
        MecDrive.strafe = 0;
        MecDrive.turn = 0;
        MecDrive.MecanumDrive();
    }
}






