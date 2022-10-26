package org.firstinspires.ftc.teamcode.OpModes;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepad1Controls;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepad2Controls;
import org.firstinspires.ftc.teamcode.ObjectClasses.Intake;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;

@TeleOp(name = "Teleop Mode w/ Turret Bot", group = "Turret Bot")
public class TeleOp_Linear_Turret_Bot extends LinearOpMode {

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    Arm ServoArm = new Arm();
    Intake ServoIntake = new Intake();
    Claw ServoClaw = new Claw();
    Gamepad1Controls G1PadControls = new Gamepad1Controls(this);
    Gamepad2Controls G2PadControls = new Gamepad2Controls(this);
    org.firstinspires.ftc.teamcode.ObjectClasses.Lift Lift = new Lift();

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
        Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM,this);


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        MecDrive.calibrateGyro(this);

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
        MecDrive.currentRobotState = DriveTrain.robotState.HUMAN_CONTROLLED;

        while (opModeIsActive()) {
            //Store the gamepad values to be used for this iteration of the loop.
            currentGamepad1 = gamepad1;
            currentGamepad2 = gamepad2;

            if (opModeIsActive() && MecDrive.currentRobotState == DriveTrain.robotState.HUMAN_CONTROLLED)
            {
                //Driver Controls for Gamepad1
                G1PadControls.CheckControls(currentGamepad1, MecDrive);

                //Operator Controls for Gamepad2
                G2PadControls.CheckControls(currentGamepad2, Lift, ServoArm, ServoClaw, ServoIntake);

            } else if (opModeIsActive() && MecDrive.currentRobotState == DriveTrain.robotState.AUTOMATIC_TASK)
            {
                if (MecDrive.alreadyDriving == true) {
                    MecDrive.ContinueDriving(this);
                }
                else if (MecDrive.alreadyStrafing == true) {
                    MecDrive.ContinueStrafing(this);
                }
                if (Lift.alreadyLifting)
                {
                    Lift.keepLifting(this);
                }
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Motors", "leftfront(%.2f), rightfront (%.2f)", MecDrive.leftFrontPower*MecDrive.multiplier, MecDrive.rightFrontPower*MecDrive.multiplier);
            telemetry.addData("Motors", "leftback (%.2f), rightback (%.2f)", MecDrive.leftBackPower*MecDrive.multiplier, MecDrive.rightBackPower*MecDrive.multiplier);
            telemetry.addData("Lift Position", Lift.liftMotor.getCurrentPosition());
            telemetry.addData("Arm Position", ServoArm.arm.getPosition());
            telemetry.addData("Claw Position", ServoClaw.claw.getPosition());
            telemetry.addData("Gyro Angle", (int) MecDrive.getAbsoluteAngle(this));
            telemetry.addLine("");
            telemetry.addData("# of Cones Delivered", teleopConeDeliveryTracker);
            telemetry.update();
        }
        MecDrive.drive = 0;
        MecDrive.strafe = 0;
        MecDrive.turn = 0;
        MecDrive.MecanumDrive();
    }
}






