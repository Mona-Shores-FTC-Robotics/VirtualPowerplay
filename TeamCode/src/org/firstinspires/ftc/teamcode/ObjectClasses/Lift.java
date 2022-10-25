package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public DcMotor liftMotor = null;
    public boolean alreadyLifting = false;
    private int newLiftTarget;
    //motor parameters
    final double TICKS_PER_REV = 537.7;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_MM = 130;
    double COUNTS_PER_MM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);

    final double STEP_LIFT_POWER = .8;

    double liftPowerMultiplier = 1.0;
    double LIFT_POWER_MULTIPLIER_MAX = 1.0;
    double LIFT_POWER_MULTIPLIER_MIN = .4;

    public Lift(){
    }
    public void init(HardwareMap ahwMap) {
        // Define and Initialize Motor
        liftMotor  = ahwMap.get(DcMotor.class, "lift_motor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(0);
    }

    public void moveLift(double targetHeightInMM, LinearOpMode activeOpMode) {
        int newLiftTarget = (int) (targetHeightInMM * COUNTS_PER_MM);
        liftMotor.setTargetPosition(newLiftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(STEP_LIFT_POWER);
        while (activeOpMode.opModeIsActive() && liftMotor.isBusy()) {
           //keep moving
            activeOpMode.telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder Target", newLiftTarget);
            activeOpMode.telemetry.addData("Status", "Run Time: " + activeOpMode.getRuntime());
            activeOpMode.telemetry.update();
        }
        liftMotor.setPower(0);
    }

    public void startLifting(double targetHeightInMM, LinearOpMode activeOpMode) {
        if (activeOpMode.opModeIsActive() && alreadyLifting == false) {
            //begin lifting
            newLiftTarget = (int) (targetHeightInMM * COUNTS_PER_MM);
            liftMotor.setTargetPosition(newLiftTarget);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(STEP_LIFT_POWER);
            alreadyLifting = true;
        }
    }
    public void keepLifting(LinearOpMode activeOpMode) {
        if (activeOpMode.opModeIsActive() && alreadyLifting == true && liftMotor.isBusy() == true) {
            //keep lifting because liftMotor is still trying to reach the target
            activeOpMode.telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder Target", newLiftTarget);
            activeOpMode.telemetry.addData("Status", "Run Time: " + activeOpMode.getRuntime());
        }
        else if (activeOpMode.opModeIsActive() && alreadyLifting == true && liftMotor.isBusy() == false) {
            //lift has reached target
            alreadyLifting = false;
            liftMotor.setPower(0);
        }
    }


    public void ManualLift(double power) {
        power = power * .7;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if ((power > 0 && liftMotor.getCurrentPosition() < 1500) || (power < 0 && liftMotor.getCurrentPosition() > 250)) {
            liftMotor.setPower(power*liftPowerMultiplier);
        }
        else
        {
            liftMotor.setPower(0);
        }
    }

}
