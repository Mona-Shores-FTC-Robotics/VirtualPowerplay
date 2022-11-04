package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_MM;
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
    public int newLiftTarget;
    //motor parameters
    final double TICKS_PER_REV = 537.7;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_MM = 250;
    double COUNTS_PER_MM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);

    final double STEP_LIFT_POWER = .8;

    double LIFT_TARGET_MULTIPLIER = 10;
    double liftPowerMultiplier = 1.0;
    double LIFT_POWER_MULTIPLIER_MAX = 1.0;
    double LIFT_POWER_MULTIPLIER_MIN = .4;

    LinearOpMode activeOpMode;

    public Lift(LinearOpMode mode){
        activeOpMode = mode;
    }

    public void init(HardwareMap ahwMap) {
        // Define and Initialize Motor
        liftMotor  = ahwMap.get(DcMotor.class, "lift_motor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(0);
    }

    public void StartLifting(double targetHeightInMM) {
        if (activeOpMode.opModeIsActive() && alreadyLifting == false) {
            //begin lifting
            newLiftTarget = (int) (targetHeightInMM * COUNTS_PER_MM);
            liftMotor.setTargetPosition(newLiftTarget);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(STEP_LIFT_POWER*liftPowerMultiplier);
            alreadyLifting = true;
        }
    }

    public void ContinueLifting() {
        if (liftMotor.isBusy() == true) {
            //keep lifting because liftMotor is still trying to reach the target
        } else if (liftMotor.isBusy() == false) {
            //lift has reached target
            alreadyLifting = false;
        }
    }

    public void ManualLift(double liftTarget) {
        alreadyLifting = false;
        newLiftTarget = (int) ((liftTarget*LIFT_TARGET_MULTIPLIER) + newLiftTarget);
        if (liftTarget >0 && newLiftTarget > 700) {newLiftTarget =700;}
        if (liftTarget <0 && newLiftTarget < 50) {newLiftTarget =50;}
        liftMotor.setTargetPosition(newLiftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(STEP_LIFT_POWER*liftPowerMultiplier);
    }

    public void CheckLift(Boolean liftStageDownCurrentButton, Boolean liftStageDownPreivousButton,
                          Boolean liftStageUpCurrentButton, Boolean liftStageUpPreviousButton,
                          double manualLiftTargetChange) {
        if (manualLiftTargetChange != 0) {
            ManualLift(-manualLiftTargetChange);
        } else if (liftStageDownCurrentButton && !liftStageDownPreivousButton) {
            StartLifting(ONE_CONE_INTAKE_HEIGHT_MM);
        } else if (liftStageUpCurrentButton && !liftStageUpPreviousButton) {
            StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM);
        } else if (alreadyLifting) {
            ContinueLifting();
        }
    }
}
