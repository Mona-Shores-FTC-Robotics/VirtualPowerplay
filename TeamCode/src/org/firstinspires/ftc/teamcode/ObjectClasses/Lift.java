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

    //motor parameters
    final double TICKS_PER_REV = 1200;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_MM = 1000;
    double COUNTS_PER_MM = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);

    final double LIFT_POWER = .8;

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
        liftMotor.setPower(LIFT_POWER);
        while (activeOpMode.opModeIsActive() && liftMotor.isBusy()) {
           //keep moving
            activeOpMode.telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder Target", newLiftTarget);
            activeOpMode.telemetry.addData("Status", "Run Time: " + activeOpMode.getRuntime());
            activeOpMode.telemetry.update();
        }
        liftMotor.setPower(0);
    }

}
