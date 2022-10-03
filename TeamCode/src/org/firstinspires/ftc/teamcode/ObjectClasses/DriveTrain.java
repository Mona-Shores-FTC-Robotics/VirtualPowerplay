/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 */

public class DriveTrain
{
    /* Public OpMode members. */
    public DcMotor LFDrive = null;
    public DcMotor RFDrive = null;
    public DcMotor LBDrive = null;
    public DcMotor RBDrive = null;
    public double leftFrontPower = 0;
    public double rightFrontPower = 0;
    public double leftBackPower = 0;
    public double rightBackPower = 0;
    public double drive = 0;
    public double strafe = 0;
    public double turn = 0;
    public Orientation lastAngles = new Orientation();
    public double currAngle = 0.0;
    public double multiplier = 1;
    public double MINMULT = .5;
    public double MAXMULT = 1;

    public double gyroOffset = 0;

    //motor and wheel parameters
    final double TICKS_PER_REV = 537.7;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 3.93701;
    double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double LOW_SPEED = .4;
    public static final double MED_SPEED = .6;
    public static final double HIGH_SPEED = 1;

    BNO055IMU imu;
    public ColorSensor colorSensor;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public DriveTrain(){
    }

    /* Initialize Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LFDrive  = ahwMap.get(DcMotor.class, "front_left_motor");
        RFDrive = ahwMap.get(DcMotor.class, "front_right_motor");
        LBDrive  = ahwMap.get(DcMotor.class, "back_left_motor");
        RBDrive = ahwMap.get(DcMotor.class, "back_right_motor");

        LFDrive.setDirection(DcMotor.Direction.FORWARD);
        RFDrive.setDirection(DcMotor.Direction.REVERSE);
        LBDrive.setDirection(DcMotor.Direction.FORWARD);
        RBDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        LFDrive.setPower(0);
        RFDrive.setPower(0);
        LBDrive.setPower(0);
        RBDrive.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

       colorSensor = hwMap.colorSensor.get("color_sensor");

    }

    //Set power to all motors
    public void setAllPower(double p){setMotorPower(p,p,p,p);}

    public void setMotorPower(double lF,double rF,double lB,double rB){
        LFDrive.setPower(lF*multiplier);
        RFDrive.setPower(rF*multiplier);
        LBDrive.setPower(lB*multiplier);
        RBDrive.setPower(rB*multiplier);
    }
    public void MecanumDrive() {

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Put Mecanum Drive math and motor commands here.
        double dPercent = Math.abs(drive) / (Math.abs(drive) + Math.abs(strafe) + Math.abs(turn));
        double sPercent = Math.abs(strafe) / (Math.abs(drive) + Math.abs(turn) + Math.abs(strafe));
        double tPercent = Math.abs(turn) / (Math.abs(drive) + Math.abs(turn) + Math.abs(strafe));

        rightFrontPower = (drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent);
        rightBackPower = (drive * dPercent) + (strafe * sPercent) + (-turn * tPercent);
        leftFrontPower = (drive * dPercent) + (strafe * sPercent) + (turn * tPercent);
        leftBackPower = (drive * dPercent) + (-strafe * sPercent) + (turn * tPercent);

        if (!Double.isNaN(leftFrontPower) && !Double.isNaN(rightFrontPower) && !Double.isNaN(leftBackPower) && !Double.isNaN(rightBackPower)) {
            LFDrive.setPower(leftFrontPower);
            RFDrive.setPower(rightFrontPower);
            LBDrive.setPower(leftBackPower);
            RBDrive.setPower(rightBackPower);
        }
    }


    public void encoderDrive(double speed, int leftInches, int rightInches, LinearOpMode activeOpMode) {

        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
        int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

        LFDrive.setTargetPosition(newLeftFrontTarget);
        RFDrive.setTargetPosition(newRightFrontTarget);
        LBDrive.setTargetPosition(newLeftBackTarget);
        RBDrive.setTargetPosition(newRightBackTarget);

        LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        period.reset();
        RFDrive.setPower(abs(speed));
        LFDrive.setPower(abs(speed));
        LBDrive.setPower(abs(speed));
        RBDrive.setPower(abs(speed));



        while (activeOpMode.opModeIsActive() &&
                (period.seconds() < 5) &&
                (RFDrive.isBusy() && LFDrive.isBusy() && LBDrive.isBusy() && RBDrive.isBusy())) {
            /**
            activeOpMode.telemetry.addData("Encoder BL", LFDrive.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder FR", RFDrive.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder BL", LBDrive.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder BR", RBDrive.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder Target", newLeftFrontTarget);
            activeOpMode.telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            activeOpMode.telemetry.addData("Status", "Run Time: " + activeOpMode.getRuntime());
            activeOpMode.telemetry.update();
            **/
        }

        setAllPower(0);
        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void strafeDrive(double speed, int leftInches, int rightInches, LinearOpMode activeOpMode) {

        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
        int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

        LFDrive.setTargetPosition(newLeftFrontTarget);
        RFDrive.setTargetPosition(-newRightFrontTarget);
        LBDrive.setTargetPosition(-newLeftBackTarget);
        RBDrive.setTargetPosition(newRightBackTarget);

        LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        period.reset();

        RFDrive.setPower(abs(speed));
        LFDrive.setPower(abs(speed));
        LBDrive.setPower(abs(speed));
        RBDrive.setPower(abs(speed));

        while (activeOpMode.opModeIsActive() &&
                (period.seconds() < 5) &&
                (RFDrive.isBusy() && LFDrive.isBusy() && LBDrive.isBusy() && RBDrive.isBusy())) {
            /**
            activeOpMode.telemetry.addData("Encoder BL", LFDrive.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder FR", RFDrive.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder BL", LBDrive.getCurrentPosition());
            activeOpMode.telemetry.addData("Encoder BR", RBDrive.getCurrentPosition());
            activeOpMode.telemetry.update();
            **/
        }

        setAllPower(0);

        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        if (deltaAngle > 180)
        {
            deltaAngle -= 360;
        }
        else if (deltaAngle <= -180)
        {
            deltaAngle+=360;
        }
        currAngle += deltaAngle;
        lastAngles = orientation;
        return currAngle;
    }

    public void turn (double degrees, LinearOpMode activeOpMode)
    {
        resetAngle();
        double error = degrees;
        while (activeOpMode.opModeIsActive() && Math.abs(error) > 1)
        {
            double motorPower = (error < 0 ? .6 : -.6);
            setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();

        }
        setAllPower(0);
        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnTo(double degrees, LinearOpMode activeOpMode)
    {

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle + gyroOffset;

        if (error > 180)
        {
            error -= 360;
        }
        else if (error <= -180)
        {
            error+=360;
        }
        turn(error, activeOpMode);

    }


    public void colorDrive(double speed, int allianceColor, LinearOpMode activeOpMode)
    {

        if (allianceColor == 1)
        {
            while (activeOpMode.opModeIsActive() && colorSensor.blue() < 230 && colorSensor.red() > 50)
            {
                setAllPower(speed);
                activeOpMode.telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
                activeOpMode.telemetry.update();
            }

        }
        else if (allianceColor == -1)
        {
            while (activeOpMode.opModeIsActive() && colorSensor.red() < 230 && colorSensor.blue() > 50)
                {
                    setAllPower(speed);
                    activeOpMode.telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
                    activeOpMode.telemetry.update();
                }

        }

        setAllPower(0);
        activeOpMode.telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        activeOpMode.telemetry.update();

    }
    public void colorStrafe(double speed, int allianceColor, LinearOpMode activeOpMode)
    {

        if (allianceColor == 1)
        {
            while (activeOpMode.opModeIsActive() && colorSensor.blue() < 230 && colorSensor.red() > 50)
            {
                setMotorPower(-speed, speed, speed, -speed);
                activeOpMode.telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
                activeOpMode.telemetry.update();
            }

        }
        else if (allianceColor == -1)
        {
            while (activeOpMode.opModeIsActive() && colorSensor.red() < 230 && colorSensor.blue() > 50)
            {
                setMotorPower(speed, -speed, -speed, speed);
                activeOpMode.telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
                activeOpMode.telemetry.update();
            }

        }

        setAllPower(0);
        activeOpMode.telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        activeOpMode.telemetry.update();

    }

    public void calibrateGyro(LinearOpMode activeOpMode)
    {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroOffset = orientation.firstAngle;

        activeOpMode.telemetry.addData("Gyro Offset", gyroOffset);
        activeOpMode.telemetry.update();

    }

    public void encoderDrive(double speed, int leftInches, int rightInches) {

        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
        int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

        LFDrive.setTargetPosition(newLeftFrontTarget);
        RFDrive.setTargetPosition(newRightFrontTarget);
        LBDrive.setTargetPosition(newLeftBackTarget);
        RBDrive.setTargetPosition(newRightBackTarget);

        LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        period.reset();
        RFDrive.setPower(abs(speed));
        LFDrive.setPower(abs(speed));
        LBDrive.setPower(abs(speed));
        RBDrive.setPower(abs(speed));

        while ( (period.seconds() < 5) &&
                (RFDrive.isBusy() && LFDrive.isBusy() && LBDrive.isBusy() && RBDrive.isBusy())) {
        }

        setAllPower(0);
        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void strafeDrive(double speed, int leftInches, int rightInches) {

        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
        int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

        LFDrive.setTargetPosition(newLeftFrontTarget);
        RFDrive.setTargetPosition(-newRightFrontTarget);
        LBDrive.setTargetPosition(-newLeftBackTarget);
        RBDrive.setTargetPosition(newRightBackTarget);

        LFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        period.reset();

        RFDrive.setPower(abs(speed));
        LFDrive.setPower(abs(speed));
        LBDrive.setPower(abs(speed));
        RBDrive.setPower(abs(speed));

        while ( (period.seconds() < 5) &&
                (RFDrive.isBusy() && LFDrive.isBusy() && LBDrive.isBusy() && RBDrive.isBusy())) {
        }

        setAllPower(0);

        LFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}


