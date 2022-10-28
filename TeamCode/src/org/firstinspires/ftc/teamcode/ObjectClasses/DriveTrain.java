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
import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {

    //DriveTrain Constants
    public static final double STARTING_DRIVE_MULTIPLIER = .7;
    public double MINMULT = .5;
    public double MAXMULT = 1;

    //motor and wheel parameters
    final double TICKS_PER_REV = 537.7;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 3.93701;
    double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //Speed Constants
    public static final double LOW_SPEED = .4;
    public static final double MED_SPEED = .6;
    public static final double HIGH_SPEED = 1;
    public static final double STARTING_RAMP_VALUE = .1;

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

    public double topSpeed = 0;
    public double multiplier = STARTING_DRIVE_MULTIPLIER;
    public double ramp = STARTING_RAMP_VALUE;

    //state machine members
    public boolean alreadyDriving = false;
    public boolean alreadyStrafing = false;

    public LinearOpMode activeOpMode;
    HardwareMap hwMap = null;

    //Turn Related Variables
    public double degreesLeftToTurn;
    public double targetAngleInDegrees;
    public TurnPIDController pid = new TurnPIDController(0, 0, 0, 0);
    public boolean alreadyTurning = false;
    public boolean alreadyPIDTurning = false;
    public double targetAngle;

    private ElapsedTime drivePeriod = new ElapsedTime();
    private ElapsedTime strafePeriod = new ElapsedTime();
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public DriveTrain(LinearOpMode mode) {
        activeOpMode = mode;
    }

    /* Initialize Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LFDrive = ahwMap.get(DcMotor.class, "front_left_motor");
        RFDrive = ahwMap.get(DcMotor.class, "front_right_motor");
        LBDrive = ahwMap.get(DcMotor.class, "back_left_motor");
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


    }

    //Set power to all motors
    public void setAllPower(double p) {
        setMotorPower(p, p, p, p);
    }

    public void setMotorPower(double lF, double rF, double lB, double rB) {
        LFDrive.setPower(lF * multiplier);
        RFDrive.setPower(rF * multiplier);
        LBDrive.setPower(lB * multiplier);
        RBDrive.setPower(rB * multiplier);
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

        rightFrontPower = (drive * dPercent) + (-strafe * sPercent) + (turn * tPercent);
        rightBackPower = (drive * dPercent) + (strafe * sPercent) + (turn * tPercent);
        leftFrontPower = (drive * dPercent) + (strafe * sPercent) + (-turn * tPercent);
        leftBackPower = (drive * dPercent) + (-strafe * sPercent) + (-turn * tPercent);

        if (!Double.isNaN(leftFrontPower) && !Double.isNaN(rightFrontPower) && !Double.isNaN(leftBackPower) && !Double.isNaN(rightBackPower)) {
            LFDrive.setPower(leftFrontPower);
            RFDrive.setPower(rightFrontPower);
            LBDrive.setPower(leftBackPower);
            RBDrive.setPower(rightBackPower);
        } else {
            LFDrive.setPower(0);
            RFDrive.setPower(0);
            LBDrive.setPower(0);
            RBDrive.setPower(0);
        }

    }

    public void startEncoderDrive(double speed, int leftInches, int rightInches) {
        topSpeed = speed;
        if (activeOpMode.opModeIsActive() && alreadyDriving == false) {

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

            //track how much time we have been driving
            drivePeriod.reset();

            //reset starting ramp value
            ramp = STARTING_RAMP_VALUE;

            RFDrive.setPower(abs(ramp));
            LFDrive.setPower(abs(ramp));
            LBDrive.setPower(abs(ramp));
            RBDrive.setPower(abs(ramp));

            //we are now driving
            alreadyDriving = true;
        }
    }

    public void ContinueDriving() {
        if (activeOpMode.opModeIsActive() &&
                alreadyDriving == true &&
                (RFDrive.isBusy() && LFDrive.isBusy() && LBDrive.isBusy() && RBDrive.isBusy())) {
            RFDrive.setPower(abs(ramp));
            LFDrive.setPower(abs(ramp));
            LBDrive.setPower(abs(ramp));
            RBDrive.setPower(abs(ramp));
            if (ramp < topSpeed) {
                ramp = ramp + .003;
            } else if (ramp > topSpeed) {
                ramp = ramp - .003;
            }
        } else {
            alreadyDriving = false;
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

    public void startStrafeDrive(double speed, int leftInches, int rightInches) {
        topSpeed = speed;
        if (activeOpMode.opModeIsActive() && alreadyStrafing == false) {
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

            strafePeriod.reset();

            ramp = .1;

            RFDrive.setPower(abs(ramp));
            LFDrive.setPower(abs(ramp));
            LBDrive.setPower(abs(ramp));
            RBDrive.setPower(abs(ramp));

            alreadyStrafing = true;
        }
    }

    public void ContinueStrafing() {
        if (activeOpMode.opModeIsActive() &&
                alreadyStrafing == true &&
                (RFDrive.isBusy() && LFDrive.isBusy() && LBDrive.isBusy() && RBDrive.isBusy())) {
            RFDrive.setPower(abs(ramp));
            LFDrive.setPower(abs(ramp));
            LBDrive.setPower(abs(ramp));
            RBDrive.setPower(abs(ramp));
            if (ramp < topSpeed) {
                ramp = ramp + .003;
            } else if (ramp > topSpeed) {
                ramp = ramp - .003;
            }
        } else {
            alreadyStrafing = false;
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


    public void CheckDriveControls(Gamepad currentGamepad1, Gamepad previousGamepad1, Lift Lift, Arm ServoArm, Claw ServoClaw, Intake ServoIntake, Gyro Gyro) {

        //Driver controls have first priority
        if (currentGamepad1.left_stick_y != 0 || currentGamepad1.left_stick_x !=0 || currentGamepad1.right_stick_x !=0 ||
            currentGamepad1.left_trigger >0 || currentGamepad1.right_trigger >0)
        {
            alreadyStrafing = false;
            alreadyDriving = false;
            alreadyTurning = false;
            alreadyPIDTurning = false;

            drive = -currentGamepad1.left_stick_y; //-1.0 to 1.0
            strafe = currentGamepad1.left_stick_x; //-1.0 to 1.0
            turn = currentGamepad1.right_stick_x; //-1.0 to 1.0

            //Use fine tune values from triggers instead of right stick if Driver is pressing one of them
            if (currentGamepad1.left_trigger > .1) {
                turn = -currentGamepad1.left_trigger*.3; //-1.0 to 1.0
            } else if (currentGamepad1.right_trigger > .1) {
                turn = currentGamepad1.right_trigger*.3;
            }
            MecanumDrive();
        }
        //-------DPAD Tile moves ---------
        else if (currentGamepad1.dpad_right  &&  !previousGamepad1.dpad_right) {
            if (currentGamepad1.b == true) {
                startStrafeDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE);
            } else {
                startStrafeDrive(HIGH_SPEED, HALF_TILE_DISTANCE, HALF_TILE_DISTANCE);
            }
        } else if (currentGamepad1.dpad_down   &&  !previousGamepad1.dpad_down)   {
            if (currentGamepad1.b == true) {
                startEncoderDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE);
            } else {
                startEncoderDrive(HIGH_SPEED, -HALF_TILE_DISTANCE, -HALF_TILE_DISTANCE);
            }
        } else if (currentGamepad1.dpad_left   &&  !previousGamepad1.dpad_left) {
            if (currentGamepad1.b == true) {
                startStrafeDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE);
            } else {
                startStrafeDrive(HIGH_SPEED, -HALF_TILE_DISTANCE, -HALF_TILE_DISTANCE);
            }
        } else if (currentGamepad1.dpad_up     &&  !previousGamepad1.dpad_up) {
            if (currentGamepad1.b == true) {
                startEncoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE);
            } else {
                startEncoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE, HALF_TILE_DISTANCE);
            }
        } else if (currentGamepad1.a && !previousGamepad1.a) {
            //move from alliance substation to scoring position
            startEncoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE);
        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            //ROTATE TO THE LEFT TO THE CLOSEST RIGHT ANGLE 0, 90, 180, 270
            RotateClosestRightAngleToLeft(Gyro);
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            //ROTATE TO THE LEFT TO THE CLOSEST RIGHT ANGLE 0, 90, 180, 270
            RotateClosestRightAngleToRight(Gyro);
        } else if (currentGamepad1.y && !previousGamepad1.y) {
            auto_deliver(W_3_JUNCTION, ServoArm, Lift, ServoClaw);
        } else if (alreadyDriving) {
            ContinueDriving();
        } else if (alreadyStrafing) {
            ContinueStrafing();
        } else if (alreadyPIDTurning) {
            ContinuePIDTurning(Gyro);
        } else if (alreadyTurning) {
            ContinueTurning(Gyro);
        } else if (!alreadyPIDTurning && !alreadyTurning && !alreadyDriving && !alreadyStrafing) {
        drive = -currentGamepad1.left_stick_y; //-1.0 to 1.0
        strafe = currentGamepad1.left_stick_x; //-1.0 to 1.0
        turn = currentGamepad1.right_stick_x; //-1.0 to 1.0
        MecanumDrive();
        }
    }
    public void auto_deliver(int deliveryDestination, Arm ServoArm, Lift Lift, Claw ServoClaw) {
    }


    private void RotateClosestRightAngleToLeft(Gyro Gyro) {
        double currentAngle = Gyro.getAbsoluteAngle();
        if (currentAngle >= 0 && currentAngle < 85 || currentAngle <=0 && currentAngle > -5) {
            turnToPID(90, Gyro);
        }

        if (currentAngle <=-5 && currentAngle > -95){
            turnToPID(0, Gyro);
        }

        if (currentAngle <=-95 && currentAngle >= -180 || currentAngle >= 175 && currentAngle <= 180){
            turnToPID(-90, Gyro);
        }

        if (currentAngle >= 85 && currentAngle < 175){
            turnToPID(-180, Gyro);
        }
    }
    private void RotateClosestRightAngleToRight(Gyro Gyro) {
        double currentAngle = Gyro.getAbsoluteAngle();
        if (currentAngle <= 0 && currentAngle > -85 || currentAngle >=0 && currentAngle < 5) {
            turnToPID(-90, Gyro);
        }

        if (currentAngle <=-85 && currentAngle > -175){
            turnToPID(180, Gyro);
        }

        if (currentAngle <=-175 && currentAngle >= -180 || currentAngle > 95 && currentAngle <= 180){
            turnToPID(90, Gyro);
        }

        if (currentAngle <= 95 && currentAngle > 0){
            turnToPID(0, Gyro);
        }
    }

    public void turn(double degrees, Gyro Gyro) {
        alreadyTurning = true;
        Gyro.resetAngle();
        targetAngleInDegrees = degrees;
        degreesLeftToTurn = degrees;
    }

    public void ContinueTurning(Gyro Gyro) {
        if (abs(degreesLeftToTurn) > 2) {
            double motorPower = (degreesLeftToTurn < 0 ? .8 : -.8);
            setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            degreesLeftToTurn = targetAngleInDegrees - Gyro.getAngle();
        } else {
            alreadyTurning = false;
            setAllPower(0);
        }
    }

    public void turnTo(double degrees, Gyro Gyro) {
        double absoluteAngle = Gyro.getAbsoluteAngle();
        targetAngle = degrees - absoluteAngle;

        if (targetAngle > 180) {
            targetAngle -= 360;
        } else if (targetAngle < -180)
        {
            targetAngle +=360;
        }
        turn(targetAngle, Gyro);
    }

    void turnToPID (double degrees, Gyro Gyro) {
        double absoluteAngle = Gyro.getAbsoluteAngle();
        targetAngle = degrees - absoluteAngle;
        if (targetAngle > 180) {
            targetAngle -= 360;
        } else if (targetAngle < -180)
        {
            targetAngle +=360;
        }
        turnPID(targetAngle, Gyro);

    }

    void turnPID(double degrees, Gyro Gyro){
        alreadyPIDTurning = true;
        Gyro.resetAngle();
        pid = new TurnPIDController(degrees, .7, .1, 0);
    }

    public void ContinuePIDTurning(Gyro Gyro) {
        if (Math.abs(pid.pidAngleLeftToTurn) > 1) {
            double motorPower = pid.update(Gyro.getAngle());
            setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
        } else {
            alreadyPIDTurning = false;
            setAllPower(0);
        }
    }
}


