package com.acmerobotics.dashboard.config.disabled_samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Disabled
@Autonomous(name = "Test", group = "Test")
public class Test extends LinearOpMode {

    DcMotor BL, FL, FR, BR;
    BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    final double TICKS_PER_REV = 400.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 3.93701;
    double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    int allianceColor = -1; //1 = Blue // -1 = Red
    int startLocation = -1; //1 = close to audience A2/F2 // -1 =opposite side as audience A5/F5
    int allianceLocationFactor = allianceColor*startLocation;
    int autoNumber = 2; //1 = Just Park; 2 = Deliver One and Park; 3= Deliver Six and Park
    int halfTileDistance = 33;
    int fullTileDistance = 65;
    int Signal;

    @Override

    public void runOpMode() {
        FL = hardwareMap.dcMotor.get("front_left_motor");
        FR = hardwareMap.dcMotor.get("front_right_motor");
        BL = hardwareMap.dcMotor.get("back_left_motor");
        BR = hardwareMap.dcMotor.get("back_right_motor");

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        waitForStart();

        //Read Signal
        //Use Webcam to find out Signal and store in Signal variable
        Signal = 3;

        //Backup into wall for alignment
        encoderDrive(.8, 20, 20);

        //Just Park Based on Signal
        if (autoNumber == 1) {
            autoJustPark();
        }
        //Deliver Starting Cone to X-2 Junction (High Pole) Then Park
        else if (autoNumber == 2) {
            autoDeliverOneThenPark();
        } else if (autoNumber == 3) {
            autoDeliverSixThenPark();
        }
    }

    public void encoderDrive(double speed, int leftInches, int rightInches) {

        if (opModeIsActive()) {

            int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
            int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

            FL.setTargetPosition(newLeftFrontTarget);
            FR.setTargetPosition(newRightFrontTarget);
            BL.setTargetPosition(newLeftBackTarget);
            BR.setTargetPosition(newRightBackTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < 5) &&
                    (FR.isBusy() || FL.isBusy() || BL.isBusy() || BR.isBusy())) {
                telemetry.addData("Encoder BL", FL.getCurrentPosition());
                telemetry.addData("Encoder FR", FR.getCurrentPosition());
                telemetry.addData("Encoder BL", BR.getCurrentPosition());
                telemetry.addData("Encoder BR", BL.getCurrentPosition());

                telemetry.addData("Encoder Target", newLeftFrontTarget);

                telemetry.update();
            }


            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);

        }

    }

    public void strafeDrive(double speed, int leftInches, int rightInches) {

        if (opModeIsActive()) {

            int newLeftFrontTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightFrontTarget = (int) (rightInches * COUNTS_PER_INCH);
            int newLeftBackTarget = (int) (leftInches * COUNTS_PER_INCH);
            int newRightBackTarget = (int) (rightInches * COUNTS_PER_INCH);

            FL.setTargetPosition(newLeftFrontTarget);
            FR.setTargetPosition(-newRightFrontTarget);
            BL.setTargetPosition(-newLeftBackTarget);
            BR.setTargetPosition(newRightBackTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < 5) &&
                    (FR.isBusy() || FL.isBusy() || BL.isBusy() || BR.isBusy())) {
                telemetry.addData("Encoder BL", FL.getCurrentPosition());
                telemetry.addData("Encoder FR", FR.getCurrentPosition());
                telemetry.addData("Encoder BL", BR.getCurrentPosition());
                telemetry.addData("Encoder BR", BL.getCurrentPosition());

                telemetry.addData("Encoder Target", newLeftFrontTarget);

                telemetry.update();
            }


            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);

            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);

        }
    }


    public void autoJustPark() {
        if (opModeIsActive()) {
            encoderDrive(.8, -78, -78);

            if (Signal == 1) {
                strafeDrive(.6, 66, 66);
            } else if (Signal == 2) {
            } else if (Signal == 3) {
                strafeDrive(.6, -66, -66);
            }
        }

    }

    public void autoDeliverOneThenPark() {

        if (opModeIsActive()) {
            //Drive Forward
            encoderDrive(.8, -80, -140);
            //Strafe in Front of High Pole
            //Can we use vision to find the pole?
            strafeDrive(.6, halfTileDistance * allianceLocationFactor, halfTileDistance * allianceLocationFactor);
            //Place Cone on High Pole
            //PLACEHOLDER CODE FOR PLACING CONE
            sleep(1000);

            //Park after placing cone
            if (Signal == 1) {
                strafeDrive(.6, fullTileDistance - (halfTileDistance * allianceLocationFactor), fullTileDistance - (halfTileDistance * allianceLocationFactor));
            } else if (Signal == 2) {
                strafeDrive(.6, 0 - (halfTileDistance*allianceLocationFactor), 0 - (halfTileDistance* allianceLocationFactor));
            } else if (Signal == 3) {
                strafeDrive(.6, ((-1)*(fullTileDistance + halfTileDistance)*allianceLocationFactor), (-1*(fullTileDistance + halfTileDistance)*allianceLocationFactor));
            }
        }
    }


    public void autoDeliverSixThenPark() {

        if (opModeIsActive()) {
            //Drive Forward
            encoderDrive(.8, -140, -140);
            //Strafe in Front of High Pole
            //Can we use vision to find the pole?
            strafeDrive(.6, halfTileDistance * allianceLocationFactor, halfTileDistance *allianceLocationFactor);
            //Place Cone on High Pole
            //PLACEHOLDER CODE FOR PLACING CONE
            sleep(2000);

            //Drive to Cones by the Wall
            strafeDrive(.6, (-1*halfTileDistance*allianceLocationFactor), (-1*halfTileDistance*allianceLocationFactor));

            // Turn to align with cones [use color sensor later?]
            encoderDrive(.5, -65 * allianceLocationFactor, 65*allianceLocationFactor);
            //drive to Stacked Cone line
            encoderDrive(.5, (-1*allianceLocationFactor * halfTileDistance), (-1*allianceLocationFactor * halfTileDistance));
            //routine to pickup cone, deliver cone, and return to red line
            PickupConeFromStack();



        }
    }

    public void PickupConeFromStack() {
        //approach cone
        encoderDrive(.5, (-1*allianceLocationFactor * halfTileDistance), (-1*allianceLocationFactor * halfTileDistance));
        //drive right up to cone and align on wall?
        encoderDrive(.5, (-1*allianceLocationFactor * halfTileDistance), (-1*allianceLocationFactor * halfTileDistance));
        //PLACEHOLDER CODE FOR PICKING UP CONE
        sleep(2000);

        //backup
        encoderDrive(.5, (allianceLocationFactor * halfTileDistance), (allianceLocationFactor * halfTileDistance));

        encoderDrive(.5, (allianceLocationFactor * (fullTileDistance+halfTileDistance)), (allianceLocationFactor * (fullTileDistance+halfTileDistance)));
        // Turn to align with cones [use color sensor later?]
        encoderDrive(.5, 65 * allianceLocationFactor, -65*allianceLocationFactor);
        //PLACEHOLDER CODE FOR PLACING CONE
        sleep(2000);
        // Turn to align with cones [use color sensor later?]
        encoderDrive(.5, -65 * allianceLocationFactor, 65*allianceLocationFactor);
        //drive to Stacked Cone line
        encoderDrive(.5, (-1*allianceLocationFactor * halfTileDistance), (-1*allianceLocationFactor * halfTileDistance));
    }

}


