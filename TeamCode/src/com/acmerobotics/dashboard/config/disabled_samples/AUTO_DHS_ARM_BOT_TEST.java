package com.acmerobotics.dashboard.config.disabled_samples;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.LOW_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;


@Autonomous(name = "DHS ARM TEST")
@Disabled
public class AUTO_DHS_ARM_BOT_TEST extends LinearOpMode {

    int Signal;

    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();
    public final ElapsedTime runtime = new ElapsedTime();

    @Override

    public void runOpMode() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo handServo = hardwareMap.servo.get("hand_servo");

        MecDrive.init(hardwareMap);
        ButtonConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        while (!isStarted()) {
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 1;

            ButtonConfig.ConfigureAllianceColor(this);
            ButtonConfig.ConfigureStartingLocation( this);
            telemetry.addData("Alliance Color ", ButtonConfig.allianceColorString);
            telemetry.addData("Starting Location ", ButtonConfig.startingLocationString);
            telemetry.update();

        }
        runtime.reset();

        //start with a cone
        handServo.setPosition(1);

        //align to the wall and calibrate gyro
        MecDrive.strafeDrive(LOW_SPEED, 10, 10, this);
        MecDrive.calibrateGyro(this);

        //strafe to line up with the cone stack
        MecDrive.strafeDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE*2 + EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE*2 + EIGHTH_TILE_DISTANCE), this);

        //drive toward middle of field
        MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);

        //strafe to the high pole

        MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);

        // Deliver initial cone
        arm.setPower(.2);
        sleep(500);
        arm.setPower(0);
        handServo.setPosition(0);
        int cones = 1;
        arm.setPower(-.2);
        sleep(500);
        arm.setPower(0);

        //strafe away from the high pole
        MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);

        while (cones < 5)
        {
            //Drive to pickup cone from cone stack
            MecDrive.encoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE, FULL_TILE_DISTANCE+HALF_TILE_DISTANCE, this);

            //pickup cone
            arm.setPower(.2);
            handServo.setPosition(1);
            sleep(1000);
            handServo.setPosition(0);


            sleep(1000);

            //Drive toward middle of field
            MecDrive.encoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE), -(FULL_TILE_DISTANCE+HALF_TILE_DISTANCE), this);

            //Strafe to the high pole
            MecDrive.strafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), this);

            //drop off cone
            cones++;
            telemetry.addData("Cones", cones);
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
            sleep(1000);

            //strafe away from the high pole
            MecDrive.strafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), this);
        }

        //Park code
        if(Signal == 1) {
            MecDrive.encoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE), -(HALF_TILE_DISTANCE), this);
        }
        else if (Signal == 2) {

            MecDrive.encoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE, HALF_TILE_DISTANCE, this);
        }
        else if (Signal == 3){
            MecDrive.encoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE, this);
        }

        sleep (5000);

    }
}



