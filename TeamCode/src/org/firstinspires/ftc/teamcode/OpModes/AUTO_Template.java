package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import javafx.scene.shape.HLineTo;


@Autonomous(name = "AUTO_Template")
@Disabled
public class AUTO_Template extends LinearOpMode {

    int Signal;
    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig();
    public final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        ButtonConfig.init();

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
        //Autonomous Routine Example

        MecDrive.encoderDrive(LOW_SPEED, -30, -30, this);
        MecDrive.calibrateGyro(this);

        MecDrive.encoderDrive(HIGH_SPEED, ((FULL_TILE_DISTANCE*2)+QUARTER_TILE_DISTANCE), ((FULL_TILE_DISTANCE*2)+QUARTER_TILE_DISTANCE), this);
        MecDrive.turnTo(80, this);

        // drive to high pole
        MecDrive.encoderDrive(MED_SPEED, QUARTER_TILE_DISTANCE, QUARTER_TILE_DISTANCE, this);

        //deliver initial cone
        sleep(1000);

        //MecDrive.strafeDrive(LOW_SPEED, -20, -20, this);
        //MecDrive.turnTo(-20,this);

    }
}



