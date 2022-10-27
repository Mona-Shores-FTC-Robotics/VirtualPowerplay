package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.Arm;
import org.firstinspires.ftc.teamcode.ObjectClasses.ButtonConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Claw;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.ObjectClasses.Lift;

import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.*;

@Autonomous(name = "AUTO_SCORE_AND_PARK_MJL")
public class AUTO_SCORE_AND_PARK_MJL extends LinearOpMode {

    int Signal;

    Arm MyArm =new Arm();
    Claw MyClaw =new Claw();
    Lift MyLift =new Lift();
    DriveTrain MecDrive = new DriveTrain();
    ButtonConfig ButtonConfig = new ButtonConfig(this);
    // int code to get Full tile distance its easier
    int FULL_TILE_DISTANCE = 50;
    int HALF_TILE_DISTANCE = FULL_TILE_DISTANCE /2;
    int QUARTER_TILE_DISTANCE = HALF_TILE_DISTANCE /2;
    int EIGHTH_TILE_DISTANCE = QUARTER_TILE_DISTANCE /2;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);
        MyLift.init(hardwareMap);
        MyArm.init(hardwareMap);
        MyClaw.init(hardwareMap);
        ButtonConfig.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(1000);

        while (!isStarted()) {
            ButtonConfig.ConfigureAllianceColor();
            ButtonConfig.ConfigureStartingPosition();
            telemetry.addData("Alliance Color ", ButtonConfig.currentAllianceColor);
            telemetry.addData("Starting Location ", ButtonConfig.currentStartPosition);
            telemetry.update();
            //Use Webcam to find out Signal and store in Signal variable
            Signal = 3;
        }

        //Autonomous Routine Example using FullTileDistance thing
        MecDrive.encoderDrive(.4, -10, -10, this);
        MecDrive.encoderDrive(.4, FULL_TILE_DISTANCE + FULL_TILE_DISTANCE + HALF_TILE_DISTANCE, FULL_TILE_DISTANCE + FULL_TILE_DISTANCE + HALF_TILE_DISTANCE, this);
        //Just_PARK drive code
        if(Signal == 1) {
           // MecDrive.turnTo(45 * ButtonConfig.allianceColorAndLocationFactor,this);
           // sleep(1000);
           // MecDrive.turnTo(0,this);
            MyLift.moveLift(84, this);
            if (ButtonConfig.allianceColorAndLocationFactor == 1){
                MyArm.setPosition(Arm.ARM_LEFT_OUTTAKE);
            }
            else if (ButtonConfig.allianceColorAndLocationFactor == -1){
                MyArm.setPosition(Arm.ARM_RIGHT_OUTTAKE);
            }
            sleep(1000);
            MyClaw.toggleClaw();
            MyArm.setPosition(Arm.ARM_INTAKE);
            MyLift.moveLift(0,this);
            MecDrive.strafeDrive(.4,  -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, this);
        }
        else if (Signal == 2) {
           // MecDrive.turnTo(45 * ButtonConfig.allianceColorAndLocationFactor,this);
            //sleep(1000);
            //MecDrive.turnTo(0,this);
            MyLift.moveLift(84, this);
            if (ButtonConfig.allianceColorAndLocationFactor == 1){
                MyArm.setPosition(Arm.ARM_LEFT_OUTTAKE);
            }
            else if (ButtonConfig.allianceColorAndLocationFactor == -1){
                MyArm.setPosition(Arm.ARM_RIGHT_OUTTAKE);
            }
            sleep(1000);
            MyClaw.toggleClaw();
            MyArm.setPosition(Arm.ARM_INTAKE);
            MyLift.moveLift(0,this);
        }

         else if (Signal == 3){
           //MecDrive.turnTo(45,this);
            //MecDrive.turnTo(45 * ButtonConfig.allianceColorAndLocationFactor,this);

            //sleep(1000);
           // MecDrive.turnTo(0,this);
            MyLift.moveLift(84, this);
               if (ButtonConfig.allianceColorAndLocationFactor == 1){
                MyArm.setPosition(Arm.ARM_LEFT_OUTTAKE);
            }
            else if (ButtonConfig.allianceColorAndLocationFactor == -1){
                MyArm.setPosition(Arm.ARM_RIGHT_OUTTAKE);
            }
            sleep(1000);
            MyClaw.toggleClaw();
            MyArm.setPosition(Arm.ARM_INTAKE);
            MyLift.moveLift(0,this);
            MecDrive.strafeDrive(.4, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, this);
        }

    }
}



