package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_RIGHT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.CONE_INTAKE_HEIGHT_CHANGE_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.EIGHTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.QUARTER_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.SIXTEENTH_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.W_3_JUNCTION;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.X_2_JUNCTION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import javax.xml.stream.util.XMLEventConsumer;

public class Gamepad1Controls {

    public boolean g1Dpad_upToggleReady;
    public boolean g1Dpad_downToggleReady;
    public boolean g1Dpad_leftToggleReady;
    public boolean g1Dpad_rightToggleReady;

    public boolean g1A_ToggleReady;
    public boolean g1B_ToggleReady;
    public boolean g1X_ToggleReady;
    public boolean g1Y_ToggleReady;

    public boolean g1Left_bumperToggleReady;
    public boolean g1Right_bumperToggleReady;

    LinearOpMode activeOpMode;

    public Gamepad1Controls(LinearOpMode activeOpMode) {
        this.activeOpMode = activeOpMode;
    }

    public void CheckControls(Gamepad currentGamepad1, DriveTrain MecDrive) {

        MecDrive.drive = -currentGamepad1.left_stick_y; //-1.0 to 1.0
        MecDrive.strafe = currentGamepad1.left_stick_x; //-1.0 to 1.0
        MecDrive.turn = currentGamepad1.right_stick_x; //-1.0 to 1.0

        //fine tune turning
        if (currentGamepad1.left_trigger > .1) {
            //how do we make it turn slower to the left based on the trigger value?
            MecDrive.turn = -currentGamepad1.left_trigger*.5; //-1.0 to 1.0
        }

        if (currentGamepad1.right_trigger > .1) {
           MecDrive.turn = currentGamepad1.right_trigger*.5;
            //CALEB: COMPLETE THIS CODE FOR FINE TUNING

        }

        MecDrive.MecanumDrive();

        // CHRIS AND OLIVER LOOK AT THIS METHOD AND EXPLAIN TO MR. GREENE OR MR. SHUNTA HOW IT WORKS AND WHAT IT IS DOING
        SetGamepad1ButtonToggles(currentGamepad1);

        if (currentGamepad1.dpad_up == true && g1Dpad_upToggleReady == true) {
            g1Dpad_upToggleReady = false;

            if (currentGamepad1.b == true) {
                MecDrive.startEncoderDrive(HIGH_SPEED, FULL_TILE_DISTANCE, FULL_TILE_DISTANCE, activeOpMode);
            } else
            {
                MecDrive.startEncoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE, HALF_TILE_DISTANCE, activeOpMode);
            }
        }

        if (currentGamepad1.dpad_down == true && g1Dpad_downToggleReady == true) {
            g1Dpad_downToggleReady = false;

            if (currentGamepad1.b == true) {
                MecDrive.startEncoderDrive(HIGH_SPEED, -FULL_TILE_DISTANCE, -FULL_TILE_DISTANCE, activeOpMode);
            } else
           MecDrive.startEncoderDrive(HIGH_SPEED, -HALF_TILE_DISTANCE, -HALF_TILE_DISTANCE, activeOpMode); {

            }
            //CALEB WRITE THIS CODE FOR THE DPAD-DOWN BUTTON SO IT MOVES A FULL TILE TOWARD THE BACK OF THE ROBOT (WHERE THE INTAKE IS) IF YOU ARE HOLDING THE B BUTTON, BUT A HALF TILE IF YOU AREN'T

        }

        if (currentGamepad1.dpad_left == true && g1Dpad_leftToggleReady == true) {
            g1Dpad_leftToggleReady = false;
            //MICHAEL WRITE THIS CODE FOR THE DPAD-LEFT BUTTON SO IT MOVES A FULL TILE TO THE LEFT IF YOU ARE HOLDING THE B BUTTON, BUT A HALF TILE IF YOU AREN'T

        }

        if (currentGamepad1.dpad_right == true && g1Dpad_rightToggleReady == true) {
            g1Dpad_rightToggleReady = false;
            //MICHAEL WRITE THIS ONE TOO - IT SHOULD BE SIMLIAR TO THE ONE YOU WROTE FOR DPAD LEFT

        }

        if (currentGamepad1.a == true && g1A_ToggleReady == true) {
            g1A_ToggleReady = false;
            //move from alliance substation to scoring position
            //MecDrive.startEncoderDrive(HIGH_SPEED, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, HALF_TILE_DISTANCE+FULL_TILE_DISTANCE+EIGHTH_TILE_DISTANCE, activeOpMode);
        }

        if (currentGamepad1.x == true && g1X_ToggleReady == true) {
            g1X_ToggleReady = false;
            //what should robot do when we press x on gamepad 1?
            //automated deliver?
        }

        if (currentGamepad1.b == true && g1B_ToggleReady == true) {
            g1B_ToggleReady = false;
            // Being used as a dpad tile distance adjustment
            // don't put code here unless aware of interaction with dpad
        }

        if (currentGamepad1.y == true && g1Y_ToggleReady == true) {
            g1Y_ToggleReady =false;
            // this is used to cancel out of the encoderDrive and strafeDrive methods during teleOp
            // don't put code here unless aware of interaction with that code
        }

        if(currentGamepad1.left_bumper == true && g1Left_bumperToggleReady == true) {
            g1Left_bumperToggleReady = false;

            Orientation orientation = MecDrive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = orientation.firstAngle;

            if (currentAngle < 0) {
                currentAngle = currentAngle + 360;
            }

            if ((currentAngle >= 0 && currentAngle < 88) || (currentAngle <= 360 && currentAngle > 355)) {
                MecDrive.turnTo(90, activeOpMode);
            } else if (currentAngle >= 87 && currentAngle < 179) {
                MecDrive.turnTo(180, activeOpMode);
            } else if (currentAngle >= 179 && currentAngle < 269) {
                MecDrive.turnTo(270, activeOpMode);
            } else if (currentAngle >= 269 && currentAngle < 359) {
                MecDrive.turnTo(0, activeOpMode);
            }
        }

        if (currentGamepad1.right_bumper == true && g1Right_bumperToggleReady == true) {
            g1Right_bumperToggleReady = false;
            //AMANDA CAN YOU WRITE THIS RIGHT BUMPER CODE TO TURN TO 0, 90, 180, 270 TO THE RIGHT THE OPPOSITE OF WHAT WE DO WITH THE LEFT BUMPER
        }
    }

    public void SetGamepad1ButtonToggles(Gamepad gamepad1) {

        //bottom button
        if (activeOpMode.opModeIsActive() && gamepad1.a == false){
            g1A_ToggleReady = true;
        }

        //left button
        if (activeOpMode.opModeIsActive() && gamepad1.x == false){
            g1X_ToggleReady = true;
        }

        //top button
        if (activeOpMode.opModeIsActive() && gamepad1.y == false){
            g1Y_ToggleReady = true;
        }

        //right button
        if (activeOpMode.opModeIsActive() && gamepad1.b == false){
            g1B_ToggleReady = true;
        }

        if (activeOpMode.opModeIsActive() && gamepad1.dpad_down == false){
            g1Dpad_downToggleReady = true;
        }
        if(activeOpMode.opModeIsActive() && gamepad1.dpad_up == false) {
            g1Dpad_upToggleReady = true;
        }
        if (activeOpMode.opModeIsActive() && gamepad1.dpad_left == false){
            g1Dpad_leftToggleReady = true;
        }
        if (activeOpMode.opModeIsActive() && gamepad1.dpad_right == false){
            g1Dpad_rightToggleReady = true;
        }

        if (activeOpMode.opModeIsActive() && gamepad1.left_bumper == false){
            g1Left_bumperToggleReady = true;
        }
        if (activeOpMode.opModeIsActive() && gamepad1.right_bumper == false){
            g1Right_bumperToggleReady = true;
        }
    }

    public void auto_deliver(int deliveryDestination, DriveTrain MecDrive, Arm ServoArm, Gamepad gamepad1, Lift Lift) {

        boolean stopExecution = false;

        MecDrive.turnTo(0, activeOpMode);

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            ServoArm.setPosition(ARM_INTAKE);
        } else stopExecution = true;

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            MecDrive.startEncoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE + QUARTER_TILE_DISTANCE), -(FULL_TILE_DISTANCE + QUARTER_TILE_DISTANCE), activeOpMode);
        } else stopExecution = true;

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            //lower lift by set amount based on current lift position
            Lift.moveLift(Lift.liftMotor.getCurrentPosition() - CONE_INTAKE_HEIGHT_CHANGE_MM, activeOpMode);
        } else stopExecution = true;

        //activate intake to grab cone

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            //raise lift by set amount based on current lift position
            Lift.moveLift(Lift.liftMotor.getCurrentPosition() + CONE_INTAKE_HEIGHT_CHANGE_MM, activeOpMode);
        } else stopExecution = true;

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            //raise lift to height to deliver to High Junction
            Lift.moveLift(MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_MM, activeOpMode);
        } else stopExecution = true;

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            //Drive toward middle of field
            if (deliveryDestination == W_3_JUNCTION) {
                MecDrive.startEncoderDrive(HIGH_SPEED, (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), (FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), activeOpMode);
            } else if (deliveryDestination == X_2_JUNCTION) {
                MecDrive.startEncoderDrive(HIGH_SPEED, ((FULL_TILE_DISTANCE * 2) + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), ((FULL_TILE_DISTANCE * 2) + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), activeOpMode);
            }
        } else stopExecution = true;

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            //raise lift to height to deliver to High Junction
            Lift.moveLift(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, activeOpMode);
        } else stopExecution = true;

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {

            if (deliveryDestination == W_3_JUNCTION) {
                //move turret to deliver position
                ServoArm.setPosition(ARM_LEFT_OUTTAKE);
            } else if (deliveryDestination == X_2_JUNCTION) {
                ServoArm.setPosition(ARM_RIGHT_OUTTAKE);
            }
        } else stopExecution = true;

        if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
            if (deliveryDestination == W_3_JUNCTION) {
                //Strafe left to the W 3 HIGH JUNCTION
                MecDrive.startStrafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), activeOpMode);
            } else if (deliveryDestination == X_2_JUNCTION) {
                //Strafe right to the W 3 HIGH JUNCTION
                MecDrive.startStrafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), activeOpMode);
            } else stopExecution = true;

            //drop off cone

            if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
                if (deliveryDestination == W_3_JUNCTION) {
                    //strafe away from the W3 JUNCTION
                    MecDrive.startStrafeDrive(HIGH_SPEED, (QUARTER_TILE_DISTANCE), (QUARTER_TILE_DISTANCE), activeOpMode);
                } else if (deliveryDestination == X_2_JUNCTION) {
                    //strafe away from the X2 JUNCTION
                    MecDrive.startStrafeDrive(HIGH_SPEED, -(QUARTER_TILE_DISTANCE), -(QUARTER_TILE_DISTANCE), activeOpMode);
                }
            } else stopExecution = true;

            if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
                //lower lift for next cone
                Lift.moveLift(ONE_CONE_INTAKE_HEIGHT_MM, activeOpMode);
            } else stopExecution = true;

            if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
                //rotate turret for next cone
                ServoArm.setPosition(ARM_INTAKE);
            } else stopExecution = true;

            if (activeOpMode.opModeIsActive() && !gamepad1.right_bumper && !stopExecution) {
                if (deliveryDestination == W_3_JUNCTION) {
                    //Drive to Alliance Station
                    MecDrive.startEncoderDrive(HIGH_SPEED, -(HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), -(HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), activeOpMode);
                } else if (deliveryDestination == X_2_JUNCTION) {
                    MecDrive.startEncoderDrive(HIGH_SPEED, -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), -(FULL_TILE_DISTANCE + HALF_TILE_DISTANCE + EIGHTH_TILE_DISTANCE), activeOpMode);
                }
            } else stopExecution = true;
        }
    }
}
