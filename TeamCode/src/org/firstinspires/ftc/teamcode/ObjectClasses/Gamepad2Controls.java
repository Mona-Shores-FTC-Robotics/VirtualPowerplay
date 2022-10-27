package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_LEFT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.ARM_RIGHT_OUTTAKE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain.HIGH_SPEED;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FULL_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HALF_TILE_DISTANCE;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_MM;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Gamepad2Controls {

    public boolean g2Dpad_upToggleReady;
    public boolean g2Dpad_downToggleReady;
    public boolean g2Dpad_leftToggleReady;
    public boolean g2Dpad_rightToggleReady;

    public boolean g2A_ToggleReady;
    public boolean g2B_ToggleReady;
    public boolean g2X_ToggleReady;
    public boolean g2Y_ToggleReady;

    public boolean g2Left_bumperToggleReady;
    public boolean g2Right_bumperToggleReady;

    LinearOpMode activeOpMode;
    public Gamepad2Controls(LinearOpMode activeOpMode) {
        this.activeOpMode = activeOpMode;
    }

    public void CheckControls(Gamepad currentGamepad2, Lift Lift, Arm ServoArm, Claw ServoClaw, Intake ServoIntake) {

        if (Lift.alreadyLifting ==true)
        {
            Lift.keepLifting(activeOpMode);
        }
        else if (Lift.alreadyLifting ==false) {
            Lift.ManualLift(-currentGamepad2.left_stick_y);
        }

        SetGamepad2ButtonToggles(currentGamepad2);

        // Gamepad 2 -- A Button Controls
        if (currentGamepad2.a == true && g2A_ToggleReady == true) {
            g2A_ToggleReady = false;
            //open and close the claw
            //MICHAEL PLEASE PUT THE CODE TO TOGGLE THE CLAW OPEN AND CLOSED HERE
        }

        // Gamepad 2 -- X Button Controls
        if (currentGamepad2.x == true && g2X_ToggleReady == true) {
            g2X_ToggleReady = false;
            //EVELYN PLEASE PUT THE CODE TO TOGGLE THE INTAKE ON AND OFF HERE
            ServoIntake.toggleIntake();
        }

        // Gamepad 2 -- B Button Controls
        if (currentGamepad2.b == true && g2B_ToggleReady == true) {
            g2B_ToggleReady = false;
            //what should robot do when we press B on gamepad 2?
        }

        // Gamepad 2 -- Y Button Controls
        if (currentGamepad2.y == true && g2Y_ToggleReady == true) {
            g2Y_ToggleReady =false;
            //what should robot do when we press Y on gamepad 2?
        }

        //----- Gamepad 2 -- BUMPER Button Controls -----//
        if (currentGamepad2.left_bumper == true && g2Left_bumperToggleReady == true) {
            g2Left_bumperToggleReady = false;
            Lift.startLifting(ONE_CONE_INTAKE_HEIGHT_MM, activeOpMode);
        }

        if (currentGamepad2.right_bumper == true && g2Right_bumperToggleReady==true) {
            g2Right_bumperToggleReady = false;
            Lift.startLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_MM, activeOpMode);

        }

        //----- Gamepad 2 -- DIRECTIONAL PAD Button Controls -----//
        if (currentGamepad2.dpad_down == true && g2Dpad_downToggleReady == true) {
            g2Dpad_downToggleReady = false;
            ServoArm.setPosition(ARM_INTAKE);
        }
        if (currentGamepad2.dpad_left == true && g2Dpad_leftToggleReady == true) {
            g2Dpad_leftToggleReady = false;
            //OLIVER PLEASE WRITE THE CODE TO PUT THE ARM TO THE LEFT

        }
        if (currentGamepad2.dpad_right == true && g2Dpad_rightToggleReady == true) {
            g2Dpad_rightToggleReady = false;
            //OLIVER PLEASE WRITE THE CODE TO PUT THE ARM TO THE RIGHT

        }
        if (currentGamepad2.dpad_up == true && g2Dpad_upToggleReady == true) {
            g2Dpad_upToggleReady = false;
            //Should this do anything?
        }
    }

    public void SetGamepad2ButtonToggles(Gamepad gamepad2) {

        //bottom button
        if (activeOpMode.opModeIsActive() && gamepad2.a == false){
            g2A_ToggleReady = true;
        }

        //left button
        if (activeOpMode.opModeIsActive() && gamepad2.x == false){
            g2X_ToggleReady = true;
        }

        //top button
        if (activeOpMode.opModeIsActive() && gamepad2.y == false){
            g2Y_ToggleReady = true;
        }

        //right button
        if (activeOpMode.opModeIsActive() && gamepad2.b == false){
            g2B_ToggleReady = true;
        }

        if (activeOpMode.opModeIsActive() && gamepad2.dpad_down == false){
            g2Dpad_downToggleReady = true;
        }
        if(activeOpMode.opModeIsActive() && gamepad2.dpad_up == false) {
            g2Dpad_upToggleReady = true;
        }
        if (activeOpMode.opModeIsActive() && gamepad2.dpad_left == false){
            g2Dpad_leftToggleReady = true;
        }
        if (activeOpMode.opModeIsActive() && gamepad2.dpad_right == false){
            g2Dpad_rightToggleReady = true;
        }

        if (activeOpMode.opModeIsActive() && gamepad2.left_bumper == false){
            g2Left_bumperToggleReady = true;
        }
        if (activeOpMode.opModeIsActive() && gamepad2.right_bumper == false){
            g2Right_bumperToggleReady = true;
        }
    }

}
