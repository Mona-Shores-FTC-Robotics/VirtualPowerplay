package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    public static final double ARM_CENTER_INTAKE = 0.7;
    public static final double ARM_LEFT_OUTTAKE = 1;
    public static final double ARM_RIGHT_OUTTAKE = .33;
    public static final double ARM_FRONT_OUTTAKE = 0;

    public static final double HEIGHT_FOR_PREVENTING_ARM_ROTATION = 400;
    public static final double SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION = 700;
    public static final double SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER = 1.2;

    public Servo arm;

    public armState currentArmState;
    public enum armState {  ARM_LEFT, ARM_CENTER, ARM_RIGHT, ARM_FRONT,
                            ARM_CENTERED_MOVE_LIFT_TO_INTAKE, ARM_LEFT_WAITING_FOR_LIFT,
                            ARM_RIGHT_WAITING_FOR_LIFT, ARM_FRONT_WAITING_FOR_LIFT}
    public Lift Lift;
    public ElapsedTime liftTimer = new ElapsedTime();

    public Arm(Lift m_Lift) {
       Lift = m_Lift;
    }

    public void init(HardwareMap ahwMap) {
        arm = ahwMap.servo.get("turret_servo");
        //set arm at intake position
        arm.setPosition(ARM_CENTER_INTAKE);
        currentArmState = armState.ARM_CENTER;
    }

    public void CheckArm(Boolean armLeftCurrentButton, Boolean armLeftPreviousButton,
                         Boolean armCenterCurrentButton, Boolean armCenterPreviousButton,
                         Boolean armRightCurrentButton, Boolean armRightPreviousButton,
                         Boolean armFrontCurrentButton, Boolean armFrontPreviousButton){
        if (armLeftCurrentButton && !armLeftPreviousButton) {
            setPosition(ARM_LEFT_OUTTAKE);
        } else if (armCenterCurrentButton && !armCenterPreviousButton) {
            setPosition(ARM_CENTER_INTAKE);
        } else if (armRightCurrentButton && !armRightPreviousButton) {
            setPosition(ARM_RIGHT_OUTTAKE);
        } else if (armFrontCurrentButton && !armFrontPreviousButton) {
            setPosition(ARM_FRONT_OUTTAKE);
        }
    }

    public void AdvancedCheckArm(Boolean armLeftCurrentButton, Boolean armLeftPreviousButton,
                                 Boolean armCenterCurrentButton, Boolean armCenterPreviousButton,
                                 Boolean armRightCurrentButton, Boolean armRightPreviousButton,
                                 Boolean armFrontCurrentButton, Boolean armFrontPreviousButton){
        if (armLeftCurrentButton && !armLeftPreviousButton) {
            currentArmState = armState.ARM_LEFT;
            setArmState(currentArmState);
        } else if (armCenterCurrentButton && !armCenterPreviousButton) {
            currentArmState = armState.ARM_CENTER;
            setArmState(currentArmState);
        } else if (armRightCurrentButton && !armRightPreviousButton) {
            currentArmState = armState.ARM_RIGHT;
            setArmState(currentArmState);
        } else if (armFrontCurrentButton && !armFrontPreviousButton) {
            currentArmState = armState.ARM_FRONT;
            setArmState(currentArmState);
        } else if ( currentArmState == armState.ARM_LEFT_WAITING_FOR_LIFT ||
                currentArmState == armState.ARM_RIGHT_WAITING_FOR_LIFT ||
                currentArmState == armState.ARM_FRONT_WAITING_FOR_LIFT ||
                currentArmState == armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE){
            setArmState(currentArmState);
        }
    }


    public void setPosition(double position) {
        arm.setPosition(position);
    }

    public void setArmState(armState state) {
        if (state == armState.ARM_CENTER) {
            //if the arm position isn't in the intake position already, then we need to set the lift timer so that we wait for a moment to center the arm before lowering the lift
            if (arm.getPosition() <.5 || arm.getPosition() > .9) {
                liftTimer.reset();
            }
            //Center the Arm
            arm.setPosition(ARM_CENTER_INTAKE);

            //Set the state so the lift will be lowered to the intake position on a future loop
            currentArmState = armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE;
        }

        //Check if the lift is too low and trying to rotate to a non-center position, then move the lift to a safe height before we rotate the arm
        else if (Lift.liftMotor.getCurrentPosition() < HEIGHT_FOR_PREVENTING_ARM_ROTATION && currentArmState != armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE) {

            //Raise the lift to a safe height that is well above the height for preventing arm rotation
            Lift.StartLifting(SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION);
            Lift.alreadyLifting = true;

            //change the state for left/front/right arm positions so we can check if the lift has raised to a safe height before rotating the arm
            if (state == armState.ARM_LEFT) {
                currentArmState = armState.ARM_LEFT_WAITING_FOR_LIFT;
            } else if (state == armState.ARM_RIGHT) {
                currentArmState = armState.ARM_RIGHT_WAITING_FOR_LIFT;
            } else if (state == armState.ARM_FRONT) {
                currentArmState = armState.ARM_FRONT_WAITING_FOR_LIFT;
            }
        }

        //next three else statements check whether we are at a safe lift height and then rotates the arm
        else if ((state == armState.ARM_LEFT_WAITING_FOR_LIFT || state== armState.ARM_LEFT) && Lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION ) {
            arm.setPosition(ARM_LEFT_OUTTAKE);
            currentArmState = armState.ARM_LEFT;
        } else if ((state == armState.ARM_RIGHT_WAITING_FOR_LIFT || state== armState.ARM_RIGHT) && Lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION ) {
            arm.setPosition(ARM_RIGHT_OUTTAKE);
            currentArmState = armState.ARM_RIGHT;
        } else if ((state == armState.ARM_FRONT_WAITING_FOR_LIFT || state== armState.ARM_FRONT) && Lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_FRONT_OUTTAKE);
            currentArmState = armState.ARM_FRONT;
        }

        //Lower the lift if the arm is centered and enough time has passed
        else if (state == armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE && liftTimer.seconds() > SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER) {
            Lift.StartLifting(GameConstants.ONE_CONE_INTAKE_HEIGHT_ENC_VAL);
            Lift.alreadyLifting = true;
            currentArmState = armState.ARM_CENTER;
        }
    }
}

