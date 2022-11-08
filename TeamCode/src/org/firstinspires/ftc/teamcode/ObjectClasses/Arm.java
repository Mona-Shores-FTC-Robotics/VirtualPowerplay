package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    public static final double ARM_CENTER_INTAKE = 0.66;
    public static final double ARM_LEFT_OUTTAKE = 1;
    public static final double ARM_RIGHT_OUTTAKE = .33;
    public static final double ARM_FRONT_OUTTAKE = 0;

    public static final double HEIGHT_FOR_PREVENTING_ARM_ROTATION = 400;
    public static final double SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION = 700;
    public static final double SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER = 1;

    public Servo arm;

    public armState currentArmState;
    public enum armState {ARM_LEFT, ARM_CENTER, ARM_RIGHT, ARM_FRONT, ARM_CENTERED_MOVE_LIFT_TO_INTAKE, ARM_LEFT_WAITING_FOR_LIFT, ARM_RIGHT_WAITING_FOR_LIFT, ARM_FRONT_WAITING_FOR_LIFT}
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
            arm.setPosition(ARM_CENTER_INTAKE);
            //if the lift is high, lower it to position for intaking a cone automatically
            if (Lift.liftMotor.getCurrentPosition() > GameConstants.ONE_CONE_INTAKE_HEIGHT_ENC_VAL)
            {
                currentArmState = armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE;
                liftTimer.reset();
            }

        } else if (Lift.liftMotor.getCurrentPosition() < HEIGHT_FOR_PREVENTING_ARM_ROTATION && currentArmState !=armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE) {
            Lift.StartLifting(SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION);
            Lift.alreadyLifting = true;
            if (state == armState.ARM_LEFT) {
                currentArmState = armState.ARM_LEFT_WAITING_FOR_LIFT;
            } else if (state == armState.ARM_RIGHT) {
                currentArmState = armState.ARM_RIGHT_WAITING_FOR_LIFT;
            } else if (state == armState.ARM_FRONT) {
                currentArmState = armState.ARM_FRONT_WAITING_FOR_LIFT;
            }
        } else if ((state == armState.ARM_LEFT_WAITING_FOR_LIFT || state==armState.ARM_LEFT) && Lift.liftMotor.getCurrentPosition() > HEIGHT_FOR_PREVENTING_ARM_ROTATION ) {
            arm.setPosition(ARM_LEFT_OUTTAKE);
            currentArmState = armState.ARM_LEFT;
        } else if ((state == armState.ARM_RIGHT_WAITING_FOR_LIFT|| state==armState.ARM_RIGHT) && Lift.liftMotor.getCurrentPosition() > HEIGHT_FOR_PREVENTING_ARM_ROTATION ) {
            arm.setPosition(ARM_RIGHT_OUTTAKE);
            currentArmState = armState.ARM_RIGHT;
        } else if ((state == armState.ARM_FRONT_WAITING_FOR_LIFT  || state==armState.ARM_FRONT) && Lift.liftMotor.getCurrentPosition() > HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_FRONT_OUTTAKE);
            currentArmState = armState.ARM_FRONT;
        } else if (state == armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE && liftTimer.seconds() > SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER) {
            Lift.StartLifting(GameConstants.ONE_CONE_INTAKE_HEIGHT_ENC_VAL);
            Lift.alreadyLifting = true;
            currentArmState = armState.ARM_CENTER;
        }
    }


    }

