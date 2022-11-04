package org.firstinspires.ftc.teamcode.ObjectClasses;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    public static final double ARM_CENTER_INTAKE = 0.5;
    public static final double ARM_LEFT_OUTTAKE = 1;
    public static final double ARM_RIGHT_OUTTAKE = 0;

    public static final double HEIGHT_FOR_PREVENTING_ARM_ROTATION = 200;
    public static final double SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION = 300;
    public static final double SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER = .5;

    public Servo arm;

    public armState currentArmState;
    public enum armState {ARM_LEFT, ARM_CENTER, ARM_RIGHT, ARM_CENTERED_MOVE_LIFT_TO_INTAKE, ARM_LEFT_WAITING_FOR_LIFT, ARM_RIGHT_WAITING_FOR_LIFT}
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

    public void setArmState(armState state) {
        if (state == armState.ARM_CENTER) {
            arm.setPosition(ARM_CENTER_INTAKE);
            currentArmState = armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE;
            liftTimer.reset();
        } else if (currentArmState != armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE && Lift.liftMotor.getCurrentPosition() < HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            Lift.StartLifting(SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION);
            Lift.alreadyLifting = true;
            if (state == armState.ARM_LEFT) {
                currentArmState = armState.ARM_LEFT_WAITING_FOR_LIFT;
            } else if (state == armState.ARM_RIGHT) {
                currentArmState = armState.ARM_RIGHT_WAITING_FOR_LIFT;
            }
        } else if ((state == armState.ARM_LEFT_WAITING_FOR_LIFT || state == armState.ARM_LEFT) && Lift.liftMotor.getCurrentPosition() > HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_LEFT_OUTTAKE);
            currentArmState = armState.ARM_LEFT;
        } else if ((state == armState.ARM_RIGHT_WAITING_FOR_LIFT || state == armState.ARM_RIGHT) && Lift.liftMotor.getCurrentPosition() > HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_RIGHT_OUTTAKE);
            currentArmState = armState.ARM_RIGHT;
        } else if (state == armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE && liftTimer.seconds() > SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER) {
            Lift.StartLifting(GameConstants.ONE_CONE_INTAKE_HEIGHT_MM);
            Lift.alreadyLifting = true;
            currentArmState = armState.ARM_CENTER;
        }
    }

    public void CheckArm(Boolean armLeftCurrentButton, Boolean armLeftPreviousButton,
                         Boolean armCenterCurrentButton, Boolean armCenterPreviousButton,
                         Boolean armRightCurrentButton, Boolean armRightPreviousButton){
        if (armLeftCurrentButton && !armLeftPreviousButton) {
            setArmState(armState.ARM_LEFT);
        } else if (armCenterCurrentButton && !armCenterPreviousButton) {
            setArmState(armState.ARM_CENTER);
        } else if (armRightCurrentButton && !armRightPreviousButton) {
            setArmState(armState.ARM_RIGHT);
        } else if ( currentArmState == armState.ARM_LEFT_WAITING_FOR_LIFT ||
                    currentArmState == armState.ARM_RIGHT_WAITING_FOR_LIFT ||
                    currentArmState == armState.ARM_CENTERED_MOVE_LIFT_TO_INTAKE){
            setArmState(currentArmState);
        }
    }
}
