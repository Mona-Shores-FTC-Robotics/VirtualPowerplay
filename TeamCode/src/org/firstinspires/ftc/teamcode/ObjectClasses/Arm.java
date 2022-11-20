package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_CENTER;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_CENTERED_LIFT_DELAY;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_CENTER_INTAKE_ON;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_FRONT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_FRONT_WAITING_FOR_LIFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_LEFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_LEFT_WAITING_FOR_LIFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.ARM_RIGHT_WAITING_FOR_LIFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION_WAITING_FOR_LIFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION_WAITING_FOR_LIFT;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Arm.armState.OPEN_CLAW_CENTER_ARM_LOWER_LIFT_INTAKE_ON;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_ENC_VAL;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    public static final double ARM_CENTER_INTAKE = .67;
    public static final double ARM_LEFT_OUTTAKE = 1;
    //THIS WAS .33 TODAY IF WE REPROGRAM SERVO
    public static final double ARM_RIGHT_OUTTAKE = .33;
    public static final double ARM_FRONT_OUTTAKE = 0;
    public static final double ARM_NEAR_FRONT = .15;

    public static final double HEIGHT_FOR_PREVENTING_ARM_ROTATION = 650;
    public static final double SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION = 800;

    //we had this value at 1.2 yesterday, trying .7 to see if still gets arm centered in time
    public static final double SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER = .7;

    public int autoDeliverSide;

    public Servo arm;
    public armState currentArmState;

    public enum armState {
        ARM_LEFT, ARM_CENTER, ARM_RIGHT, ARM_FRONT,
        ARM_CENTERED_LIFT_DELAY,
        ARM_CENTER_INTAKE_ON,
        OPEN_CLAW_CENTER_ARM_LOWER_LIFT_INTAKE_ON,
        AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION,
        AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION,
        AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION_WAITING_FOR_LIFT,
        AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION_WAITING_FOR_LIFT,
        ARM_LEFT_WAITING_FOR_LIFT,
        ARM_RIGHT_WAITING_FOR_LIFT,
        ARM_FRONT_WAITING_FOR_LIFT
    }

    public Lift lift;
    public Intake intake;
    public Claw claw;
    public ElapsedTime liftTimer = new ElapsedTime();
    public LinearOpMode activeOpMode;


    private double targetLiftPositionAfterArmRotation;

    public Arm(Lift m_Lift, Intake m_ServoIntake, Claw m_claw, LinearOpMode mode) {
        lift = m_Lift;
        intake = m_ServoIntake;
        claw = m_claw;
        activeOpMode = mode;
    }

    public void init(HardwareMap ahwMap) {

        arm = ahwMap.servo.get("turret_servo");
        //set arm at intake position
        arm.setPosition(ARM_CENTER_INTAKE);
        currentArmState = ARM_CENTER;
    }

    public void init2(HardwareMap ahwMap) {
        arm = ahwMap.servo.get("turret_servo");
        //set arm at intake position
        arm.setPosition(ARM_NEAR_FRONT);
        currentArmState = armState.ARM_FRONT;

    }

    public void CheckArm(Boolean armLeftCurrentButton, Boolean armLeftPreviousButton,
                         Boolean armCenterCurrentButton, Boolean armCenterPreviousButton,
                         Boolean armRightCurrentButton, Boolean armRightPreviousButton,
                         Boolean armFrontCurrentButton, Boolean armFrontPreviousButton,
                         Boolean autoIntakeCurrentButton, Boolean autoIntakePreviousButton,
                         Boolean autoOuttakeCurrentButton, Boolean autoOuttakePreviousButton,
                         Boolean modButton1, Boolean modButton2) {
        if (armLeftCurrentButton && !armLeftPreviousButton) {
            setArmState(ARM_LEFT);
        } else if (armCenterCurrentButton && !armCenterPreviousButton) {
            setArmState(ARM_CENTER);
        } else if (armRightCurrentButton && !armRightPreviousButton) {
            setArmState(ARM_RIGHT);
        } else if (armFrontCurrentButton && !armFrontPreviousButton) {
            setArmState(ARM_FRONT);
        } else if (autoIntakeCurrentButton && !autoIntakePreviousButton) {
            if (modButton1) {
                autoDeliverSide = 1;
                setArmState(AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION);
            }else if (modButton2) {
                autoDeliverSide = 2;
                setArmState(AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION);
            } else {
                setArmState(AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION);
            }
        } else if (autoOuttakeCurrentButton && !autoOuttakePreviousButton) {
            setArmState(OPEN_CLAW_CENTER_ARM_LOWER_LIFT_INTAKE_ON);
        }
        //if no lift controls are being operated, hold the position of the arm by setting the servo to its current position
        else if (currentArmState == ARM_CENTER ||
                currentArmState == ARM_LEFT || currentArmState == ARM_LEFT_WAITING_FOR_LIFT ||
                currentArmState == ARM_RIGHT || currentArmState == ARM_RIGHT_WAITING_FOR_LIFT   ||
                currentArmState == ARM_FRONT || currentArmState == ARM_FRONT) {
            setArmState(currentArmState);
        }

        //if no lift controls are being operated, and the arm just centered, then lower lift to intake position
        else if (currentArmState == ARM_CENTERED_LIFT_DELAY) {
            setArmState(currentArmState);
        }

        //waiting for the arm to get above the safe rotate height for this automation
        else if (currentArmState == AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION_WAITING_FOR_LIFT) {
            setArmState(currentArmState);
        }
        //waiting for the arm to get above the safe rotate height for this automation
        else if (currentArmState == AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION_WAITING_FOR_LIFT) {
            setArmState(currentArmState);
        }

    }

    public void setPosition(double position) {
        arm.setPosition(position);
    }

    public void setArmState(armState targetState) {
//bannanan
        //if centering arm from non-center position, center arm and set lift delay state
        if (targetState == ARM_CENTER && currentArmState != ARM_CENTER) {
            centerArmSetLiftDelay(ARM_CENTER, ONE_CONE_INTAKE_HEIGHT_ENC_VAL);
        }

        //if the arm is centered, and target is center, set the position to steady arm
        else if (targetState == ARM_CENTER && currentArmState == ARM_CENTER) {
            arm.setPosition(ARM_CENTER_INTAKE);
        }

        //Lower the lift if the arm is centered and enough time has passed
        else if (targetState == ARM_CENTERED_LIFT_DELAY &&
                liftTimer.seconds() > SECONDS_TO_CENTER_ARM_BEFORE_LIFT_LOWER) {
            currentArmState = ARM_CENTER;
            arm.setPosition(ARM_CENTER_INTAKE);
            lift.StartLifting(targetLiftPositionAfterArmRotation, this);
        }

        //If 1) the lift is too low, 2) trying to rotate to non-center, and 3) lift isn't already lifting,
        //  then move the lift to a safe height before rotating arm
        else if (lift.liftMotor.getCurrentPosition() < HEIGHT_FOR_PREVENTING_ARM_ROTATION &&
                (targetState == ARM_LEFT || targetState == ARM_RIGHT || targetState == ARM_FRONT) &&
                !lift.alreadyLifting) {

            //Start raising the lift to a safe height that is well above the height for preventing arm rotation
            lift.StartLifting(SAFE_HEIGHT_FOR_ALLOWING_ARM_ROTATION, this);
            lift.alreadyLifting = true;

            if (targetState==ARM_LEFT) {
                currentArmState = ARM_LEFT_WAITING_FOR_LIFT;
            }

            if (targetState==ARM_RIGHT) {
                currentArmState = ARM_RIGHT_WAITING_FOR_LIFT;
            }

            if (targetState==ARM_FRONT) {
                currentArmState = ARM_FRONT_WAITING_FOR_LIFT;
            }

        }

        //If 1) target is LEFT/RIGHT/FRONT, and 2) current lift position is above height preventing arm rotation, then rotate to target
        else if ((targetState == ARM_LEFT || targetState == ARM_LEFT_WAITING_FOR_LIFT) &&
                lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_LEFT_OUTTAKE);
            currentArmState = ARM_LEFT;
        } else if ((targetState == armState.ARM_RIGHT || targetState == ARM_RIGHT_WAITING_FOR_LIFT) &&
                lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_RIGHT_OUTTAKE);
            currentArmState = armState.ARM_RIGHT;
        } else if ((targetState == armState.ARM_FRONT || targetState == ARM_FRONT_WAITING_FOR_LIFT) &&
                lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_FRONT_OUTTAKE);
            currentArmState = armState.ARM_FRONT;
        }

        // Automated TeleOp Operator Control for shutting off intake, closing claw, lifting arm, and rotating arm to front
        else if (targetState == AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION) {
            //Turn intake off
            intake.turnIntakeOff();
            claw.closeClaw();
            lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, this);
            currentArmState = AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION_WAITING_FOR_LIFT;
        }
        // rotate the arm once the lift is high enough
        else if (targetState == AUTOMATIC_INTAKE_TO_FRONT_DELIVER_POSITION_WAITING_FOR_LIFT &&
                lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            arm.setPosition(ARM_FRONT_OUTTAKE);
            currentArmState = armState.ARM_FRONT;
        }

        // Automated TeleOp Operator Control for shutting off intake, closing claw,
        // lifting arm, and rotating arm to SIDE
        else if (targetState == AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION) {
            //Turn intake off
            intake.turnIntakeOff();
            claw.closeClaw();
            lift.StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, this);
            currentArmState = AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION_WAITING_FOR_LIFT;
        }
        // rotate the arm once the lift is high enough
        else if (targetState == AUTOMATIC_INTAKE_TO_SIDE_DELIVER_POSITION_WAITING_FOR_LIFT &&
                lift.liftMotor.getCurrentPosition() >= HEIGHT_FOR_PREVENTING_ARM_ROTATION) {
            if (autoDeliverSide==1){
                arm.setPosition(ARM_RIGHT_OUTTAKE);
                currentArmState = ARM_RIGHT;
            }
            else if (autoDeliverSide==2){
                arm.setPosition(ARM_LEFT_OUTTAKE);
                currentArmState = ARM_LEFT;
            }
        }


        //Automated TeleOp Operator Control for opening claw, centering the arm,
        // closing claw for easy intake,  turning the intake on, and lowering the lift
        else if (targetState == OPEN_CLAW_CENTER_ARM_LOWER_LIFT_INTAKE_ON) {
            //Open the claw to release the cone
            claw.openClaw();

            // small delay before rotating
            activeOpMode.sleep(250);

            //this will close the claw to easy intake, turn the intake on, and lower lift
            centerArmSetLiftDelay(ARM_CENTER_INTAKE_ON, ONE_CONE_INTAKE_HEIGHT_ENC_VAL);
        }
    }

    public void centerArmSetLiftDelay(armState targetState, double target) {
        if (targetState == ARM_CENTER || targetState == ARM_CENTER_INTAKE_ON) {
            //if the arm position isn't in the intake position already, then we need to set the lift timer so that we wait for a moment to center the arm before lowering the lift
            if (currentArmState != ARM_CENTER) {
                liftTimer.reset();
                targetLiftPositionAfterArmRotation = target;
            }
            //Center the Arm
            arm.setPosition(ARM_CENTER_INTAKE);

            //Set the targetState so the lift will be lowered to the intake position on a future loop once enough time to ensure the arm is centered has passed
            currentArmState = ARM_CENTERED_LIFT_DELAY;

            //Set the claw to a partially closed position for easy intake of cones
            claw.setEasyIntake();

            if (targetState == ARM_CENTER_INTAKE_ON) {
                intake.turnIntakeOn();
            }
        }
    }
}


