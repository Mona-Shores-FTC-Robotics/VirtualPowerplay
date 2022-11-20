package org.firstinspires.ftc.teamcode.ObjectClasses;

import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.LOW_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.ONE_CONE_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;
import static org.firstinspires.ftc.teamcode.ObjectClasses.GameConstants.TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    //Lift Class Constants
    final double LIFT_LOWER_POWER = .9;
    final double LIFT_FALL_POWER = 0;
    final double LIFT_RAISE_POWER = .9;

    final int MAX_LIFT_HEIGHT = 2300;
    final int MIN_LIFT_HEIGHT = 0;

    final double BUFFER_HEIGHT = 400; // Buffer on top of crash height to give us room to stop before safe crash height
    final int SAFE_CRASH_HEIGHT = 650; // Lift can fall from this height without crashing even if arm is not centered
    final int MUST_FALL_HEIGHT = 75; // For staged movement sometimes we want to move the lift lower than the SAFE_CRASH_HEIGHT while the arm is centered, for example to pick up cones off cone stack

    //Private Lift Class Members
    private int newLiftTarget;
    private LinearOpMode activeOpMode;
    private DigitalChannel limitSwitch1;
    private enum liftJunctionStates { HIGH_CONE_JUNCTION_SCORE_HEIGHT, MEDIUM_CONE_JUNCTION_SCORE_HEIGHT,
        LOW_CONE_JUNCTION_SCORE_HEIGHT}

    private enum liftConeStackStates {   ONE_CONE_INTAKE_HEIGHT, TWO_CONE_STACK_INTAKE_HEIGHT,
        THREE_CONE_STACK_INTAKE_HEIGHT, FOUR_CONE_STACK_INTAKE_HEIGHT,
        FIVE_CONE_STACK_INTAKE_HEIGHT}
    private liftJunctionStates currentLiftJunctionState;
    private liftConeStackStates currentLiftConeStackState;

    // Public Lift Class Members
    public DcMotor liftMotor = null;
    public boolean alreadyLifting = false;

    public Lift(LinearOpMode mode){
        activeOpMode = mode;
    }

    public void init(HardwareMap ahwMap) {
        // Define and Initialize Motor
        liftMotor  = ahwMap.get(DcMotor.class, "lift_motor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(0);

        //Initialize Lift Stage States
        currentLiftJunctionState = liftJunctionStates.HIGH_CONE_JUNCTION_SCORE_HEIGHT;
        currentLiftConeStackState = liftConeStackStates.FIVE_CONE_STACK_INTAKE_HEIGHT;

        //Define limit switch
        //limitSwitch1 = ahwMap.get(DigitalChannel.class, "limit1");
    }

    public boolean LimitSwitchIsPressed() {
        //Checks if the current state of the input pin is true
        return limitSwitch1.getState();
    }

    public int getLiftTarget() {
        //Checks if the current state of the input pin is true
        return liftMotor.getTargetPosition();
    }

    public void StartLifting(double targetHeightEncVal, Arm arm) {
        if (activeOpMode.opModeIsActive()) {

            int currentLiftPosition = liftMotor.getCurrentPosition();

            //deltaLift is the amount the lift has to move to get to the new height
            //if its negative the lift needs to be lowered, if its positive the lift needs to be raised
            double deltaLift = targetHeightEncVal - currentLiftPosition;
            if (Math.abs(deltaLift) <= 50){
                deltaLift = 0;
                newLiftTarget = (int) (currentLiftPosition);
            } else {
                newLiftTarget = (int) (targetHeightEncVal);
            }

            liftMotor.setTargetPosition(newLiftTarget);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //if lift is 1) being lowered, 2) to below the MUST_FALL_HEIGHT, and 3) the arm is centered, then let the lift fall
            if (deltaLift <= 0 && newLiftTarget < MUST_FALL_HEIGHT && arm.currentArmState == Arm.armState.ARM_CENTER) {
                liftMotor.setPower(LIFT_FALL_POWER);

            }

            //if lift is being raised, no risk of crashing so give lift power
            else if (deltaLift >=0){
                liftMotor.setPower(LIFT_RAISE_POWER);
            }

            //if the lift is being lowered to a target above the SAFE CRASH HEIGHT, then give power
            else if (deltaLift < 0 && newLiftTarget >= SAFE_CRASH_HEIGHT) {
                liftMotor.setPower(LIFT_LOWER_POWER);
            }

            //if lift is 1) being lowered, 2) to below the SAFE CRASH HEIGHT, and 3) the arm is not centered,
            // then center the arm and let that code move the lift to the new target after the arm is centered
            else if (deltaLift < 0 && newLiftTarget < SAFE_CRASH_HEIGHT && arm.currentArmState != Arm.armState.ARM_CENTER) {
                arm.centerArmSetLiftDelay(Arm.armState.ARM_CENTER, newLiftTarget);
            }

            //if lift is 1) being lowered, 2) to above the MUST_FALL_HEIGHT, and 3) the arm is centered, then give lift power
            else if (deltaLift < 0 && newLiftTarget > MUST_FALL_HEIGHT && arm.currentArmState == Arm.armState.ARM_CENTER) {
                liftMotor.setPower(LIFT_LOWER_POWER);
            }

            alreadyLifting = true;
        }
    }

    public void ContinueLifting() {
        //if lift motor is busy, nothing to do
        if (liftMotor.isBusy() == true) {
        }
        //if lift has reached target lift motor wont be busy and alreadylifting state can be changed
        //This is helpful if we need to make sure the lift is at its target before moving to the next state
        else if (liftMotor.isBusy() == false) {
             alreadyLifting = false;
        }
    }

    public void ManualLift(double liftTarget, Arm arm) {
        alreadyLifting = false;

        //how do we get rid of bumpy lowering of lift? could we make step size larger when lowering the lift? would this even work? How could we quickly test?
        if (liftTarget <0) {
            newLiftTarget = MIN_LIFT_HEIGHT;
        } else if (liftTarget>0)
        {
            newLiftTarget = MAX_LIFT_HEIGHT;
        } else if (liftTarget ==0 )
        {
            if (Math.abs(liftMotor.getTargetPosition() - liftMotor.getCurrentPosition()) > 50) {
                newLiftTarget = liftMotor.getCurrentPosition();
                liftMotor.setPower(LIFT_RAISE_POWER);
            }
        }

        liftMotor.setTargetPosition(newLiftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //if the lift is being lowered and the new target is below 100 and the lift is below a safe fall height, then turn the power off and re-zero the encoder
        if (liftTarget < 0 && (liftMotor.getCurrentPosition() > SAFE_CRASH_HEIGHT + BUFFER_HEIGHT || arm.currentArmState == Arm.armState.ARM_CENTER) ){
            liftMotor.setPower(LIFT_FALL_POWER);
        }
        //if the lift is not centered then make sure to stop the lift from falling
        else if (liftTarget < 0 && arm.currentArmState != Arm.armState.ARM_CENTER){
            liftMotor.setTargetPosition(SAFE_CRASH_HEIGHT);
            liftMotor.setPower(LIFT_LOWER_POWER);
        }
        //if the lift is being raised, set the power to .8
        else if (liftTarget >0) {
            liftMotor.setPower(LIFT_RAISE_POWER);
        }
    }

    public void LowerLiftOneJunctionStage(Arm arm) {
        if (currentLiftJunctionState == liftJunctionStates.HIGH_CONE_JUNCTION_SCORE_HEIGHT) {
            currentLiftJunctionState = liftJunctionStates.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT;
            StartLifting(MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftJunctionState == liftJunctionStates.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT) {
            currentLiftJunctionState = liftJunctionStates.LOW_CONE_JUNCTION_SCORE_HEIGHT;
            StartLifting(LOW_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftJunctionState == liftJunctionStates.LOW_CONE_JUNCTION_SCORE_HEIGHT) {
            currentLiftJunctionState = liftJunctionStates.LOW_CONE_JUNCTION_SCORE_HEIGHT;
            StartLifting(LOW_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, arm);
        }
    }

    public void RaiseLiftOneJunctionStage(Arm arm) {
        if (currentLiftJunctionState == liftJunctionStates.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT) {
            currentLiftJunctionState = liftJunctionStates.HIGH_CONE_JUNCTION_SCORE_HEIGHT;
            StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftJunctionState == liftJunctionStates.LOW_CONE_JUNCTION_SCORE_HEIGHT) {
            currentLiftJunctionState = liftJunctionStates.MEDIUM_CONE_JUNCTION_SCORE_HEIGHT;
            StartLifting(MEDIUM_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, arm);
        }  else if (currentLiftJunctionState == liftJunctionStates.HIGH_CONE_JUNCTION_SCORE_HEIGHT) {
            currentLiftJunctionState = liftJunctionStates.HIGH_CONE_JUNCTION_SCORE_HEIGHT;
            StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, arm);
        }
    }

    public void LowerLiftOneConeStackStage(Arm arm) {
        if (currentLiftConeStackState == liftConeStackStates.FIVE_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.FOUR_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.FOUR_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.THREE_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.THREE_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.TWO_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.TWO_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.ONE_CONE_INTAKE_HEIGHT;
            StartLifting(ONE_CONE_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.ONE_CONE_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.ONE_CONE_INTAKE_HEIGHT;
            StartLifting(ONE_CONE_INTAKE_HEIGHT_ENC_VAL, arm);
        }
    }

    public void RaiseLiftOneConeStackStage(Arm arm) {
        if (currentLiftConeStackState == liftConeStackStates.ONE_CONE_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.TWO_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(TWO_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.TWO_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.THREE_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(THREE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.THREE_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.FOUR_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(FOUR_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.FOUR_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.FIVE_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        } else if (currentLiftConeStackState == liftConeStackStates.FIVE_CONE_STACK_INTAKE_HEIGHT) {
            currentLiftConeStackState = liftConeStackStates.FIVE_CONE_STACK_INTAKE_HEIGHT;
            StartLifting(FIVE_CONE_STACK_INTAKE_HEIGHT_ENC_VAL, arm);
        }
    }

    public void CheckLift(  double  liftStageDownCurrentTrigger, double liftStageDownPreviousTrigger,
                                    double  liftStageUpCurrentTrigger, double liftStageUpPreviousTrigger,
                                    Boolean modifierButton,
                                    double manualLiftTargetChange,
                                    Arm arm) {

        if (Math.abs(manualLiftTargetChange) <= .2){
            manualLiftTargetChange = 0;
        }

        if (manualLiftTargetChange != 0) {
            ManualLift(-manualLiftTargetChange, arm);
        }  else if (liftStageDownCurrentTrigger >= .2  && liftStageDownPreviousTrigger < .2) {
            if (!modifierButton) {
                LowerLiftOneJunctionStage(arm);
            } else if (modifierButton) {
                LowerLiftOneConeStackStage(arm);
            }
        } else if (liftStageUpCurrentTrigger >= .2 && liftStageUpPreviousTrigger < .2) {
            if (!modifierButton) {
                RaiseLiftOneJunctionStage(arm);
            } else if (modifierButton) {
                RaiseLiftOneConeStackStage(arm);
            }
            /* Consider using this code to just go between the top and the bottom if the staged lifting isn't working well
            else if (liftStageDownCurrentButton && !liftStageDownPreivousButton) {
                StartLifting(ONE_CONE_INTAKE_HEIGHT_ENC_VAL, arm);
            } else if (liftStageUpCurrentButton && !liftStageUpPreviousButton) {
                StartLifting(HIGH_CONE_JUNCTION_SCORE_HEIGHT_ENC_VAL, arm);
            */
        } else if (alreadyLifting) {
            ContinueLifting();
        } else if (manualLiftTargetChange ==0){
            //check if the limit switch is pressed (false is pressed) and reset encoder if it is
            //removed limit switch code for virtual
            if ((liftMotor.getCurrentPosition()<60) && (liftMotor.getTargetPosition() <25)){
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                currentLiftConeStackState = liftConeStackStates.ONE_CONE_INTAKE_HEIGHT;
               currentLiftJunctionState = liftJunctionStates.LOW_CONE_JUNCTION_SCORE_HEIGHT;
            }
            ManualLift(manualLiftTargetChange, arm);
        }
    }
}
