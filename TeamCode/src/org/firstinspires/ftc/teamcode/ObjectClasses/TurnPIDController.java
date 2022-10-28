package org.firstinspires.ftc.teamcode.ObjectClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDController {
    public double targetPIDAngle;
    public double pidAngleLeftToTurn;
    private double kP, kI, kD;
    private double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastAngleLeftToTurn = 0;
    private double lastTime = 0;

    public TurnPIDController (double target, double p, double i, double d) {
        targetPIDAngle = target;
        pidAngleLeftToTurn = targetPIDAngle;
        kP = p;
        kI = i;
        kD = d;
    }
    public double update(double currentAngle) {
        //P
        pidAngleLeftToTurn = targetPIDAngle - currentAngle;

        if (pidAngleLeftToTurn > 180) {
            pidAngleLeftToTurn -= 360;
        } else if (pidAngleLeftToTurn < -180)
        {
            pidAngleLeftToTurn +=360;
        }

        //I
        accumulatedError +=pidAngleLeftToTurn;
        if (Math.abs(pidAngleLeftToTurn) < 1){
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(pidAngleLeftToTurn);

        //D
        double slope = 0;
        if (lastTime > 0) {
            slope = (pidAngleLeftToTurn - lastAngleLeftToTurn) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastAngleLeftToTurn = pidAngleLeftToTurn;

        //motor power calculation
        double motorPower = -Math.tanh((kP * pidAngleLeftToTurn +kI *accumulatedError + kD*slope));
        return motorPower;
    }
}
