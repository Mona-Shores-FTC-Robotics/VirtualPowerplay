package org.firstinspires.ftc.teamcode.ObjectClasses;

import static java.lang.Math.signum;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDController2 {
    double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double m_kP = 0;
    private double m_kI = 0;
    private double m_kD = 0;
    public double m_kF = 0;
    public double m_target =0;
    public double degree_error;
    public double percent_error;
    public double last_percent_error;
    public double output;

    public TurnPIDController2(double target, double kP, double kI, double kD, double kF) {
        m_kP = kP;
        m_kI = kI;
        m_kD = kD;
        m_kF = kF;

        m_target = target;
        degree_error    = target;
        percent_error = target;

        timer.reset();
    }

    public double update(double currentState) {
        degree_error = m_target - currentState;
        percent_error = (degree_error/180);
        integralSum += percent_error * timer.seconds();
        double derivative = (percent_error - last_percent_error) / timer.seconds();
        last_percent_error = percent_error;
        timer.reset();
        output = (percent_error * m_kP) + (derivative*m_kD) + (integralSum*m_kI) + signum(percent_error)*(m_kF);
        return output;
    }

}



