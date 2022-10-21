package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDController {
    private double targetAngle;
    private double kP, kD, kI;
    private double acculumatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;
    public TurnPIDController(double target, double p, double i, double d){
        targetAngle = target;
        kP = p;
        kI = i;
        kD = d;
    }

    public double update(double currentAngle){
        //P
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if(error > 180){
            error -= 360;
        }
        //I
        acculumatedError += error;
        if(Math.abs(error) < 2){
            acculumatedError = 0;
        }
        acculumatedError = Math.abs(acculumatedError) * Math.signum(error);

        //D
        double slope = 0;
        if (lastTime > 0){
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //Motor power calculation

        double motorPower = .1 * Math.signum(error) + 0.9 * Math.tanh( kP * error + kI * acculumatedError + kD * slope);

        return motorPower;
    }
}
