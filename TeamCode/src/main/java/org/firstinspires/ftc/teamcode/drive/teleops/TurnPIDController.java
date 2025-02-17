package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class TurnPIDController {
    Orientation angles;

    public BNO055IMU gyro;
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
    public void updateAngle() {
        angles = gyro.getAngularOrientation();
    }

    public double getAngle() {
        updateAngle();
        return angles.firstAngle;
    }

    public double newAngleDiff(double angle1, double angle2)
    {
        if (angle1 >= 0 && angle2 >= 0 || angle1 <= 0 && angle2 <= 0)
        { //curr & goal are both positive or both negative
            return -(angle1 - angle2);
        }
        else if (Math.abs(angle1 - angle2) <= 180)
        { //diff btwn curr & goal is less than or equal to 180
            return -(angle1 - angle2);
        }
        else if (angle1 > angle2)
        { //curr is greater than goal
            return (360 - (angle1 - angle2));
        }
        else
        { //goal is greater than curr
            return -(360 + (angle1 - angle2));
        }
    }

    public double angleDiff(double goalAngle) {
        double currAngle = getAngle();
        if (currAngle >= 0 && goalAngle >= 0 || currAngle <= 0 && goalAngle <= 0) { //curr & goal are both positive or both negative
            return -(currAngle - goalAngle);
        } else if (Math.abs(currAngle - goalAngle) <= 180) {//diff btwn curr & goal is less than or equal to 180
            return -(currAngle - goalAngle);
        } else if (currAngle > goalAngle) {//curr is greater than goal
            return (360 - (currAngle - goalAngle));
        } else {//goal is greater than curr
            return -(360 + (currAngle - goalAngle));
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
