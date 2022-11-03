package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous(group = "Real")
public class PIDtesting extends LinearOpMode {

    Hardware robot = new Hardware();
    TurnPIDController gyro;
    ElapsedTime timer;
    Hardware hardware;
    private ElapsedTime runtime = new ElapsedTime();
    private Orientation lastAngle = new Orientation();
    private double currentAngle = 0.0;
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        robot.init();
        waitForStart();

        turn(90); //should turn counterclockwise 90 degrees
        sleep(3000);
        turnTo(-90); //from current position should turn 180 degrees clockwise

    }

    public void resetAngle(){
        lastAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }
    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;

        if(deltaAngle > 180){
            deltaAngle -= 360;
        } else if (deltaAngle <= - 180){
            deltaAngle += 360;
        }

        currentAngle += deltaAngle;
        lastAngle = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currentAngle;
    }

    public void turn(double degrees){

        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2){
            double motorPower = (error < 0 ? -.3 : .3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.setAllPower(0);
    }

    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if(error > 180){
            error-=360;
        } else if(error < -180){
            error += 360;
        }
        turn(error);
    }
    public double getAbsoluteAngle(){
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnToPID(double targetAngle){
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        while (opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1){
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
        }
        robot.setAllPower(0);
    }

    public void turnPID(double degrees){
            turnToPID(degrees + getAbsoluteAngle());
    }
    public void movePIDFGyro(double inches, double kp, double ki, double kd, double f, double threshold, double time){
        timer.reset();
        hardware.resetEncoder();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();

        double initialError = Math.abs(inches); //-20
        double error = initialError;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;


        while (timeAtSetPoint < time && !isStopRequested() && opModeIsActive()) {
            telemetry.addData("angle", gyro.getAngle());
            telemetry.update();

            if (inches < 0){
                error = inches + robot.getTic() / COUNTS_PER_INCH;
            }
            else{
                error = inches - robot.getTic() / COUNTS_PER_INCH;
            }

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;

            double difference = gyro.angleDiff(initialHeading);

            if (difference > .4){
                if (power > 0) {
                    robot.setMotorPower((power + f), (power + f), (power + f), (power + f));
                }
                else {
                    robot.setMotorPower((power - f), (power - f), (power - f), (power - f));
                }
            }
            else if(difference < -.5){
                if (power > 0) {
                    robot.setMotorPower((power + f), (power + f), (power - f), (power - f));
                }
                else {
                    robot.setMotorPower((power - f),(power - f), (power - f), (power - f));
                }
            }
            else{
                if (power > 0) {
                    robot.setMotorPower(power + f,  power + f, power + f, power + f);
                }
                else {
                    robot.setMotorPower(power - f, power - f,  power - f,  power - f);
                }
            }
            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        robot.setAllPower(0);
    }


}
