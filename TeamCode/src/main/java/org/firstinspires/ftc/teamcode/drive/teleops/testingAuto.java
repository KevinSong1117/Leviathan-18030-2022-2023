package org.firstinspires.ftc.teamcode.drive.teleops;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static android.graphics.Color.green;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="autoTest", group="Real")

public class testingAuto extends LinearOpMode
{
    // Declare OpMode members.
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;  // instantiates motor variables
    public DcMotor bR;
    public DcMotor lL;
    public DcMotor rL;
    Servo lO;
    Servo rO;
    Servo fI;
    Servo bI;
    Sensors gyro;
    ElapsedTime timer;
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.7796;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException  {
        timer = new ElapsedTime();
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        lL = hardwareMap.get(DcMotor.class, "lL");
        rL = hardwareMap.get(DcMotor.class, "rL");
        lO = hardwareMap.servo.get("lO");
        rO = hardwareMap.servo.get("rO");
        fI = hardwareMap.servo.get( "fI");
        bI = hardwareMap.servo.get("bI");
        gyro = new Sensors(this);
        lL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Consider using BRAKE for the drivetrain in order to reduce inertia?
        //Very damaging to the motors however

        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
       //moveLiftPID(110, 0,0,0,.60,2,.5);
        sleep(4000);
        //18 inches
        //136,11111111111108

        //This works quite well have tested it twice
        //moveLiftPID(10, 0, 0, 0, .3, .5, .5);
        //35 inches
        //movePIDFGyro(30,1,0,0,.15,.3,.5);

        movePIDFGyro(-50,1,0,0,.15,.4,.5);
        turnLeft(30, 0, 0, 0, -.39, .9, .5);
        moveLiftPID(100, 0,0,0,.60,2,.5);
        out();
        outtake();
        in();
        moveLiftPID(-100, 0,0,0,.60,2,.5);
        turnRight(-30, 0, 0, 0, .39, .9, .5);
        movePIDFGyro(30,0, 0, 0, -.39, .9, .5);


        /*strafePIDGyro(0,0,0,.3,15,.3,.5);
        turnRight(-90, 0, 0, 0, .4, .8, .5);




        telemetry.addData("FirstAngle", gyro.getAngle());
        telemetry.update();
        */

    }


    public void deliverA(String level){
        movePIDFGyro(-4,.3,0,0,.15,.2,.5);
        turnRight(270, 0, 0, 0, .16, .25, .5);
        movePIDFGyro(25,.3,0,0,.15,.2,.5);
        movePIDFGyro(-25,.3,0,0,.15,.2,.5);
        turnRight(160, 0, 0, 0, .16, .25, .5);
        turnRight(265, 0, 0, 0, .16, .25, .5);
        movePIDFGyro(40,.9,0,0,.15,.2,.5);
    }

    public void resetEncoder() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(){
        fI.setPosition(1);
        bI.setPosition(0);
    }
    public void outtake() {
        fI.setPosition(.55);
        bI.setPosition(-.4);
        sleep(1000);
    }

    public void startMotors(double fl, double fr, double bl, double br) {
        fR.setPower(fr);
        fL.setPower(fl);
        bL.setPower(bl);
        bR.setPower(br);

        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);
        telemetry.update();
    }
    public void stopMotors() {
        fR.setPower(0);
        fL.setPower(0);
        bR.setPower(0);
        bL.setPower(0);
    }
    public double getTic() {
        double count = 4;
        if (fR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (fL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        double totaldis = Math.abs(fR.getCurrentPosition()) + Math.abs(fL.getCurrentPosition()) + Math.abs(bL.getCurrentPosition()) + Math.abs(bR.getCurrentPosition());
        if (count == 0) {
            return 1;
        }
        return totaldis / count;
    }
    public double angleDiffSigma(double angle1, double angle2)
    {
        return angleWrapDeg(angle2 - angle1);
    }

    public double angleWrapDeg(double angle)
    {
        double zeroTo360 = angle + 180;      //convert to 0-360
        double start = (zeroTo360 % 360); //will work for positive angles
        //angle is (-360, 0), add 360 to make it from 0-360
        if (start < 0)
        {
            start += 360;
        }
        return start - 180; //bring it back to -180 to 180
    }

    public double getLiftPosition(){
        return ((lL.getCurrentPosition() + rL.getCurrentPosition() )/2);
    }
    public void liftUp(double change){
        double old = getLiftPosition();
        if (change > 0)
            while (getLiftPosition() < old + change) {
                lL.setPower(.6);
                rL.setPower(.6);
            }
        else
            while (getLiftPosition() > old + change) {
                lL.setPower(-.6);
                rL.setPower(-.6);
            }
        lL.setPower(.01);
        rL.setPower(.01);
    }
    public void in(){
        lO.setPosition(1);
        rO.setPosition(0);
    }

    public void out()

    {
        lO.setPosition(0);
        rO.setPosition(1);
    }

    public void movePIDFGyro(double inches, double kp, double ki, double kd, double f, double threshold, double time){
        timer.reset();
        resetEncoder();

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
                error = inches + getTic() / COUNTS_PER_INCH;
            }
            else{
                error = inches - getTic() / COUNTS_PER_INCH;
            }

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;

            double difference = angleDiffSigma(initialHeading, gyro.getAngle());

            if (difference > .6){
                if (power > 0) {
                    startMotors((power + f), (power + f), (power + f), (power + f));
                }
                else {
                    startMotors((power - f), (power - f),(power - f),(power - f));
                }
            }
            else if(difference < -.6){
                if (power > 0) {
                    startMotors((power + f), (power + f),(power + f),(power + f));
                }
                else {
                    startMotors((power - f),(power - f),(power - f),(power - f));
                }
            }
            else{
                if (power > 0) {
                    startMotors(power + f,  power + f,power + f,power + f);
                }
                else {
                    startMotors(power - f,power - f,power - f,power - f);
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
        stopMotors();
    }
    public void strafePIDGyro(double kp, double ki, double kd, double f, double inches, double threshold, double time){
        timer.reset();
        resetEncoder();

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

        while (timeAtSetPoint < time && !isStopRequested()) {
            if (inches < 0){
                error = inches + getTic() / COUNTS_PER_INCH;
            }
            else{
                error = inches - getTic() / COUNTS_PER_INCH;
            }

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            double difference = gyro.angleDiff(initialHeading);

            if (difference > .5){
                if (power > 0) {
                    startMotors((power + f), (-power - f), (-power - f), (power + f));

                }
                else {
                    startMotors((power - f), (-power + f), (-power + f), (power - f));
                }
            }
            else if(difference < -.5){
                if (power > 0) {
                    startMotors((power + f), (-power - f), (-power - f), (power + f));

                }
                else {
                    startMotors((power - f), (-power + f), (-power + f), (power - f));
                }
            }
            else{
                if (power > 0) {
                    startMotors(power + f, -power - f, -power - f, power + f);

                }
                else {
                    startMotors(power - f, -power + f, -power + f, power - f);
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
        stopMotors();
    }

    public void turnLeft(double finalAngle, double kp, double ki, double kd, double f, double threshold, double time) {
        timer.reset();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();
        finalAngle = angleWrapDeg(finalAngle);

        double initialAngleDiff = angleDiffSigma(finalAngle, initialHeading);
        double error = initialAngleDiff;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;

        while (timeAtSetPoint < time && !isStopRequested() && opModeIsActive()) {
            error = gyro.newAngleDiff(gyro.getAngle(), finalAngle);
            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / Math.abs(initialAngleDiff);
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            if (power > 0) {
                if (Math.abs(kp) < .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors((-power - f),(power + f), -power - f,power + f);
            }
            else{
                if (Math.abs(kp) > .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors((-power + f),(power - f), power + f,power - f);
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
        stopMotors();
    }
    public void turnRight(double finalAngle, double kp, double ki, double kd, double f, double threshold, double time) {
        timer.reset();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();
        finalAngle = angleWrapDeg(finalAngle);

        double initialAngleDiff = angleDiffSigma(finalAngle, initialHeading);
        double error = initialAngleDiff;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;

        while (timeAtSetPoint < time && !isStopRequested() && opModeIsActive()) {
            error = gyro.newAngleDiff(gyro.getAngle(), finalAngle);
            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / Math.abs(initialAngleDiff);
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            if (power > 0) {
                if (Math.abs(kp) < .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors((power + f),(-power - f), power + f,-power - f);
            }
            else{
                if (Math.abs(kp) > .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors((-power + f),(power - f), power + f,power - f);
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
        stopMotors();
    }

    public void setLiftPower(double power){
        rL.setPower(power);
        lL.setPower(power);
    }

    public void moveLiftPID(double inches, double kp, double ki, double kd, double f, double threshold, double time){
        timer.reset();
        resetEncoder();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialError = Math.abs(inches); //-20
        double error = initialError;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;


        while (timeAtSetPoint < time && !isStopRequested() && opModeIsActive()) {

            if (inches < 0){
                error = inches + getLiftPosition() / COUNTS_PER_INCH;
            }
            else{
                error = inches - getLiftPosition() / COUNTS_PER_INCH;
            }

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;


            if (power > 0) {
                    setLiftPower(-power - f);
                }
                else {
                    setLiftPower(-power + f);
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
        setLiftPower(.001);
    }

}