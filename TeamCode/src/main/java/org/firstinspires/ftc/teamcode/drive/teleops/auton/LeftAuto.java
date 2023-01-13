package org.firstinspires.ftc.teamcode.drive.teleops.auton;

import  com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.drive.teleops.Sensors;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class LeftAuto extends LinearOpMode
{
    // Declare OpMode members.
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;  // instantiates motor variables
    public DcMotor bR;

    public DcMotor rL;
    public DcMotor lL;

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
            (WHEEL_DIAMETER_INCHES * 3.1415);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int LEFT = 3;
    int MIDDLE = 2;
    int RIGHT = 1;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         The INIT-loop
         This REPLACES waitForStart
         */
        while (!isStarted() && !isStopRequested())
        {
            fI.setPosition(.45);
            bI.setPosition(0);
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         The START command just came in: now work off the latest snapshot acquired
         during the init loop.
         */

        // update telemetry
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // Actually do something useful
        movePIDFGyro(-47,1,0,0,.15,.4,.5);
        //movePIDFGyro(-5,1,0,0,.15,.4,.5);
        //movePIDFGyro(5,1,0,0,.15,.4,.5);
        sleep(1000);
        turnRight(-38, 0, 0, 0, .35, 1.1, .5);
        sleep(1000);
        moveLiftPID(150, 0,0,0,.60,2,.5);
        sleep(1000);
        movePIDFGyro(-13,1,0,0,.15,.4,.5);
        out();
        sleep(2000);
        outtake(2000);
        in();
        sleep(1000);
        movePIDFGyro(7,1,0,0,.15,.4,.5);
        sleep(1000);
        turnLeft(0, 0, 0, 0, -.35, 1.1, .5);
        sleep(1000);
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            turnRight(-86,0, 0, 0, .35, 1.1, .5);
            sleep(1000);
            movePIDFGyro(22,1,0,0,.15,.4,.5);
        }else if(tagOfInterest.id == MIDDLE){
            //nothing is needed as it is already in parking zone
        }else if(tagOfInterest.id == RIGHT){
            turnLeft(85,0, 0, 0, -.35, 1.1, .5);
            movePIDFGyro(25,1,0,0,.15,.4,.5);
        }

        while (opModeIsActive()) {sleep(20);}
    }


    public void in(){
        lO.setPosition(1);
        rO.setPosition(0);
    }
    public void out() {
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
        lL.setPower(-power);
    }
    public void moveLiftPID(double inches, double kp, double ki, double kd, double f, double threshold, double time){
        timer.reset();
        resetLiftEncoder();

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
                setLiftPower(-power - f-.02);
            }
            else {
                setLiftPower(-power + f+.02);
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
    public void intake(){
        fI.setPosition(1);
        bI.setPosition(0);
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
    public void resetLiftEncoder(){
        lL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void outtake(int time) {
        fI.setPosition(.63);
        bI.setPosition(.33);
        sleep(time);
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
    public double angleDiffSigma(double angle1, double angle2){
        return angleWrapDeg(angle2 - angle1);
    }
    public double angleWrapDeg(double angle) {
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
        return (Math.abs(lL.getCurrentPosition()) + (rL.getCurrentPosition())/2);
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
