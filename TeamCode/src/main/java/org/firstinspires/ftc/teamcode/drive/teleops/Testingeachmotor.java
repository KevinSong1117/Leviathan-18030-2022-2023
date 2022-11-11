package org.firstinspires.ftc.teamcode.drive.teleops;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MotorTest", group="teleop")
public class Testingeachmotor extends LinearOpMode {
    @RequiresApi(api = Build.VERSION_CODES.Q)
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lL = hardwareMap.dcMotor.get("lL");
        DcMotor rL = hardwareMap.dcMotor.get("rL");
        DcMotor fL = hardwareMap.dcMotor.get("fL");
        DcMotor bL = hardwareMap.dcMotor.get("bL");
        DcMotor fR = hardwareMap.dcMotor.get("fR");
        DcMotor bR = hardwareMap.dcMotor.get("bR");
        DcMotor lI = hardwareMap.dcMotor.get("lI");
        DcMotor rI = hardwareMap.dcMotor.get("rI");

        CRServo fI = hardwareMap.get(CRServo.class, "fI");
        CRServo bI = hardwareMap.get(CRServo.class, "bI");

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        Servo lO = hardwareMap.servo.get("lO");
        Servo rO = hardwareMap.servo.get("rO");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing multiply by 1.1 if not work
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fL.setPower(frontLeftPower*.7);
            bL.setPower(backLeftPower*.7);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);

            if (gamepad2.x) {
                fI.setPower(1);
                bI.setPower(-1);
            }else if (gamepad2.y) {
                fI.setPower(-1);
                bI.setPower(1);
            }else{fI.setPower(0);
                bI.setPower(0);}

            double liftp = -gamepad2.left_stick_y;
            lL.setPower(liftp/2);
            rL.setPower(liftp/2);
            if (gamepad2.a) {
                lO.setPosition(1);
                rO.setPosition(0);
            }
            if (gamepad2.b) {
                lO.setPosition(0);
                rO.setPosition(1);
            }
            if(gamepad1.left_bumper){
                lI.setPower(-.3);
                rI.setPower(.3);
            }else if(gamepad1.right_bumper)
            {   lI.setPower(.3);
                rI.setPower(-.3);}
            else{lI.setPower(0);
                rI.setPower(0);}


            telemetry.addData("FL encoder value:", fL.getCurrentPosition());
            telemetry.addData("FR encoder value:", fR.getCurrentPosition());
            telemetry.addData("BL encoder value:", bL.getCurrentPosition());// this is negative for some reason
            telemetry.addData("BR encoder value:", bR.getCurrentPosition());
            telemetry.update();

        }
    }

}
