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
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lL = hardwareMap.dcMotor.get("lL");
        DcMotor rL = hardwareMap.dcMotor.get("rL");
        DcMotor fL = hardwareMap.dcMotor.get("fL");
        DcMotor bL = hardwareMap.dcMotor.get("bL");
        DcMotor fR = hardwareMap.dcMotor.get("fR");
        DcMotor bR = hardwareMap.dcMotor.get("bR");
        Servo fI = hardwareMap.get(Servo.class, "fI");
        Servo bI = hardwareMap.get(Servo.class, "bI");

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            boolean slow = false;
            if (gamepad1.left_bumper)
                slow = !slow;

            if (slow) {
                frontLeftPower /= 2;
                frontRightPower /= 2;
                backLeftPower /= 2;
                backRightPower /= 2;
            }
            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);

            
            if (gamepad2.left_bumper) {
                fI.setPosition(0);
                bI.setPosition(1);
            }else if (gamepad2.right_bumper) {
                fI.setPosition(1);
                bI.setPosition(0);
            }

            double liftp = -gamepad2.left_stick_y;
            lL.setPower(liftp*.7);
            rL.setPower(liftp*.7);
            if (gamepad2.a) {
                lO.setPosition(1);
                rO.setPosition(0);
            }
            if (gamepad2.b) {
                lO.setPosition(0);
                rO.setPosition(1);
            }
            telemetry.addData("Average lift encoder value:", (lL.getCurrentPosition() + rL.getCurrentPosition())/2);
            telemetry.addData("Left Enocder", lL.getCurrentPosition());
            telemetry.addData("Right Encoder", rL.getCurrentPosition());
            telemetry.update();
            //
        }
    }

}
