package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="mecanumDrive1", group="teleop")
public class TestMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.dcMotor.get("fL");
        DcMotor bL = hardwareMap.dcMotor.get("bL");
        DcMotor fR = hardwareMap.dcMotor.get("fR");
        DcMotor bR = hardwareMap.dcMotor.get("bR");

        DcMotor lI = hardwareMap.dcMotor.get("lI");
        DcMotor rI = hardwareMap.dcMotor.get("rI");
        Servo lL = hardwareMap.servo.get("lL");
        Servo rL = hardwareMap.servo.get("rL");

        lI.setDirection(DcMotor.Direction.REVERSE);

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

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            fL.setPower (.1);//(frontLeftPower);
            bL.setPower (.1);//(backLeftPower);
            fR.setPower (.1);//(frontRightPower);
            bR.setPower (.1);//(backRightPower);

            if (gamepad1.a) {
                lL.setPosition(1);
                rL.setPosition(0);
            }
            if (gamepad1.b) {
                lL.setPosition(0);
                rL.setPosition(1);
            }
            while(gamepad2.left_bumper){
                lI.setPower(.5);
                rI.setPower(.5);
            }
            lI.setPower(0);
            rI.setPower(0);

            telemetry.addData("FL encoder value:", fL.getCurrentPosition());
            telemetry.addData("FR encoder value:", fR.getCurrentPosition());
            telemetry.addData("BL encoder value:", bL.getCurrentPosition());
            telemetry.addData("BR encoder value:", bR.getCurrentPosition());
            telemetry.update();
        }
    }
}
