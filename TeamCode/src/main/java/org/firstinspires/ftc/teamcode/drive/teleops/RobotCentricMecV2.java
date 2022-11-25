package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="mecanumDrive2", group="teleop")
public class RobotCentricMecV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx fL = hardwareMap.get(DcMotorEx.class,"fL");
        DcMotorEx bL = hardwareMap.get(DcMotorEx.class,"bL");
        DcMotorEx fR = hardwareMap.get(DcMotorEx.class,"fR");
        DcMotorEx bR = hardwareMap.get(DcMotorEx.class,"bR");

        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            /*double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(y,x);
            double power = Math.hypot(x,y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double leftFront = power * cos/max + turn;
            double rightFront = power * sin/max - turn;
            double rightBack = power * sin/max + turn;
            double leftBack = power * cos/max - turn;

            if((power + Math.abs(turn))> 1){
                leftFront /= power + turn;
                rightFront /= power + turn;
                rightBack /= power + turn;
                leftBack /= power + turn;
            }
            fL.setPower(leftFront);
            bL.setPower(leftBack);
            fR.setPower(rightFront);
            bR.setPower(rightBack);
            */

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

            fL.setVelocity(frontLeftPower);
            bL.setVelocity(backLeftPower);
            fR.setVelocity(frontRightPower);
            bR.setVelocity(backRightPower);


            telemetry.addData("FL encoder value:", fL.getCurrentPosition());
            telemetry.addData("FR encoder value:", fR.getCurrentPosition());
            telemetry.addData("BL encoder value:", bL.getCurrentPosition());
            telemetry.addData("BR encoder value:", bR.getCurrentPosition());
            telemetry.update();
        }
    }
}
