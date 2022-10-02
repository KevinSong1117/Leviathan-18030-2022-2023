package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="mecanumDrive2", group="teleop")
public class TestMecanum2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fL = hardwareMap.dcMotor.get("fL");
        DcMotor bL = hardwareMap.dcMotor.get("bL");
        DcMotor fR = hardwareMap.dcMotor.get("fR");
        DcMotor bR = hardwareMap.dcMotor.get("bR");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;


        }


    }
}
