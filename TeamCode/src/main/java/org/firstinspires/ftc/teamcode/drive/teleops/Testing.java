package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="testing", group="test")
public class Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lR = hardwareMap.dcMotor.get("lR");
        DcMotor lL = hardwareMap.dcMotor.get("lL");


        lL.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double liftR = -gamepad2.right_stick_y;
            double liftL = -gamepad2.left_stick_y;



            lR.setPower(liftR/3);
            lL.setPower(liftL/3);

        }
    }
}
