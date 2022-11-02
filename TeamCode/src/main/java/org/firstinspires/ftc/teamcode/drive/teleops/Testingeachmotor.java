package org.firstinspires.ftc.teamcode.drive.teleops;

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

        Servo lO = hardwareMap.servo.get("lO");
        Servo rO = hardwareMap.servo.get("rO");
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.x) {
                fI.setPower(1);
                bI.setPower(1);
            }
            if (gamepad2.y) {
                fI.setPower(0);
                bI.setPower(0);
            }
            double liftp = -gamepad2.left_stick_y;
            lL.setPower(liftp/2);
            rL.setPower(liftp/2);
            while(gamepad1.a) {
                fL.setPower(1);
                bL.setPower(1);
                fR.setPower(1);
                bR.setPower(1);
            }
            while(gamepad1.b){
                fL.setPower(.5);
                bL.setPower(.5);
                fR.setPower(.5);
                bR.setPower(.5);
            }
            while(gamepad1.y){
                fL.setPower(.2);
                bL.setPower(.2);
                fR.setPower(.2);
                bR.setPower(.2);
            }
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            bR.setPower(0);
            if (gamepad2.a) {
                lO.setPosition(1);
                rO.setPosition(0);
            }
            if (gamepad2.b) {
                lO.setPosition(0);
                rO.setPosition(1);
            }


        }
    }

}
