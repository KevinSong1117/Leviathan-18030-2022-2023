package org.firstinspires.ftc.teamcode.drive.teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="testing", group="test")
public class Testing extends OpMode {
    public LiftState lift = LiftState.IDLE;
    DcMotor fL = hardwareMap.dcMotor.get("fL");
    DcMotor bL = hardwareMap.dcMotor.get("bL");
    DcMotor fR = hardwareMap.dcMotor.get("fR");
    DcMotor bR = hardwareMap.dcMotor.get("bR");

    DcMotor lI = hardwareMap.dcMotor.get("lI");
    DcMotor rI = hardwareMap.dcMotor.get("rI");
    Servo lO = hardwareMap.servo.get("lO");
    Servo rO = hardwareMap.servo.get("rO");
    DcMotor lL = hardwareMap.dcMotor.get("lL");
    DcMotor rL = hardwareMap.dcMotor.get("rL");
    CRServo fI = hardwareMap.get(CRServo.class, "fI");
    CRServo bI = hardwareMap.get(CRServo.class, "bI");


    @Override
    public void init() {
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

        lL.setDirection(DcMotorSimple.Direction.FORWARD);
        lL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rL.setDirection(DcMotorSimple.Direction.REVERSE);
        rL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double getLiftPos()
    {
        return Math.abs((lL.getCurrentPosition() + rL.getCurrentPosition()) / 2.0);
    }

    public void setLiftPower(double power)
    {
        lL.setPower(power);
        rL.setPower(power);
    }
    public void setLiftPos(double liftTargetPos)
    {
        if (getLiftPos() <= liftTargetPos - 50)
        {
            setLiftPower(-.9);
        }
    }

    public void liftReset(double power, double liftTargetPos)
    {
        if (getLiftPos() >= 50 + liftTargetPos)
        {
            setLiftPower(power);
        }
    }

    public enum LiftState {
        IDLE,
        RAISE,
        PLACE,
        LOWER,
    }

    @Override
    public void loop() {
        double liftp = -gamepad2.left_stick_y;
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fL.setPower(frontLeftPower);
        bL.setPower(backLeftPower);
        fR.setPower(frontRightPower);
        bR.setPower(backRightPower);
        lL.setPower(liftp/3);
        rL.setPower(liftp/3);
        if (gamepad2.x) {
            fI.setPower(.5);
            bI.setPower(.5);
        }
        if (gamepad2.y) {
            fI.setPower(0);
            bI.setPower(0);
        }
        if (gamepad2.a) {
            lO.setPosition(1);
            rO.setPosition(0);
        }
        if (gamepad2.b) {
            lO.setPosition(0);
            rO.setPosition(1);
        }
        if(gamepad2.left_bumper){
            lI.setPower(.5);
            rI.setPower(.5);
        }else{lI.setPower(0);
            rI.setPower(0);}

        /*if (gamepad1.dpad_up && jHeight < 3 && jHeightTime.milliseconds() > 200) {
            jHeight++;
            jHeightTime.reset();
        }

        if (gamepad1.dpad_down && jHeight > 0 && jHeightTime.milliseconds() > 200) {
            jHeight--;
            jHeightTime.reset();
        }

        switch (jHeight) {
            case 0:
                liftTargetPos = 100;
                break;
            case 1:
                liftTargetPos = 1000;
                break;
            case 2:
                liftTargetPos = 1570;
                break;
            case 3:
                liftTargetPos = 2320;
        }
        switch (lift) {
            case IDLE:
                if (gamepad2.a && grabTime.milliseconds() > 200) {
                    grabbed = true;
                    grabTime.reset();
                }
                if (grabbed) {
                    if (getLiftPos() > WHATEVER ENCODER VALUE YOU WANT TO DROP YOUR LIFT TO GRAB AT)
                    liftReset(FILL OUT);
                    else {
                        setLiftPower(0);
                        grab(); //spin your intake wheels
                    }
                } else {
                    if (getLiftPos() < IDLE HEIGHT)
                    setLiftPos(IDLE HEIGHT);
                    else {
                        setLiftPower(0);
                    }
                }
                if (gamepad2.right_bumper && !active) {
                    grabbed = false;
                    active = true;
                    lift = LiftState.RAISE;
                }
                break;
            case RAISE:
                if (getLiftPos() < liftTargetPos) {
                    setLiftPos(liftTargetPos);
                } else {
                    if (macroTime.milliseconds() > 400) {
                        macroTime.reset();
                        lift = LiftState.PLACE;
                    }
                }
                liftState = "RAISE";
                break;
            case PLACE:
                //code meant to give drivers leeway if the macro is bad
                if (Math.abs(gamepad2.left_stick_y) > .05) {
                    setLiftPower(gamepad2.left_stick_y * 0.2);
                } else {
                    if (jHeight == 0)
                        setLiftPower(STALL POWER);
                    else if (jHeight == 1)
                        setLiftPower(STALL POWER);
                    else
                        setLiftPower(STALL POWER);
                }
                if (gamepad2.a && grabTime.milliseconds() > 200) {
                    release(); //whatever code you want to release your cone
                    grabTime.reset();
                    lift = LiftState.LOWER;
                    macroTime.reset();
                }
                liftState = "PLACE";
                break;
            case LOWER:
                if (macroTime.milliseconds() > 200)
                {
                    if (getLiftPos() > Wherever you want to reset to) {
                    liftReset(FILL IN VALUES HERE);
                } else {
                    setLiftPower(0);
                    active = false;
                    lift = LiftState.IDLE;
                }
                }
                liftState = "LOWER";
                break;
            default:
                lift = LiftState.IDLE;
                break;
        }*/


        telemetry.addData("FL encoder value:", fL.getCurrentPosition());
        telemetry.addData("FR encoder value:", fR.getCurrentPosition());
        telemetry.addData("BL encoder value:", bL.getCurrentPosition());
        telemetry.addData("BR encoder value:", bR.getCurrentPosition());
        telemetry.addData("rL encoder value:", rL.getCurrentPosition());
        telemetry.addData("lL encoder valye:", lL.getCurrentPosition());
        telemetry.update();
    }
}
