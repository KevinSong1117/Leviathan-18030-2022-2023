package org.firstinspires.ftc.teamcode.drive.teleops;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="teleOp", group="teleOp")
public class colorCheck extends LinearOpMode {
    private Vision vision;

    @RequiresApi(api = Build.VERSION_CODES.Q)
    @Override
    public void runOpMode() throws InterruptedException {
        while(!isStarted()){
            vision.gettrueColor(322, 200);
        }

        //IF THE WRIST EVER POINTS DOWN, YOU ARE BONED. GG. UNLUCKY
        
    }
}
