package org.firstinspires.ftc.teamcode.drive.teleops;

import android.graphics.Bitmap;
import android.os.Build;

import androidx.annotation.RequiresApi;

import static android.graphics.Color.red;
import static android.graphics.Color.green;
import static android.graphics.Color.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class Vision extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        throw new UnsupportedOperationException();
    }

    LinearOpMode opMode;
    VuforiaLocalizer vuforia;

    public Vision(LinearOpMode opMode) throws InterruptedException{
        this.opMode = opMode;
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = "AeYdBDf/////AAABmQTuBZOhz0bRuAsAOwxGPfNN1HwhR1DVfpt9k7Gj6MnpFCjkQCrgdnoYMMN1cSd3cpoQkWqmfSg+2a8OE8sTQR4A7wO4cVmyDilb29WXtggQAUaicXlrfJPOCvTovN5QIV0bhPuqE7mBACUBBAapP9KYCTqovR218Z6J6Fc3NYh5PR0ez8WdfCu3tKw/McP99K5GzQRVOaFDCzuSXMlchYSAxHP0AqfOSd2MRn9xMW48UQPMPhdQS+I4sJ9pt/q3K4Aj1x7ttSo3YJbgwbeDr3VNM2qcmm8LmIXEU/jKRom3TNG14iPWB4Po0QdUuOBAcSPJsVGJMG/cUBRNgvyGDc9DGPez/BrC8p5juRzbMV1P";
        params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();
    }

    public Bitmap getImage() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImages = frame.getNumImages();
        Image rgb = null;
        for (int i = 0; i < numImages; i++) {
            Image img = frame.getImage(i);
            int fmt = img.getFormat();
            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());
        return bm;
    }

    public boolean isGreen(int pixel) {
        boolean color = (green(pixel) <= 125) && (green(pixel) >= 70) && (red(pixel) <= 105) && (blue(pixel) <= 140);
        return color;
    }
    @RequiresApi(api = Build.VERSION_CODES.Q)
    public void gettrueColor(int x, int y)throws InterruptedException{

        Bitmap rgbImage = getImage();

        int pixel = rgbImage.getPixel(x, y);
        // w = 640
        // h = 480
        telemetry.addData("w", rgbImage.getWidth());
        telemetry.addData("h", rgbImage.getHeight());

        telemetry.addData("red", red(pixel));
        telemetry.addData("green", green(pixel));

        telemetry.addData("blue", blue(pixel));
        telemetry.addData("Color", rgbImage.getColor(x, y));

        telemetry.update();

    }

}





