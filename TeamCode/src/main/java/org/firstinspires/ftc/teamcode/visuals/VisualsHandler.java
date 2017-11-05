package org.firstinspires.ftc.teamcode.visuals;

// Created on 11/4/2017 at 12:14 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class VisualsHandler {

    public VuforiaHandler vuforia;
    private OpMode opmode;

    private final static Scalar
            BLUE_LOW = new Scalar(100, 0, 220),
            BLUE_HIGH = new Scalar(178, 255, 255);

    public VisualsHandler(OpMode om) {
        opmode = om;
        vuforia = new VuforiaHandler(opmode,false);
        OpenCVLoader.initDebug();
    }

    public void OpenCVTest() throws InterruptedException {

        Bitmap bmp = fixBitmap(vuforia.takePicture());
        if (bmp == null) return;
        Mat mat = new Mat(new Size(bmp.getWidth(),bmp.getHeight()), CvType.CV_8UC3);
        Mat image = new Mat(bmp.getHeight(), bmp.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bmp, image);

        Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = new Mat();
        Core.inRange(image, BLUE_LOW, BLUE_HIGH, mask);

        //Moments mmnts = Imgproc.moments(mask, true);

        setPreview(mask);
    }

    public Bitmap fixBitmap(Bitmap orig) {
        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        return Bitmap.createBitmap(orig,0,0,orig.getWidth(),orig.getHeight(),matrix,true);
    }

    public void setPreview(final Bitmap image) {
        if (image == null) return;
        ((Activity)opmode.hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                try {
                    ImageView preview = (ImageView)((Activity)opmode.hardwareMap.appContext).findViewById(R.id.OpenCVPreview);
                    preview.setImageBitmap(image);
                } catch (Exception e) { /*meh*/ }
            }
        });
    }

    public void setPreview(Mat image) {
        if (image == null) return;
        Bitmap bmp = Bitmap.createBitmap(image.width(), image.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(image, bmp);
        final Bitmap fbmp = bmp;
        ((Activity)opmode.hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                try {
                    ImageView preview = (ImageView)((Activity)opmode.hardwareMap.appContext).findViewById(R.id.OpenCVPreview);
                    preview.setImageBitmap(fbmp);
                } catch (Exception e) { /*meh*/ }
            }
        });
    }

    public void previewVuforia() {
        Bitmap bmp = null;
        try {
            bmp = fixBitmap(vuforia.takePicture());
        } catch (Exception e) {/*sup.*/}
        setPreview(bmp);
    }

}
