package org.firstinspires.ftc.teamcode.visuals;

// Created on 11/4/2017 at 12:14 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.net.MailTo;
import android.support.annotation.NonNull;
import android.view.View;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class VisualsHandler {

    public VuforiaHandler vuforia;
    private OpMode opmode;

    private final static Scalar
            BLUE_LOW = new Scalar(100, 0, 220),
            BLUE_HIGH = new Scalar(178, 255, 255);

    private final static int PREVIEW_ID = 314159;

    public VisualsHandler(OpMode om) {
        opmode = om;
        vuforia = new VuforiaHandler(opmode,false);
        OpenCVLoader.initDebug();
        // Initialize preview
        ((Activity)opmode.hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                LinearLayout preview = getPreviewContainer();
                View view = ((Activity)opmode.hardwareMap.appContext).findViewById(PREVIEW_ID);
                if (view != null) preview.removeView(view);
                ImageView iv = new ImageView(opmode.hardwareMap.appContext);
                iv.setId(PREVIEW_ID);
                preview.addView(iv);

                // Show preview
                final String
                        l1 = "You are using",
                        l2 = "Visuals Handler 1.0";
                final int
                        font = Core.FONT_HERSHEY_SIMPLEX,
                        size = 1,
                        line_spacing = 25;
                final Scalar color = new Scalar(255);

                Mat intro = new Mat(200, preview.getWidth(), CvType.CV_8U);
                intro.setTo(new Scalar(0));
                final Size[] boxes = new Size[]{Imgproc.getTextSize(l1, font, size, 1, new int[1]),
                        Imgproc.getTextSize(l2, font, size, 1, new int[1])};
                final Point[] points = new Point[]{
                        new Point((intro.cols()-boxes[0].width)/2,(intro.rows()-line_spacing)/2),
                        new Point((intro.cols()-boxes[1].width)/2,(intro.rows()+line_spacing)/2+boxes[1].height)
                };
                Imgproc.putText(intro, l1, points[0], font, size, color);
                Imgproc.putText(intro, l2, points[1], font, size, color);
                Core.bitwise_not(intro, intro);

                Bitmap bmp = Bitmap.createBitmap(intro.width(), intro.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(intro, bmp);
                iv.setImageBitmap(bmp);
            }
        });
    }

    public LinearLayout getPreviewContainer() {
        return (LinearLayout)((Activity)opmode.hardwareMap.appContext).findViewById(R.id.cameraMonitorViewId);
    }

    public ImageView getPreview() {
        return (ImageView)((Activity)opmode.hardwareMap.appContext).findViewById(PREVIEW_ID);
    }

    public void close() { ((Activity)opmode.hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {getPreviewContainer().removeView(getPreview());}
        }); }

    public void OpenCVTest() throws InterruptedException {

        Bitmap bmp = fixBitmap(vuforia.takePicture());
        if (bmp == null) return;
        Mat image = new Mat(bmp.getHeight(), bmp.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bmp, image);

        Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = new Mat();
        Core.inRange(image, BLUE_LOW, BLUE_HIGH, mask);

        setPreview(mask);
    }

    public static Bitmap fixBitmap(Bitmap orig) {
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
                    ImageView preview = getPreview();
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
                    ImageView preview = getPreview();
                    preview.setImageBitmap(fbmp);
                } catch (Exception e) { /*meh*/ }
            }
        });
    }

    public void setBRGPreview(Mat image) {
        Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2RGB);
        setPreview(image);
    }

    public void setHSVPreview(Mat image) {
        Imgproc.cvtColor(image, image, Imgproc.COLOR_HSV2RGB_FULL);
        setPreview(image);
    }

    public void previewVuforia() {
        Bitmap bmp = null;
        try {
            bmp = fixBitmap(vuforia.takePicture());
        } catch (Exception e) {/*sup.*/}
        setPreview(bmp);
    }

    public static Point getCOM(Mat image, Scalar colorHigh, Scalar colorLow) {
        Mat mask = mask(image, colorHigh, colorLow);
        Moments mmnts = Imgproc.moments(mask,true);
        return new Point((mmnts.get_m10()/mmnts.get_m00()),(mmnts.get_m01()/mmnts.get_m00()));
    }

    public void previewMask(Mat image, Scalar colorHigh, Scalar colorLow) { Mat mask = mask(image, colorHigh, colorLow); setPreview(mask); }

    public void previewCOM(Mat image, Scalar colorHigh, Scalar colorLow) {
        Imgproc.circle(image, getCOM(image, colorHigh, colorLow), 10, new Scalar(0,0,255));
        setPreview(image);
    }

    public Mat takeMatPicture() throws InterruptedException{ return BitmapToMat(fixBitmap(vuforia.takePicture())); }

    private static Mat BitmapToMat(Bitmap orig) {
        if (orig == null) return null;
        Mat image = new Mat(orig.getHeight(), orig.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(orig, image);
        return image;
    }

    public static Mat mask(Mat image, Scalar colorHigh, Scalar colorLow) {
        Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2HSV_FULL);
        Mat mask = new Mat();
        Core.inRange(image, colorLow, colorHigh, mask);

        return mask;
    }

}
