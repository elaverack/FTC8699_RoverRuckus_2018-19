package org.firstinspires.ftc.teamcode.autonomous.visuals;

// Created on 11/4/2017 at 12:14 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
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

public class VisualsHandler {

    public enum JEWEL_CONFIG {
        RED_BLUE("RED_BLUE"), BLUE_RED("BLUE_RED");

        public final String toString;

        JEWEL_CONFIG(String ts) {
            toString = ts;
        }

        public static JEWEL_CONFIG intCOMs(Point red, Point blue) {
            if (red.x < blue.x) return RED_BLUE;
            return BLUE_RED;
        }
    }

    public enum COL_CONFIG {
        LEFT("LEFT", 1), MID("MIDDLE", 2), RIGHT("RIGHT", 3);

        public final String toString;
        public final int colNum;;
        COL_CONFIG(String ts, int cn) {
            toString = ts; colNum = cn;
        }
        public static COL_CONFIG intVuMark(RelicRecoveryVuMark m) {
            if (m == RelicRecoveryVuMark.LEFT) return LEFT;
            if (m == RelicRecoveryVuMark.CENTER) return MID;
            return RIGHT;
        }
    }

    public VuforiaHandler vuforia;
    public JEWEL_CONFIG jewel_config;
    public COL_CONFIG column_config;

    public LayoutInterfacer layout;

    private OpMode opmode;

    private final static int PREVIEW_ID = 314159;
    private final static double
            SAT_L=128,
            SAT_H=255,
            VAL_L=160,
            VAL_H=255,
            RESIZE_FACT = .25;
    private final static Scalar
            RED_L = new Scalar(0,SAT_L,VAL_L),
            RED_H = new Scalar(42.5,SAT_H,VAL_H),
            BLUE_L = new Scalar(127.5,SAT_L,VAL_L),
            BLUE_H = new Scalar(198.3,SAT_H,VAL_H);
    private final static Size JEWELS_BSIZE = new Size(4,4);

    public VisualsHandler(OpMode om) {
        opmode = om;
        vuforia = new VuforiaHandler(opmode,false);
        OpenCVLoader.initDebug();
        // Initialize preview
        ImageView iv = new ImageView(opmode.hardwareMap.appContext);
        iv.setId(PREVIEW_ID);
        layout = new LayoutInterfacer(opmode, getPreviewContainer(), iv);

    }

    public LinearLayout getPreviewContainer() {
        if (layout != null) return layout.getParent();
        return (LinearLayout)((Activity)opmode.hardwareMap.appContext).findViewById(R.id.cameraMonitorViewId);
    }
    public ImageView getPreview() { return (ImageView)layout.getView(PREVIEW_ID); }
    public void close() { layout.close(); }

    public void checkJewels() throws InterruptedException {
        Mat
                start = new Mat(),
                red = new Mat(),
                blue = new Mat();

        Imgproc.cvtColor(takeMatPicture(), start, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.resize(start,start,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);

        Imgproc.blur(start, start, JEWELS_BSIZE);

        Core.inRange(start, RED_L, RED_H, red);
        Core.inRange(start, BLUE_L, BLUE_H, blue);

        jewel_config = JEWEL_CONFIG.intCOMs(getCOM(red), getCOM(blue));
    }
    public void checkJewels(Mat start) throws InterruptedException {
        Mat
                red = new Mat(),
                blue = new Mat();

        Imgproc.cvtColor(takeMatPicture(), start, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.resize(start,start,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);

        Imgproc.blur(start, start, JEWELS_BSIZE);

        Core.inRange(start, RED_L, RED_H, red);
        Core.inRange(start, BLUE_L, BLUE_H, blue);

        jewel_config = JEWEL_CONFIG.intCOMs(getCOM(red), getCOM(blue));
    }

    public void setPreview(final Bitmap image) { if (image == null) return; layout.run(new Runnable() {
            @Override
            public void run() { getPreview().setImageBitmap(image); }
        }); }
    public void setPreview(Mat image) {
        if (image == null) return;
        Bitmap bmp = Bitmap.createBitmap(image.width(), image.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(image, bmp);
        setPreview(bmp);
    }
    public void setBRGPreview(Mat image) { Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2RGB); setPreview(image); }
    public void setHSVPreview(Mat image) { Imgproc.cvtColor(image, image, Imgproc.COLOR_HSV2RGB_FULL); setPreview(image); }
    public void previewVuforia() throws InterruptedException { setPreview(fixBitmap(vuforia.takePicture())); }
    public void previewJewels() throws InterruptedException {
        Mat
                start = new Mat(),
                red = new Mat(),
                blue = new Mat(),
                jewels;
        final Point
                RCOM,
                BCOM;

        Imgproc.cvtColor(takeMatPicture(), start, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.resize(start,start,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);

        Imgproc.blur(start, start, JEWELS_BSIZE);

        Core.inRange(start, RED_L, RED_H, red);
        Core.inRange(start, BLUE_L, BLUE_H, blue);

        RCOM = getCOM(red); BCOM = getCOM(blue);
        jewel_config = JEWEL_CONFIG.intCOMs(RCOM, BCOM);

        // Preview
        jewels = start; // Note: start is hsv, but we are going to treat jewels as RGB
        jewels.setTo(new Scalar(0,0,0));
        Imgproc.cvtColor(red, red, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(blue, blue, Imgproc.COLOR_GRAY2RGB);
        Core.multiply(red, (new Mat(red.rows(), red.cols(), red.type())).setTo(new Scalar(255,0,0)), red);
        Core.multiply(blue, (new Mat(blue.rows(), blue.cols(), blue.type())).setTo(new Scalar(0,0,255)), blue);
        Core.add(red,blue,jewels);
        Imgproc.circle(jewels, RCOM, 10, new Scalar(255,255,0), -1);
        Imgproc.circle(jewels, BCOM, 10, new Scalar(0,255,255), -1);
        setPreview(jewels);
    }

    public static Point getCOM(Mat image, Scalar colorHigh, Scalar colorLow) {
        Mat mask = mask(image, colorHigh, colorLow);
        Moments mmnts = Imgproc.moments(mask,true);
        return new Point((mmnts.get_m10()/mmnts.get_m00()),(mmnts.get_m01()/mmnts.get_m00()));
    }
    public static Point getCOM(Mat binaryImage) {
        Moments mmnts = Imgproc.moments(binaryImage,true);
        return new Point((mmnts.get_m10()/mmnts.get_m00()),(mmnts.get_m01()/mmnts.get_m00()));
    }

    public void previewMask(Mat image, Scalar colorHigh, Scalar colorLow) { Mat mask = mask(image, colorHigh, colorLow); setPreview(mask); }
    public void previewCOM(Mat image, Scalar colorHigh, Scalar colorLow) {
        Imgproc.circle(image, getCOM(image, colorHigh, colorLow), 10, new Scalar(0,0,255));
        setPreview(image);
    }

    public Mat takeMatPicture() throws InterruptedException{ return BitmapToMat(fixBitmap(vuforia.takePicture())); }

    public static Bitmap fixBitmap(Bitmap orig) {
        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        return Bitmap.createBitmap(orig,0,0,orig.getWidth(),orig.getHeight(),matrix,true);
    }

    public static Mat mask(Mat image, Scalar colorHigh, Scalar colorLow) {
        Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2HSV_FULL);
        Mat mask = new Mat();
        Core.inRange(image, colorLow, colorHigh, mask);
        return mask;
    }

    private static Mat BitmapToMat(Bitmap orig) {
        if (orig == null) return null;
        Mat image = new Mat(orig.getHeight(), orig.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(orig, image);
        return image;
    }

    private static Mat generateIntro(int width) {
        final String
                l1 = "You are using",
                l2 = "Visuals Handler 1.0";
        final int
                font = Core.FONT_HERSHEY_SIMPLEX,
                size = 1,
                line_spacing = 25;
        final Scalar color = new Scalar(255);

        Mat intro = new Mat(200, width, CvType.CV_8U);
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
        return intro;
    }
}
