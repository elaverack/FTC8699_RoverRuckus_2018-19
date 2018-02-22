package org.firstinspires.ftc.teamcode.visuals;

// Created on 11/4/2017 at 12:14 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.hardware.Camera;
import android.os.Environment;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.CameraDevice;

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

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;

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

    private final static int PREVIEW_ID = 314159;
    private final static double
            SAT_L       = 128,
            SAT_H       = 255,
            VAL_L       = 160,
            VAL_H       = 255,
            RESIZE_FACT = 1;
    private final float lines_delta = 50f, circlexy_delta = 50f, circler_delta = 10f;
    private final static Scalar
            RED_L       = new Scalar(0,     SAT_L,  VAL_L),
            RED_H       = new Scalar(42.5,  SAT_H,  VAL_H),
            BLUE_L      = new Scalar(127.5, SAT_L,  VAL_L),
            BLUE_H      = new Scalar(198.3, SAT_H,  VAL_H),
            RED_C       = new Scalar(255,0,0), 
            BLUE_C      = new Scalar(0,0,255);
    private final static Size JEWELS_BSIZE = new Size(4,4);
    private final static String[] splitEnds = new String[]{
            " red.png",
            " green.png",
            " blue.png",
            " hue.png",
            " sat.png",
            " val.png"
    };
    private final String PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM).toString();


    public int vertx = 20, hory = 20;
    public AlignmentCircle 
            red = new AlignmentCircle(new Point(20,20), 50), 
            blue = new AlignmentCircle(new Point(200,20), 50);

    public VuforiaHandler vuforia;
    public LayoutInterfacer layout;

    public JEWEL_CONFIG jewel_config;
    public COL_CONFIG column_config;

    private OpMode opmode;
    private boolean hasLight = false, light = false;

    @Deprecated public VisualsHandler(OpMode om) {
        opmode = om;
        vuforia = new VuforiaHandler(opmode,false);
        OpenCVLoader.initDebug();
        // Initialize preview
        ImageView iv = new ImageView(opmode.hardwareMap.appContext);
        iv.setId(PREVIEW_ID);
        layout = new LayoutInterfacer(opmode, getPreviewContainer(), iv);
        setPreview(generateIntro(getPreviewContainer().getWidth()));
    }
    public VisualsHandler(OpMode om, boolean doVuforiaPreview) {
        opmode = om;
        //hasLight = opmode.hardwareMap.appContext.getPackageManager().hasSystemFeature(PackageManager.FEATURE_CAMERA_FLASH);
        //togglePhoneLight();
        vuforia = new VuforiaHandler(opmode,doVuforiaPreview);
        OpenCVLoader.initDebug();
        // Initialize preview
        ImageView iv = new ImageView(opmode.hardwareMap.appContext);
        iv.setId(PREVIEW_ID);
        layout = new LayoutInterfacer(opmode, getPreviewContainer(), iv);
        if (!doVuforiaPreview) showIntro();
    }

    public LinearLayout getPreviewContainer() {
        if (layout != null) return layout.getParent();
        return (LinearLayout)((Activity)opmode.hardwareMap.appContext).findViewById(R.id.cameraMonitorViewId);
    }
    public ImageView getPreview() { return (ImageView)layout.getView(PREVIEW_ID); }
    public void close() { layout.close(); phoneLightOff(); }

    public static void phoneLightOn () { CameraDevice.getInstance().setFlashTorchMode(true); }
    public static void phoneLightOff () { CameraDevice.getInstance().setFlashTorchMode(false); }
    @Deprecated public void togglePhoneLight () {
        if (!hasLight) return;
        Camera c = Camera.open();
        Camera.Parameters p = c.getParameters();
        if (light) {
            p.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
            c.setParameters(p);
            light = false; return;
        }
        p.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
        c.setParameters(p);
        light = true;
    }
    @Deprecated private void turnOffPhoneLight () {
        if (!hasLight) return;
        Camera c = Camera.open();
        Camera.Parameters p = c.getParameters();
        p.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
        c.setParameters(p);
    }

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

    public void showAlignmentLines () throws InterruptedException {
        Mat img = new Mat();
        Imgproc.resize(takeMatPicture(),img,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);
        drawVert(img, vertx);
        drawHor(img, hory);
        setPreview(img);
    }
    public void tryAlignmentLines (float vert, float hor) throws InterruptedException {
        if (vert == 0 && hor == 0) return;
        Mat img = new Mat();
        Imgproc.resize(takeMatPicture(),img,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);
        if (vert != 0) {
            vertx += (int)(vert*lines_delta);
            if (vertx < 0) vertx = 0;
            if (vertx > img.width()) vertx = img.width();
            drawVert(img, vertx);
            drawHor(img, hory);
            setPreview(img);
        }
        if (hor != 0) {
            hory += (int)(hor*lines_delta);
            if (hory < 0) hory = 0;
            if (hory > img.height()) hory = img.height();
            drawVert(img, vertx);
            drawHor(img, hory);
            setPreview(img);
        }
    }
    public void showAlignmentCircles () {
        Mat img = new Mat();
        try {
            Imgproc.resize(takeMatPicture(),img,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);
        } catch (InterruptedException e) {}
        red.draw(img, RED_C);
        blue.draw(img, BLUE_C);
        setPreview(img);
    }
    public void tryAlignmentCircles (float redx, float redy, float redr, float bluex, float bluey, float bluer) {
        if (redx == 0 && redy == 0 && redr == 0 && bluex == 0 && bluey == 0 && bluer == 0) return;
        Mat img = new Mat();
        try {
            Imgproc.resize(takeMatPicture(),img,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);
        } catch (InterruptedException e) {}
        if (redx != 0) {
            red.center.x += (int)(redx*circlexy_delta);
            if (red.center.x < 0) red.center.x = 0;
            if (red.center.x > img.width()) red.center.x = img.width();
            red.draw(img, RED_C);
            blue.draw(img, BLUE_C);
        }
        if (redy != 0) {
            red.center.y += (int)(redy*circlexy_delta);
            if (red.center.y < 0) red.center.y = 0;
            if (red.center.y > img.height()) red.center.y = img.height();
            red.draw(img, RED_C);
            blue.draw(img, BLUE_C);
        }
        if (redr != 0) {
            red.radius += (int)(redr*circler_delta);
            if (red.radius < 1) red.radius = 1;
            red.draw(img, RED_C);
            blue.draw(img, BLUE_C);
        }
        if (bluex != 0) {
            blue.center.x += (int)(bluex*circlexy_delta);
            if (blue.center.x < 0) blue.center.x = 0;
            if (blue.center.x > img.width()) blue.center.x = img.width();
            red.draw(img, RED_C);
            blue.draw(img, BLUE_C);
        }
        if (bluey != 0) {
            blue.center.y += (int)(bluey*circlexy_delta);
            if (blue.center.y < 0) blue.center.y = 0;
            if (blue.center.y > img.height()) blue.center.y = img.height();
            red.draw(img, RED_C);
            blue.draw(img, BLUE_C);
        }
        if (bluer != 0) {
            blue.radius += (int)(bluer*circler_delta);
            if (blue.radius < 1) blue.radius = 1;
            red.draw(img, RED_C);
            blue.draw(img, BLUE_C);
        }
        setPreview(img);
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

    public Mat takeMatPicture() throws InterruptedException { return BitmapToMat(fixBitmap(vuforia.takePicture())); }

    public Mat takeAndSavePic(String name) throws InterruptedException { Mat ret = takeMatPicture(); savePhoto(ret, name); return ret; }
    public Mat takeSaveSplitPic(String name) throws InterruptedException { Mat ret = takeMatPicture(); saveSplitPhoto(ret, name); return ret; }

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
    private static Bitmap MatToBitmap(Mat orig, Bitmap.Config type) {
        Bitmap bmp = Bitmap.createBitmap(orig.width(), orig.height(), type);
        Utils.matToBitmap(orig, bmp);
        return bmp;
    }

    public void showIntro () { setPreview(generateIntro(getPreviewContainer().getWidth())); }
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

    private static Mat drawVert(Mat mat, int x) {
        Imgproc.line(mat, new Point(x,0), new Point(x,mat.height()), new Scalar(255,0,0), 8);
        return mat;
    }
    private static Mat drawHor(Mat mat, int y) {
        Imgproc.line(mat, new Point(0,y), new Point(mat.width(),y), new Scalar(255,0,0), 8);
        return mat;
    }

    private void savePhoto (Mat mat, String fileName) {

        Bitmap bmp = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat, bmp);

        File file = new File(PHOTO_DIRECTORY, fileName);

        if (file.exists()) {
            String newFileName = fileName.split(".png")[0];
            int i = 2; file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png");
            while (file.exists()) { i++; file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png"); }
        }

        if (bmp != null) {
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            bmp.compress(Bitmap.CompressFormat.PNG, 0, bos);
            byte[] bitmapdata = bos.toByteArray();
            try { FileOutputStream f = new FileOutputStream(file); f.write(bitmapdata); f.flush(); f.close(); }
            catch (Exception e) { /* meh */ }
        }

    }

    /**
     * Take an image (mat), save it to the save directory, then split the rgb and hsv and save those to a folder.
     * @param toSplit The image to save and split.
     * @param filename The name of the image file (ENDING IN .PNG).
     */
    private void saveSplitPhoto (Mat toSplit, String filename) {

        Bitmap bmp = Bitmap.createBitmap(toSplit.width(), toSplit.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(toSplit, bmp);

        filename = filename.split(".png")[0];
        File file = new File(PHOTO_DIRECTORY, filename + ".png");

        if (file.exists()) {
            int i = 2;
            file = new File(PHOTO_DIRECTORY, filename + " (" + i + ").png");
            while (file.exists()) { i++; file = new File(PHOTO_DIRECTORY, filename + " (" + i + ").png"); }
            filename += "_(" + i + ")";
        }

        if (bmp != null) { // Save toSplit
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            bmp.compress(Bitmap.CompressFormat.PNG, 0, bos);
            byte[] bitmapdata = bos.toByteArray();
            try { FileOutputStream f = new FileOutputStream(file); f.write(bitmapdata); f.flush(); f.close(); }
            catch (Exception e) { /* meh */ }
        }

        ArrayList<Mat> rgb = new ArrayList<>(), hsv = new ArrayList<>();
        Core.split(toSplit, rgb);
        Imgproc.cvtColor(toSplit, toSplit, Imgproc.COLOR_RGB2HSV_FULL);
        Core.split(toSplit, hsv);

        File split_dir = new File(PHOTO_DIRECTORY + filename + "_splits");
        if (split_dir.isDirectory() && !split_dir.exists()) split_dir.mkdir();

        for (int i = 0; i < 3; i++) {

            Bitmap rgb_b = VisualsHandler.MatToBitmap(rgb.get(i), Bitmap.Config.RGB_565);
            File rgb_f = new File(split_dir, filename + splitEnds[i]);
            Bitmap hsv_b = VisualsHandler.MatToBitmap(hsv.get(i), Bitmap.Config.RGB_565);
            File hsv_f = new File(split_dir, filename + splitEnds[i + 3]);

            ByteArrayOutputStream rgb_bos = new ByteArrayOutputStream();
            rgb_b.compress(Bitmap.CompressFormat.PNG, 0, rgb_bos);
            try {
                FileOutputStream f = new FileOutputStream(rgb_f);
                f.write(rgb_bos.toByteArray());
                f.flush();
                f.close();
            } catch (Exception e) { /* meh */ }

            ByteArrayOutputStream hsv_bos = new ByteArrayOutputStream();
            hsv_b.compress(Bitmap.CompressFormat.PNG, 0, hsv_bos);
            try {
                FileOutputStream f = new FileOutputStream(hsv_f);
                f.write(hsv_bos.toByteArray());
                f.flush();
                f.close();
            } catch (Exception e) { /* meh */ }

        }
        
    }

}
