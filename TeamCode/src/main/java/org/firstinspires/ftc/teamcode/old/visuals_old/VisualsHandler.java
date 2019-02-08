package org.firstinspires.ftc.teamcode.old.visuals_old;

// Created on 11/4/2017 at 12:14 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.os.Environment;
import android.util.Log;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.visuals.LayoutInterfacer;
import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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

import static org.firstinspires.ftc.teamcode.old.Enums_old.Color;
import static org.firstinspires.ftc.teamcode.old.Enums_old.JewelConfig;

public class VisualsHandler {

    public final static String TAG = "VisualsHandler";

    /** CONSTANTS */
    private final static int PREVIEW_ID = 314159;
    private final static double
            SAT_L       = 180,
            SAT_H       = 255,
            VAL_L       = 0,
            VAL_H       = 255,
            RESIZE_FACT = 1;
    private final static float lines_delta = 50f, circlexy_delta = 50f, circler_delta = 10f;
    final static Scalar
            RED_L       = new Scalar(128,   0,      0),
            RED_H       = new Scalar(255,   255,    30),
            BLUE_L      = new Scalar(0,     0,      128),
            BLUE_H      = new Scalar(255,   255,    255);
    private final static Scalar
            blue_hsv_low    = new Scalar(128,   SAT_L,  VAL_L), // Blue side crypto low
            blue_hsv_high   = new Scalar(191,   SAT_H,  VAL_H), // Blue side crypto high
            red_hsv_low     = new Scalar(0,     SAT_L,  VAL_L), // Red side crypto low
            red_hsv_high    = new Scalar(21,    SAT_H,  VAL_H); // Red side crypto high
    private final static Size JEWELS_BSIZE = new Size(4,4);
    private final static String[] splitEnds = new String[]{
            " red.png",
            " green.png",
            " blue.png",
            " hue.png",
            " sat.png",
            " val.png"
    };
    private static final String PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM).toString();

    /** VARIABLES */
    public int vertx = 20, hory = 20;
    public AlignmentCircle
            right = new AlignmentCircle(new Point(621, 1095), 80),
            left = new AlignmentCircle(new Point(378, 1107), 80);

    public VuforiaHandler   vuforia;
    public LayoutInterfacer layout;

    public JewelConfig jewelConfig = JewelConfig.ERROR;

    private OpMode opmode;
    private boolean hasLight = false, light = false;

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
    public void previewVuforia() { setPreview(fixBitmap(vuforia.takePicture())); }
    public void previewJewels () {
        Mat
                start = takeMatPicture(),
                rightc = new Mat(start.rows(), start.cols(), start.type()),
                leftc = rightc.clone(),
                preview = new Mat();

        rightc.setTo(this.right.getJewelColor(start).toScalar());
        this.right.isolate(rightc);

        leftc.setTo(this.left.getJewelColor(start).toScalar());
        this.left.isolate(leftc);

        Core.add(rightc, leftc, preview);

        setPreview(preview);

    }
    public List<Point> previewCryptobox (Color allianceColor) {
        Log.d(TAG, "- BEGIN PREVIEW CRYPTOBOX -");
        ElapsedTime localTime = new ElapsedTime();

        Mat
                start = takeMatPicture(),               // Starting image
                hsv = new Mat(),                        // Image in HSV
                lines = new Mat(),                      // For HoughLinesP
                hierarchy = new Mat(),                  // For contour finding
                end = new Mat(),                        // End result
                preview = new Mat();                    // Image for preview
        List<MatOfPoint> contours = new ArrayList<>();  // List for contour finding
        List<Point> coms = new ArrayList<>();           // List of COMs

        Imgproc.cvtColor(start, hsv, Imgproc.COLOR_RGB2HSV_FULL); // Convert start to HSV for masking

        if (allianceColor == Color.RED) { // Mask the image
            Core.inRange(hsv, red_hsv_low, red_hsv_high, end);
        } else Core.inRange(hsv, blue_hsv_low, blue_hsv_high, end);

        // Close the holes in the mask
        Imgproc.morphologyEx(end, end, Imgproc.MORPH_CLOSE,
                Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(6, 6)));

        // Look for the big vertical separators of the cryptobox in the mask
        Imgproc.HoughLinesP(end, lines, 50, Math.PI, 100, end.rows() * 2 / 3, 75);
        end.setTo(new Scalar(0)); // Empty end to prepare for line drawing
        Mat tempPreview = new Mat(start.rows(), start.cols(), start.type());

        // Draw them into one image
        for (int i = 0; i < lines.rows(); i++) {
            double[] val = lines.get(i, 0);
            if (val == null) continue;
            Imgproc.line(end, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(255), 10);
            Imgproc.line(tempPreview, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(255, 255, 255), 10);
        }

        // Close holes caused by line drawing
        Imgproc.morphologyEx(end, end,
                Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(6, 6)));
        Imgproc.morphologyEx(tempPreview, tempPreview,
                Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(6, 6)));

        // Find each individual divider and store its center of mass
        Imgproc.findContours(end, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            Mat divider = new Mat(end.rows(), end.cols(), end.type());
            Imgproc.drawContours(divider, contours, i, new Scalar(255), -1);
            coms.add(getCOM(divider));
        }

        Core.subtract(start, tempPreview, preview);
        Core.subtract(start, preview, preview);

        // Draw a circle for each center of mass
        for (Point com : coms) {
            Imgproc.circle(end, com, 25, new Scalar(128), -1);
            Imgproc.circle(preview, com, 25, new Scalar(0, 255, 0), -1);
        }

        // Lastly, set the preview window to the result
        setPreview(preview);

        Log.d(TAG, "Done previewing cryptobox. Time was " + localTime.seconds() + " seconds");

        return coms;

        // NOTE: C# code below
//        MCvScalar
//                rgb_low = new MCvScalar(0, 0, 0),
//                rgb_high = new MCvScalar(30, 30, 255),
//                hsv_low = new MCvScalar(128, 180, 0),
//                hsv_high = new MCvScalar(191, 255, 255);
//
//        showRGBImg ("crypto", rgb, .5);
//
//        //splitRGB(rgb, .5);
//
//        //splitHSV(hsv, .5);
//
//        //Mat blur = converted.Clone();
//        //Blur(blur, blur, new Size(4, 4), new Point(-1, -1));
//        //Mat dst = new Mat();
//        //Canny(blur, dst, 30, 60);
//        //showImg("edged", dst);
//        Mat hsvm = showMask("hsvmask", hsv, hsv_low, hsv_high, .5);
//        //showMask("rgbmask", rgb, rgb_low, rgb_high, .5);
//
//        MorphologyEx(hsvm, hsvm, Emgu.CV.CvEnum.MorphOp.Close,
//                GetStructuringElement(
//                        Emgu.CV.CvEnum.ElementShape.Rectangle, new Size(2, 6), new Point(-1, -1)
//                ),
//                new Point(-1, -1), 1, Emgu.CV.CvEnum.BorderType.Constant, MorphologyDefaultBorderValue
//        );
//        MorphologyEx(hsvm, hsvm, Emgu.CV.CvEnum.MorphOp.Close,
//                GetStructuringElement(
//                        Emgu.CV.CvEnum.ElementShape.Rectangle, new Size(6, 2), new Point(-1, -1)
//                ),
//                new Point(-1, -1), 1, Emgu.CV.CvEnum.BorderType.Constant, MorphologyDefaultBorderValue
//        );
//        //showImg("hsvm closed", hsvm, .5);
//        Mat hsvlines = emptyMat(hsvm);
//
//        LineSegment2D[] lines = HoughLinesP(hsvm, 50, Math.PI, 100, 1000, 75);
//        foreach (LineSegment2D line in lines) Line(hsvlines, line.P1, line.P2, new MCvScalar(255), 10);
//        //showImg("lines", hsvlines, .5);
//
//        Emgu.CV.Util.VectorOfVectorOfPoint contours = new Emgu.CV.Util.VectorOfVectorOfPoint();
//        Mat hierarchy = new Mat();
//        //Mat con = emptyMat(hsvlines, 1);
//
//        FindContours(hsvlines, contours, hierarchy,
//                Emgu.CV.CvEnum.RetrType.External, Emgu.CV.CvEnum.ChainApproxMethod.ChainApproxSimple);
//        //Random rand = new Random();
//        //for (int i = 0; i < contours.Size; i++) {
//        //    DrawContours(con, contours, i, new MCvScalar(( rand.NextDouble() + .01 ) * ( 254 )), 10);
//        //}
//        //showImg("contours", con);
//        Mat[] dividers = showSplitContours("lines", hsvlines, contours, .5);
//        for (int i = 0; i < dividers.Length; i++) {
//            Point com = getCOM(dividers[i]);
//            Circle(hsvlines, com, 25, new MCvScalar(128), -1);
//        }
//        showImg("final", hsvlines, .5);
    }

    public static List<Point> processCryptobox (Mat start, Color allianceColor) {
        Log.d(TAG, "- BEGIN PROCESS CRYPTOBOX -");

        ElapsedTime localTime = new ElapsedTime();

        Mat // Assuming start is RGB
                hsv = new Mat(),                                                // Image in HSV
                mask = new Mat(),                                               // HSV mask
                lines = new Mat(),                                              // For HoughLinesP
                linesR = new Mat(start.rows(), start.cols(), CvType.CV_8UC1),   // HoughLinesP results
                hierarchy = new Mat();                                          // For contour finding
        List<Point> ret = new ArrayList<>();                                    // List of COMs
        List<MatOfPoint> contours = new ArrayList<>();                          // For contour finding

        Imgproc.cvtColor(start, hsv, Imgproc.COLOR_RGB2HSV_FULL); // Convert start to HSV for masking

        if (allianceColor == Color.RED) { // Mask the image
            Core.inRange(hsv, red_hsv_low, red_hsv_high, mask);
        } else Core.inRange(hsv, blue_hsv_low, blue_hsv_high, mask);

        // Close the holes in the mask
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE,
                Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(6,6)));

        // Look for the big vertical separators of the cryptobox in the mask
        Imgproc.HoughLinesP(mask, lines, 50, Math.PI,
                100, mask.rows() * 2 / 3, 75);

        // Draw them into one image
        for(int i = 0; i < lines.rows(); i++) {
            double[] val = lines.get(i, 0);
            if (val == null) continue;
            Imgproc.line(linesR, new Point(val[0], val[1]),
                    new Point(val[2], val[3]), new Scalar(255), 10);
        }

        // Close holes caused by line drawing
        Imgproc.morphologyEx(linesR, linesR, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(6,6)));

        // Find each individual divider and store its center of mass
        Imgproc.findContours(linesR, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) { // For each divider...
            Mat divider = new Mat(linesR.rows(), linesR.cols(), linesR.type()); // Create an empty mat
            Imgproc.drawContours(divider, contours, i, new Scalar(255), -1); // Draw the divider
            ret.add(getCOM(divider)); // Save its center of mass
        }

        Log.d(TAG, "Done processing cryptobox. Time was " + localTime.seconds() + " seconds.");

        return ret;
    }
    public List<Point> processCryptobox (Color allianceColor) {
        return processCryptobox(takeMatPicture(), allianceColor);
    }

    public void checkJewelsWithCamera () {
        Mat img = takeMatPicture();
        jewelConfig = JewelConfig.intCircles(left.getJewelColor(img), right.getJewelColor(img));
    }

    public void showAlignmentCircles () {
        Mat img = new Mat();
        Imgproc.resize(takeMatPicture(),img,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);
        right.draw(img, Color.ERROR.toScalar());
        left.draw(img, Color.ERROR.toScalar());
        putWarning(img);
        setPreview(img);
    }
    public void tryAlignmentCircles (float redx, float redy, float redr, float bluex, float bluey, float bluer) {
        if (redx == 0 && redy == 0 && redr == 0 && bluex == 0 && bluey == 0 && bluer == 0) return;
        Mat img = new Mat();
        Imgproc.resize(takeMatPicture(),img,new Size(),RESIZE_FACT,RESIZE_FACT,Imgproc.INTER_LINEAR);
        if (redx != 0) {
            right.center.x += (int)(redx*circlexy_delta);
            if (right.center.x < 0) right.center.x = 0;
            if (right.center.x > img.width()) right.center.x = img.width();
        }
        if (redy != 0) {
            right.center.y += (int)(redy*circlexy_delta);
            if (right.center.y < 0) right.center.y = 0;
            if (right.center.y > img.height()) right.center.y = img.height();
        }
        if (redr != 0) { right.radius += (int)(redr*circler_delta); if (right.radius < 1) right.radius = 1; }
        if (bluex != 0) {
            left.center.x += (int)(bluex*circlexy_delta);
            if (left.center.x < 0) left.center.x = 0;
            if (left.center.x > img.width()) left.center.x = img.width();
        }
        if (bluey != 0) {
            left.center.y += (int)(bluey*circlexy_delta);
            if (left.center.y < 0) left.center.y = 0;
            if (left.center.y > img.height()) left.center.y = img.height();
        }
        if (bluer != 0) { left.radius += (int)(bluer*circler_delta); if (left.radius < 1) left.radius = 1; }
        right.draw(img, Color.ERROR.toScalar());
        left.draw(img, Color.ERROR.toScalar());
        setPreview(img);
    }

    private static Point getCOM(Mat binaryImage) {
        Moments mmnts = Imgproc.moments(binaryImage,true);
        return new Point((mmnts.get_m10()/mmnts.get_m00()),(mmnts.get_m01()/mmnts.get_m00()));
    }

    public Mat takeMatPicture() { return BitmapToMat(fixBitmap(vuforia.takePicture())); }

    public Mat takeAndSavePic(String name) { Mat ret = takeMatPicture(); savePhoto(ret, name); return ret; }
    public Mat takeSaveSplitPic(String name) { Mat ret = takeMatPicture(); saveSplitPhoto(ret, name); return ret; }

    private static Bitmap fixBitmap(Bitmap orig) {
        if (orig == null) return null;
        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        return Bitmap.createBitmap(orig,0,0,orig.getWidth(),orig.getHeight(),matrix,true);
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

    private static void putWarning (Mat orig) {
        String l1 = "PRESS", l2 = "HERE";
        int size = 6, line_spacing = 35, top_margin = 200, thickness = 15, font = Core.FONT_HERSHEY_SIMPLEX;
        Scalar color = new Scalar(255,0,0);

        Size[] boxes = new Size[]{
                Imgproc.getTextSize(l1, font, size, thickness, new int[1]),
                Imgproc.getTextSize(l2, font, size, thickness, new int[1])
        };

        Point[] points = new Point[] {
                new Point((orig.cols()-boxes[0].width)/2, top_margin + boxes[0].height),
                new Point((orig.cols()-boxes[1].width)/2, top_margin + line_spacing + boxes[0].height + boxes[1].height)
        };

        Imgproc.putText(orig, l1, points[0], font, size, color, thickness);
        Imgproc.putText(orig, l2, points[1], font, size, color, thickness);
    }

    static void savePhoto (Mat mat, String fileName) {

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
