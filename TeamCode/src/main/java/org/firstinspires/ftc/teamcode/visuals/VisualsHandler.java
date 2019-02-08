package org.firstinspires.ftc.teamcode.visuals;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.os.Environment;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.firstinspires.ftc.teamcode.Enums.SampleArrangement;

public class VisualsHandler {
    
    private static final String PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM).toString();
    
    private static VuforiaHandler vuforia;
    private static LayoutInterfacer layout;
    
    // TODO: Replace temp setup
    public static ImageView tempSetup (OpMode om) {
        //hasLight = opmode.hardwareMap.appContext.getPackageManager().hasSystemFeature(PackageManager.FEATURE_CAMERA_FLASH);
        //togglePhoneLight();
        vuforia = new VuforiaHandler(om, false);
        OpenCVLoader.initDebug();
        // Initialize preview
        ImageView iv = new ImageView(om.hardwareMap.appContext);
        int ID = 314159;
        iv.setId(ID);
        layout = new LayoutInterfacer(om, (LinearLayout)((Activity)om.hardwareMap.appContext).findViewById(R.id.monitorContainer), iv);
        //if (!doVuforiaPreview) showIntro();
        return iv;
    }
    public static void tempSetupStop () {
        layout.close(); phoneLightOff();
    }
    
    public static void phoneLightOn () { CameraDevice.getInstance().setFlashTorchMode(true); }
    public static void phoneLightOff () { CameraDevice.getInstance().setFlashTorchMode(false); }
    
    public static Mat takeMatPicture() { return bitmapToMat(rotateBitmap(vuforia.takePicture())); }
    public static void setPreview(final Bitmap image) { if (image == null) return; layout.run(new Runnable() {
        @Override
        public void run() { ((ImageView)layout.getView(314159)).setImageBitmap(image); }
    }); }
    public static Mat takeAndPreviewPicture () {
        Bitmap b = rotateBitmap(vuforia.takePicture());
        setPreview(b);
        return bitmapToMat(b);
    }
    
    public static SampleArrangement takePreviewAndSample () {
        Bitmap b = vuforia.takePicture();
        setPreview(b);
        
        return sample(bitmapToMat(b));
    }
    public static Sampler offloadPreviewandSample () {
        return new Sampler();
    }
    private static SampleArrangement sample (Mat start) {
        
        Mat silverMask = new Mat(), goldMask = new Mat();
        
        Imgproc.cvtColor(start, start, Imgproc.COLOR_RGB2BGR);
        
        double spacingThres = 50;
        
        Core.inRange(start, SampleArrangement.silverLow, SampleArrangement.silverHigh, silverMask); // mask for silver mineral
        Core.inRange(start, SampleArrangement.goldLow, SampleArrangement.goldHigh, goldMask); // mask for gold mineral
        
        Imgproc.morphologyEx(silverMask, silverMask, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5, 5))); // clear out noise of silver mask
        Imgproc.morphologyEx(silverMask, silverMask, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(10, 10))); // close holes in silver mask
        Imgproc.morphologyEx(goldMask, goldMask, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(4, 4))); // clear out noise of gold mask
        Imgproc.morphologyEx(goldMask, goldMask, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(10, 10))); // close holes in gold mask
        
        //Windows.showImg("start", start.clone());
        //Windows.showImg("silver mask", silverMask.clone());
        //Windows.showImg("gold mask", goldMask.clone());
        
        MatOfPoint[] silCons = SampleArrangement.circularContours(silverMask, .6, 1).toArray(new MatOfPoint[0]); // get all the roughly circular contours in silver mask
        MatOfPoint[] golCons = SampleArrangement.rectangularContours(goldMask, .5, 1).toArray(new MatOfPoint[0]); // get all the roughly rectangular contours in gold mask
        
        Point[] silCens = new Point[silCons.length]; // array for storing the silver contour centers
        int[]   silRadi = new int[silCons.length]; // array for storing the silver contour radii
        goldMask.setTo(new Scalar(0)); // clear gold mask so we can redraw only the rectangular contours
        
        for (int i = 0; i < silCons.length; i++) { // get all the centers and radii of the circular contours in silver mask
            Point center = new Point();
            float[] radius = new float[1];
            
            Imgproc.minEnclosingCircle(new MatOfPoint2f(silCons[i].toArray()), center, radius);
            
            silCens[i] = center;
            silRadi[i] = (int)Math.ceil(radius[0]);
            
            Imgproc.circle(start, silCens[i], silRadi[i], new Scalar(0,0,255), -1);
        }
        
        for (int i = 0; i < golCons.length; i++) // redraw all the rectangular contours in gold mask
            Imgproc.drawContours(goldMask, Collections.singletonList(golCons[i]), 0, new Scalar(255), -1);
        
        //Windows.showImg("circles", start.clone());
        //Windows.showImg("gold", goldMask.clone());
        
        if (silCons.length > 1) for (int i = 0; i < silCons.length; i++) for (int j = i+1; j < silCons.length; j++) {
            // note: the above line is basically saying "if I have more than one circle, for each unique pair of circles, do the following
            
            int radius = silRadi[i] > silRadi[j] ? silRadi[i] : silRadi[j]; // get the larger of the two radii
            
            // START SECTION: this section is all about cropping out a section corresponding to the line of thickness 2*radius you would get if you drew a line through the entire mat through both silver minerals
            int y1 = (int) (( (silCens[j].y - silCens[i].y) / (silCens[j].x - silCens[i].x) ) * (-silCens[i].x) + silCens[i].y); // coordinates
            int y2 = (int) (( (silCens[j].y - silCens[i].y) / (silCens[j].x - silCens[i].x) ) * (start.width()-silCens[i].x-1) + silCens[i].y);
            
            Mat crop = new Mat(start.rows(), start.cols(), CvType.CV_8UC1); // destination mat
            crop.setTo(new Scalar(0));
            
            Imgproc.line(crop, new Point(0, y1), new Point(start.width()-1, y2), new Scalar(255), 2*radius); // crop line
            
            Core.subtract(goldMask, crop, crop); // cropping
            Core.subtract(goldMask, crop, crop);
            
            Imgproc.morphologyEx(goldMask, goldMask, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(4, 4))); // clear out noise
            Imgproc.morphologyEx(goldMask, goldMask, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(10, 10))); // close holes
            // END SECTION
            
            if (Imgproc.moments(crop, true).get_m00() <= 0) continue; // if there is no gold in cropped mat, skip and go to next pair
            
            List<MatOfPoint> contours = new ArrayList<>(); // list for storing contours from cropped mat
            Imgproc.findContours(crop, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); // find contours in cropped mat
            
            Moments goldmn = null; Point goldmnp = null;
            for (MatOfPoint m : contours) {
                Moments mn = Imgproc.moments(m); // get moments of contour
                Point mnp = new Point((int)(mn.get_m10() / mn.get_m00()), (int)(mn.get_m01() / mn.get_m00())); // get center of contour
                
                Point[] ps = { silCens[i], silCens[j], mnp }; // array of points with gold center and both silver centers
                Arrays.sort(ps, new Comparator<Point>() { // sort them in x-coordinate order (lesser first)
                    @Override
                    public int compare (Point point, Point t1) {
                        return (int)Math.round(t1.x - point.x);
                    }
                });
                
                if (Math.abs( // check the difference in the distance between the left-most and center and center and right-most
                              Math.sqrt(Math.pow(ps[2].x - ps[1].x, 2) + Math.pow(ps[2].y - ps[1].y, 2)) -
                                      Math.sqrt(Math.pow(ps[1].x - ps[0].x, 2) + Math.pow(ps[1].y - ps[0].y, 2))
                ) > spacingThres) continue;
                
                if (goldmn == null || mn.get_m00() > goldmn.get_m00()) { // get contour in cropped mat with largest area
                    goldmn = mn;
                    goldmnp = mnp;
                }
            }
            
            if (goldmn == null) continue;
            
            return SampleArrangement.fromInts((int)goldmnp.x, (int)silCens[i].x, (int)silCens[j].x);
            
        }
        
        return SampleArrangement.ERROR;
        
    }
    
    private static Bitmap rotateBitmap(Bitmap orig) { // rotates by 90 deg counterclock
        if (orig == null) return null;
        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        return Bitmap.createBitmap(orig,0,0,orig.getWidth(),orig.getHeight(),matrix,true);
    }
    private static Mat bitmapToMat(Bitmap orig) {
        if (orig == null) return null;
        Mat image = new Mat(orig.getHeight(), orig.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(orig, image);
        return image;
    }
    public static Bitmap matToBitmap(Mat orig, Bitmap.Config type) {
        Bitmap bmp = Bitmap.createBitmap(orig.width(), orig.height(), type);
        Utils.matToBitmap(orig, bmp);
        return bmp;
    }
    
    public static void savePhoto (Mat mat, String fileName) {
        
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
    
    public static class Sampler implements Runnable {
        private volatile SampleArrangement sa;
    
        @Override
        public void run () {
            sa = VisualsHandler.takePreviewAndSample();
        }
        
        public SampleArrangement getValue() {
            return sa;
        }
    }

}
