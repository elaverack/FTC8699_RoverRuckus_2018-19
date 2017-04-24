package org.firstinspires.ftc.teamcode.robotHandlers;

import android.graphics.Bitmap;
import android.os.Environment;

import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robotHandlers.DebugLogger;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.util.Arrays;

/**
 * Created by Chandler on 3/6/2017.
 */

public class Vuforia2017Manager {

    private final String PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Photos";
    public final static int
            ERROR = 0,
            BLUE_RED = 1,
            RED_BLUE = 2,
            BLUE_BLUE = 3,
            RED_RED = 4;
    private final static Scalar
            BLUE_LOW = new Scalar(100, 0, 220),
            BLUE_HIGH = new Scalar(178, 255, 255);

    private int
            beacon1Config = -1,
            beacon2Config = -1;
    private final int
            BEACON_1 = 3,
            BEACON_2 = 1;

    private VuforiaTrackables beacons;
    private VuforiaLocalizer vuforia;

    private DebugLogger log;

    private final boolean DEBUG;

    public enum TRACKABLE {
        WHEELS(0), TOOLS(1), LEGO(2), GEARS(3);

        int index;

        TRACKABLE(int index) {
            this.index = index;
        }
    }

    public Vuforia2017Manager() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AfvDu9r/////AAAAGesE+mqXV0hVqVSqU52GJ10v5Scxwd9O/3bf1yzGciRlpe31PP3enyPDvcDbz7KEDxGCONmmpf7+1w7C0PJgkJLNzqxyuHE/pUZlkD37cwnxvJSozZ7I7mx1Vk4Lmw8fAeKlvBAtMCfSeBIPQ89lKkKCuXC7vIjzY66pMmrplByqaq/Ys/TzYkNp8hAwbupsSeykVODtbIbJtgmxeNnSM35zivwcV0hpc5S0oVOoRczJvVxKh5/tzMqH2oQ1fVlNwHhvSnyOGi5L2eoAHyQjsP/96H3vYniltziK13ZmHTM7ncaSC/C0Jt4jL9hHMxvNeFl2Rs7U1u4A+WYJKJ6psFBe2TLJzOwBuzM3KGfZxfkU";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, false);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        OpenCVLoader.initDebug();
        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        DEBUG = false;

        if (DEBUG) log = new DebugLogger();
        if (DEBUG) log.log("Initialized Vuforia.");

    }

    public Vuforia2017Manager(boolean doDebugging) {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AfvDu9r/////AAAAGesE+mqXV0hVqVSqU52GJ10v5Scxwd9O/3bf1yzGciRlpe31PP3enyPDvcDbz7KEDxGCONmmpf7+1w7C0PJgkJLNzqxyuHE/pUZlkD37cwnxvJSozZ7I7mx1Vk4Lmw8fAeKlvBAtMCfSeBIPQ89lKkKCuXC7vIjzY66pMmrplByqaq/Ys/TzYkNp8hAwbupsSeykVODtbIbJtgmxeNnSM35zivwcV0hpc5S0oVOoRczJvVxKh5/tzMqH2oQ1fVlNwHhvSnyOGi5L2eoAHyQjsP/96H3vYniltziK13ZmHTM7ncaSC/C0Jt4jL9hHMxvNeFl2Rs7U1u4A+WYJKJ6psFBe2TLJzOwBuzM3KGfZxfkU";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, false);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        OpenCVLoader.initDebug();
        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        DEBUG = doDebugging;

        if (DEBUG) {
            log = new DebugLogger();
            log.log("Initialized Vuforia.");
            File photo_dir = new File(PHOTO_DIRECTORY);
            if (photo_dir.isDirectory() && !photo_dir.exists()) photo_dir.mkdir();
        }
    }

    public void start() {beacons.activate(); log.log("Started Vuforia.");}

    // beaconIndex is either 1 or 2, signifying which beacon the robot is in front of.
    public void checkOnBeacons(int beaconIndex) throws InterruptedException {

        if (beaconIndex == 1) {
            VuforiaTrackableDefaultListener BEACON = (VuforiaTrackableDefaultListener) beacons.get(BEACON_1).getListener();
            if (beacon1Config <= 0 && (BEACON != null && BEACON.getRawPose() != null)) {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_1).getListener(), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon1Config = values[0];
            } else {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon1Config = values[0];
            }
        } else if (beaconIndex == 2) {
            VuforiaTrackableDefaultListener BEACON = (VuforiaTrackableDefaultListener) beacons.get(BEACON_2).getListener();
            if (beacon2Config <= 0 && (BEACON != null && BEACON.getRawPose() != null)) {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_2).getListener(), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon2Config = values[0];
            } else {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon2Config = values[0];
            }
        }

    }

    public void checkOnBeacons() throws InterruptedException {
        if (beacon1Config <= 0 && beacons.get(BEACON_1) != null) {
            vuforia.setFrameQueueCapacity(5);
            Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_1).getListener(), vuforia.getCameraCalibration());
            int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
            beacon1Config = values[0];
        }
        if (beacon2Config <= 0 && beacons.get(BEACON_2) != null) {
            vuforia.setFrameQueueCapacity(5);
            Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_2).getListener(), vuforia.getCameraCalibration());
            int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
            beacon2Config = values[0];
        }
    }

    // Returns Bitmap's {config, width, height}.
    private int[] processImageOpenCV(Bitmap bmp, Scalar blueLow, Scalar blueHigh) {

        if (bmp == null) {
            int[] Return = {0, 0, 0};
            return Return;
        }

        int config;
        int width;
        int height;

        width = bmp.getWidth();
        height = bmp.getHeight();
        bmp = bmp.copy(Bitmap.Config.RGB_565, true);

        Mat image = new Mat(bmp.getWidth(), bmp.getHeight(), CvType.CV_8UC3);
        Utils.bitmapToMat(bmp, image);

        Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = new Mat();
        Core.inRange(image, blueLow, blueHigh, mask);
        Moments mmnts = Imgproc.moments(mask, true);

        savePhoto(mask, "process-1.png");

        if (DEBUG) log.log("" + mask.cols() + ", " + mask.rows() + ", " + mask.width() + ", " + mask.height());

        if (mmnts.get_m00() / mask.total() > 0.5) {
            config =  BLUE_BLUE;
            int[] Return = {config, width, height};
            if (DEBUG) log.log("" + (mmnts.get_m00() / mask.total()));
            return Return;
        } else if (mmnts.get_m00() / mask.total() < 0.1) {
            config =  RED_RED;
            int[] Return = {config, width, height};
            return Return;
        } else {
            if (DEBUG) log.log("" + (mmnts.get_m00() / mask.total()));
        }

        if ((mmnts.get_m10()) / (mmnts.get_m00()) <  image.cols() / 2) {
            config =  BLUE_RED;
            if (DEBUG) log.log("" + mmnts.get_m10() + ", " + mmnts.get_m00() + ", " + image.cols());
            int[] Return = {config, width, height};
            return Return;
        } else {
            config =  RED_BLUE;
            int[] Return = {config, width, height};
            return Return;
        }



    }

    // Returns String based on beacon config, e.g. "RED_BLUE".
    public static String decodeBeaconConfig(int config) {
        switch (config) {
            case BLUE_RED:
                return "Blue, Red";
            case RED_BLUE:
                return "Red, Blue";
            case BLUE_BLUE:
                return "Blue, Blue";
            case RED_RED:
                return "Red, Red";
            default:
                return "ERROR";
        }
    }

    private Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat) {

        long numImgs = frame.getNumImages();

        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == pixelFormat) {
                return frame.getImage(i);
            } else {
                if (DEBUG) log.log("Not the image I'm looking for. Format: " + frame.getImage(i).getFormat());
            }
        }

        return null;

    }

    private Bitmap getBeacon(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        OpenGLMatrix pose = beacon.getRawPose();

        if (pose == null) {
            if (DEBUG) log.log("Ahh! Pose is null!");
            return null;
        }

        if (img == null) {
            if (DEBUG) log.log("Ahh! Image is null!");
            return null;
        } else if (img.getPixels() == null) {
            if (DEBUG) log.log("Ahh! Image pixels are null!");
            return null;
        } else {
            if (DEBUG) {
                log.log("Saving 1.png...");
                savePhoto(img, "get1-1.png");
                log.log("Done. Converting to bitmap...");
            }

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 92, 0)).getData(); //bottom right
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 92, 0)).getData(); //bottom left

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            if (DEBUG) {
                log.log("Done. Saving 2.png...");
                savePhoto(bm, "get1-2.png");
                log.log("Done. Converting to mat and cropping...");
            }

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));
            Core.flip(cropped.t(), cropped, 1);

            if (DEBUG) log.log("Done. Converting back to bitmap and returning...");
            Bitmap Return = Bitmap.createBitmap(cropped.width(), cropped.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(cropped, Return);

            if (DEBUG) {
                savePhoto(Return, "get1-3.png");
                log.log("Done getting beacon.");
            }
            return Return;

        }

    }

    private Bitmap getBeacon(Image img, CameraCalibration camCal) {

        if (img == null) {
            if (DEBUG) log.log("Ahh! Image is null!");
            return null;
        } else if (img.getPixels() == null) {
            if (DEBUG) log.log("Ahh! Image pixels are null!");
            return null;
        } else {

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            if (DEBUG) {
                log.log("Done. Saving 1.png...");
                savePhoto(bm, "get2-1.png");
                log.log("Done. Converting to mat and rotating...");
            }

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);
            Core.flip(crop.t(), crop, 1);

            savePhoto(crop, "get2-2.png");
            if (DEBUG) log.log("Done. Trying to cut in half...");

            // Cut in half.
            Mat cropped = new Mat(crop, new Rect(0, (crop.rows() / 2), crop.cols(), (crop.rows() / 2)));
            savePhoto(cropped, "get2-3.png");
            if (DEBUG) log.log("Done. Converting back to bitmap and returning...");

            Bitmap Return = Bitmap.createBitmap(cropped.width(), cropped.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(cropped, Return);

            savePhoto(Return, "get2-4.png");

            if (DEBUG) log.log("Done getting beacon without cropping.");
            return Return;

        }

    }

    private void savePhoto (Bitmap bmp, String fileName) {
        if (!DEBUG) return;



        File file = new File(PHOTO_DIRECTORY, fileName);

        if (file.exists()) {
            String newFileName = fileName.split(".png")[0];
            int i = 1;
            file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png");
            while (file.exists()) {
                i++;
                file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png");
            }
        }

        ByteArrayOutputStream bos = new ByteArrayOutputStream();
        bmp.compress(Bitmap.CompressFormat.PNG, 0, bos);
        byte[] bitmapdata = bos.toByteArray();

        try {
            FileOutputStream f = new FileOutputStream(file);
            f.write(bitmapdata);
            f.flush();
            f.close();
        } catch (Exception e) {
            // meh
        }
    }

    private void savePhoto (Mat mat, String fileName) {
        if (!DEBUG) return;

        Bitmap bmp = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat, bmp);

        File file = new File(PHOTO_DIRECTORY, fileName);

        if (file.exists()) {
            String newFileName = fileName.split(".png")[0];
            int i = 1;
            file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png");
            while (file.exists()) {
                i++;
                file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png");
            }
        }

        if (bmp != null) {

            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            bmp.compress(Bitmap.CompressFormat.PNG, 0, bos);
            byte[] bitmapdata = bos.toByteArray();

            try {
                FileOutputStream f = new FileOutputStream(file);
                f.write(bitmapdata);
                f.flush();
                f.close();
            } catch (Exception e) {
                // meh
            }
        }

    }

    private void savePhoto (Image img, String fileName) {
        if (!DEBUG) return;

        if (img == null) return;

        Bitmap bmp = null;
        try {
            bmp.copyPixelsFromBuffer(img.getPixels());
        } catch (Exception e) {
            //meh
        }

        File file = new File(PHOTO_DIRECTORY, fileName);

        if (file.exists()) {
            String newFileName = fileName.split(".png")[0];
            int i = 1;
            file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png");
            while (file.exists()) {
                i++;
                file = new File(PHOTO_DIRECTORY, newFileName + " (" + i + ").png");
            }
        }

        if (bmp != null) {

            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            bmp.compress(Bitmap.CompressFormat.PNG, 0, bos);
            byte[] bitmapdata = bos.toByteArray();

            try {
                FileOutputStream f = new FileOutputStream(file);
                f.write(bitmapdata);
                f.flush();
                f.close();
            } catch (Exception e) {
                // meh
            }
        }

    }

    // 1 for gears beacon, 2 for tools beacon
    public int getBeaconConfig(int index) {
        if (index != 1 && index != 2) return -1;
        if (index == 1) return beacon1Config;
        return beacon2Config;
    }

    public void closeLog() {if (DEBUG) log.close_log();}

}
