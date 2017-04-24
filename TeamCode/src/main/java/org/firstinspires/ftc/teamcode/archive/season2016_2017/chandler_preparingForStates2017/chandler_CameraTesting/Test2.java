/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_CameraTesting;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;

// Created on 2/6/2017 at 6:21 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.chandler_VuforiaTesting

@TeleOp(name = "VU-Test2", group = "VU")  // @Autonomous(...) is the other common choice
@Disabled
public class Test2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public final static Scalar blueLow = new Scalar(100, 0, 220);
    public final static Scalar blueHigh = new Scalar(178, 255, 255);

    public final static int ERROR = 0;
    public final static int BLUE_RED = 1;
    public final static int RED_BLUE = 2;
    public final static int BLUE_BLUE = 3;
    public final static int RED_RED = 4;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AfvDu9r/////AAAAGesE+mqXV0hVqVSqU52GJ10v5Scxwd9O/3bf1yzGciRlpe31PP3enyPDvcDbz7KEDxGCONmmpf7+1w7C0PJgkJLNzqxyuHE/pUZlkD37cwnxvJSozZ7I7mx1Vk4Lmw8fAeKlvBAtMCfSeBIPQ89lKkKCuXC7vIjzY66pMmrplByqaq/Ys/TzYkNp8hAwbupsSeykVODtbIbJtgmxeNnSM35zivwcV0hpc5S0oVOoRczJvVxKh5/tzMqH2oQ1fVlNwHhvSnyOGi5L2eoAHyQjsP/96H3vYniltziK13ZmHTM7ncaSC/C0Jt4jL9hHMxvNeFl2Rs7U1u4A+WYJKJ6psFBe2TLJzOwBuzM3KGfZxfkU";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        OpenCVLoader.initDebug();

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int config1;
        int config2;

        beacons.activate();

        telemetry.clearAll();
        vuforia.setFrameQueueCapacity(5);

        if (beacons.get(3) != null) {

            telemetry.addData("Status", "Gears beacon is a go. Processing...");
            telemetry.update();

            try {
                config1 = getBeaconConfig(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(3).getListener(), vuforia.getCameraCalibration());
            } catch (InterruptedException e) {
                config1 = 0;
            }

        } else {
            config1 = 0;
        }

        if (beacons.get(1) != null) {

            telemetry.addData("Status", "Tools beacon is a go. Processing...");
            telemetry.update();

            try {
                config2 = getBeaconConfig(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(1).getListener(), vuforia.getCameraCalibration());
            } catch (InterruptedException e) {
                config2 = 0;
            }

        } else {
            config2 = 0;
        }

        float processTime = (float) runtime.seconds();
        telemetry.addData("Status", "Done. Process time: " + processTime + " seconds.");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Gears Beacon", decodeBeaconConfig(config1));
            telemetry.addData("Tools Beacon", decodeBeaconConfig(config2));

            telemetry.update();

        }
    }

    public String decodeBeaconConfig(int config) {
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

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat) {

        long numImgs = frame.getNumImages();

        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == pixelFormat) {
                return frame.getImage(i);
            }
        }

        return null;

    }

    public int getBeaconConfig (Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        telemetry.clearAll();
        telemetry.addData("Status", "Getting Raw Pose...");
        telemetry.update();

        OpenGLMatrix pose = beacon.getRawPose();

        telemetry.addData("Status", "Got Raw Pose, beginning analysis...");
        telemetry.update();

        if (pose != null && img != null && img.getPixels() != null) {

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 92, 0)).getData(); //bottom right
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 92, 0)).getData(); //bottom left

            telemetry.addData("Status", "Created corners, creating bitmap...");
            telemetry.update();

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            telemetry.addData("Status", "Cropping bitmap...");
            telemetry.update();

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            telemetry.addData("Status", "Processing cordinates...");
            telemetry.update();

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            telemetry.addData("Status", "Adapting colors...");
            telemetry.update();

            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            telemetry.addData("Status", "Masking...");
            telemetry.update();

            Mat mask = new Mat();
            Core.inRange(cropped, blueLow, blueHigh, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            telemetry.addData("Status", "Processing mask...");
            telemetry.update();

            if (mmnts.get_m00() / mask.total() > 0.8) {
                return BLUE_BLUE;
            } else if (mmnts.get_m00() / mask.total() < 0.1) {
                return RED_RED;
            }

            if ((mmnts.get_m01()) / (mmnts.get_m00()) <  cropped.rows() / 2) {
                return RED_BLUE;
            } else {
                return BLUE_RED;
            }

        }

        return ERROR;

    }
}
