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
import android.os.Environment;

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

import java.io.File;
import java.util.Arrays;


// Created on 2/3/2017 at 9:59 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@TeleOp(name = "PhotoTest4 (VU)", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class BETA_PhotoNVu extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private final File PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
    private final int RED_TOLERANCE = 70;
    private final int BLUE_TOLERANCE = 55;
    private final int[] photoNames = {1, 2, 3, 4};
    private int[][] values = new int[4][2];


    @Override
    public void runOpMode() throws InterruptedException {
        /*File[] Photos = new File[4];
        Bitmap[] photos = new Bitmap[4];
        ImageAnalyst[] imageAnalysts = new ImageAnalyst[4];

        for (int i = 0; i < photoNames.length; i++) {
            Photos[i] = new File(PHOTO_DIRECTORY, photoNames[i] + ".jpg");
            photos[i] = BitmapFactory.decodeFile(Photos[i].getAbsolutePath());
            imageAnalysts[i] = new ImageAnalyst(photos[i]);
            values[i][0] = imageAnalysts[i].analyzeDistribution(imageAnalysts[i].getVerticalDistribution('R', RED_TOLERANCE));
            values[i][1] = imageAnalysts[i].analyzeDistribution(imageAnalysts[i].getVerticalDistribution('B', BLUE_TOLERANCE));
        }*/

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AfvDu9r/////AAAAGesE+mqXV0hVqVSqU52GJ10v5Scxwd9O/3bf1yzGciRlpe31PP3enyPDvcDbz7KEDxGCONmmpf7+1w7C0PJgkJLNzqxyuHE/pUZlkD37cwnxvJSozZ7I7mx1Vk4Lmw8fAeKlvBAtMCfSeBIPQ89lKkKCuXC7vIjzY66pMmrplByqaq/Ys/TzYkNp8hAwbupsSeykVODtbIbJtgmxeNnSM35zivwcV0hpc5S0oVOoRczJvVxKh5/tzMqH2oQ1fVlNwHhvSnyOGi5L2eoAHyQjsP/96H3vYniltziK13ZmHTM7ncaSC/C0Jt4jL9hHMxvNeFl2Rs7U1u4A+WYJKJ6psFBe2TLJzOwBuzM3KGfZxfkU";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        /*int one = 6969;
        int two = 6969;

        runtime.reset();
        File photoFile = new File(PHOTO_DIRECTORY, "1.jpg");
        if (photoFile.exists()) {
            //Bitmap photo = BitmapFactory.decodeFile(photoFile.getPath());
            FileInputStream f;
            try {
                f = new FileInputStream(photoFile);
            } catch (Exception e) {
                f = null;
            }
            Bitmap photo1 = BitmapFactory.decodeStream(f);
            try {
                f.close();
            } catch (Exception e) {

            }
            photo1 = Bitmap.createScaledBitmap(photo1, (int) photo1.getWidth()/8, (int) photo1.getHeight()/8, true);
            telemetry.addData("Status", "Decode done. Processing...");
            telemetry.update();
            ImageAnalyst imageAnalyst = new ImageAnalyst(photo1);
            one = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('R', RED_TOLERANCE));
            two = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('B', BLUE_TOLERANCE));
        }
        double processTime = runtime.seconds();

        telemetry.addData("Status", processTime);
        telemetry.update();*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        beacons.activate();

        int red;
        int blue;
        int width;
        int hieght;
        VuforiaTrackableDefaultListener gears = (VuforiaTrackableDefaultListener) beacons.get(3).getListener();

        /*if (gears != null) {

            Bitmap image = getBeaconWithVU(vuforia, gears);
            //vuforia.setFrameQueueCapacity(5);
            //Bitmap image = getBeaconWithVU(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), gears, vuforia.getCameraCalibration());
            telemetry.addData("Status", "Got beacon, processing...");
            telemetry.update();
            ImageAnalyst imageAnalyst = new ImageAnalyst(image);
            red = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('R', RED_TOLERANCE));
            blue = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('B', BLUE_TOLERANCE));
            width = image.getWidth();
            hieght = image.getHeight();


        } else {

            red = 0;
            blue = 0;
            width = 0;
            hieght = 0;

        }*/

        vuforia.setFrameQueueCapacity(5);
        int[] formats = getImageFormats(vuforia.getFrameQueue().take());
        telemetry.addData("Formats", "RGB565: " + PIXEL_FORMAT.RGB565 + ", RGB888: " + PIXEL_FORMAT.RGB888 + ", 8888: " + PIXEL_FORMAT.RGBA8888 + ", INDEX: " + PIXEL_FORMAT.INDEXED + ", GRAY: " + PIXEL_FORMAT.GRAYSCALE + ", YUV: " + PIXEL_FORMAT.YUV + ", UNKNOWN: " + PIXEL_FORMAT.UNKNOWN_FORMAT);
        for (int i = 0; i < formats.length; i++) {
            telemetry.addData("" + i, formats[i]);
        }
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Photo", "RED: " + red + ", BLUE: " + blue);
            //telemetry.addData("Photo WH", "W: " + width + ", H: " + hieght);

            telemetry.update();
        }
    }

    public Bitmap getBeaconWithVU (VuforiaLocalizer vu, VuforiaTrackableDefaultListener beacon) throws InterruptedException{

        OpenGLMatrix pose = beacon.getRawPose();
        vu.setFrameQueueCapacity(5);
        Image img = getImageFromFrame(vu.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
        if (img == null) {
            img = getImageFromFrame(vu.getFrameQueue().take(), PIXEL_FORMAT.RGB888);
        }
        CameraCalibration camCal = vu.getCameraCalibration();

        if (pose != null && img != null && img.getPixels() != null) {

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

            return bm;
        }

        telemetry.addData("Debug", pose + ", " + img + ", " + img.getPixels());
        telemetry.update();
        return null;

    }

    public Bitmap getBeaconWithVU (Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) throws InterruptedException{

        OpenGLMatrix pose = beacon.getRawPose();

        if (pose != null && img != null && img.getPixels() != null) {

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

            return bm;
        }

        telemetry.addData("Debug", pose + ", " + img + ", " + img.getPixels());
        telemetry.update();
        return null;

    }

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat) {

        long numImgs = frame.getNumImages();

        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i) != null && frame.getImage(i).getFormat() == pixelFormat) {
                return frame.getImage(i);
            }
        }

        return null;

    }

    public int[] getImageFormats(VuforiaLocalizer.CloseableFrame frame) {

        long numImgs = frame.getNumImages();
        int[] Return = new int[(int)numImgs];

        for (int i = 0; i < numImgs; i++) {
            Return[i] = frame.getImage(i).getFormat();
        }

        return Return;

    }

}
