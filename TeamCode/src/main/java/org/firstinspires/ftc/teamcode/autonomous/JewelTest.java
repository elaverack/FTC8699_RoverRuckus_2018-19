package org.firstinspires.ftc.teamcode.autonomous;

import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.visuals.VisualsHandler;

// Created on 11/15/2017 at 7:47 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "Jewel Test", group = "test")
//@Disabled
public class JewelTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler vh;

    private static final int BUTTON_ID = 3141592;
    private static final int IMG2_ID = 31415926;

    @Override
    public void runOpMode() throws InterruptedException {

        vh = new VisualsHandler(this);
        boolean
                xp = false;

        Button button = new Button(hardwareMap.appContext);
        button.setText("Check");
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) { try { vh.previewJewels(); } catch (InterruptedException e) {/*meh*/} }
        });
        button.setId(BUTTON_ID);
        button.setLayoutParams(new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));
        vh.layout.createViews(button);
//        ((Activity)hardwareMap.appContext).runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                LinearLayout layout = vh.getPreviewContainer();
//                ImageView iv = new ImageView(hardwareMap.appContext);
//                iv.setId(IMG2_ID);
//                layout.addView(iv);
//            }
//        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {

            // This is the time during which I can press the button and test the
            //  camera. Nothing needs to be done by the opmode.

        }

        vh.close();

    }

//    private void processJewels() throws InterruptedException {
//        if (!opModeIsActive()) return;
//        Mat
//                start = new Mat(),
//                red = new Mat(),
//                blue = new Mat(),
//                jewels;
//        cvtColor(vh.takeMatPicture(), start, COLOR_RGB2HSV_FULL);
//        resize(start, start, new Size(), .25,.25, INTER_LINEAR);
//
//        blur(start, start, new Size(4,4));
//
//        inRange(start, new Scalar(0,128,160), new Scalar(42.5,255,255), red);
//        inRange(start, new Scalar(127.5,128,160), new Scalar(198.3,255,255), blue);
//
//        Point
//                rCOM = getCOM(red),
//                bCOM = getCOM(blue);
//
//        telemetry.addData("Config", interpretCOMs(rCOM, bCOM));
//        telemetry.update();
//
//        // UNNECESSARY FOR FINAL BUILD
//
//        jewels = start; // Note: start is hsv, but we are going to treat jewels as RGB
//        jewels.setTo(new Scalar(0,0,0));
//        cvtColor(red, red, COLOR_GRAY2RGB);
//        cvtColor(blue, blue, COLOR_GRAY2RGB);
//        multiply(red, (new Mat(red.rows(), red.cols(), red.type())).setTo(new Scalar(255,0,0)), red);
//        multiply(blue, (new Mat(blue.rows(), blue.cols(), blue.type())).setTo(new Scalar(0,0,255)), blue);
//        add(red,blue,jewels);
//        circle(jewels, rCOM, 10, new Scalar(255,255,0), -1);
//        circle(jewels, bCOM, 10, new Scalar(0,255,255), -1);
//        vh.setPreview(jewels);
//
//    }
//
//    private String interpretCOMs(Point red, Point blue) {
//        if (red.x < blue.x) return "RED_BLUE";
//        return "BLUE_RED";
//    }
//
//    public Point getCOM(Mat binaryMat) {
//        Moments moments = moments(binaryMat, true);
//        return new Point(
//                (moments.get_m10()/moments.get_m00()),
//                (moments.get_m01()/moments.get_m00()));
//    }
}
