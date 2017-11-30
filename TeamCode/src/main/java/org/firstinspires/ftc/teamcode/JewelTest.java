package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.opencv.core.Scalar;

// Created on 11/4/2017 at 9:56 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "JewelTest", group = "Linear Opmode")
//@Disabled
public class JewelTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler vh;
    private Scalar
            high = new Scalar(0,0,0),
            low = new Scalar(0,0,0); // Note: Goes Blue, Green, Red (for some stupid reason)

    @Override
    public void runOpMode() throws InterruptedException {

        vh = new VisualsHandler(this);

        double[][] RGBLH = new double[][]{new double[]{0,0,0}, new double[]{0,0,0}}; // Note: Bounds are [0,255]
        int step = 1; // Note: Bounds are [1, 5000]. Divide by 100 when
        int
                lowhigh = 0, //Note: Can either be 0 or 1 for low or high
                selection = 3, // Note: Can either be 0, 1, 2, or 3 for red, green, blue or all
                stepping = 1; // Note: Is for changing step, can be 1, 2, 3, 4

        boolean
                xp = false,
                yp = false,
                ap = false,
                bp = false,
                dup = false,
                ddp = false,
                drp = false,
                dlp = false,
                rbp = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            /*
             * Control Set:
             * X        - toggle between changing all, red, blue, and green
             * A        - increase step
             * B        - decrease step
             * D_UP     - step selection
             * D_DOWN   - downstep selection
             * D_LEFT   - increase step change
             * D_RIGHT  - decrease step change
             * Y        - preview
             * R_BUMP   - switch between high and low
             */

            if (gamepad1.x && !xp) {
                if (selection == 3) {
                    selection = 0;
                } else selection++;
                xp = true;
            } else if (!gamepad1.x && xp) xp = false;

            if (gamepad1.a && !ap) {
//                int n=((int)Math.floor(step/Math.pow(10,(stepping-3))))%10;
//                step -= ((double)n)*Math.pow(10,(stepping-3));
//                if (n==9) {
//                    n = 0;
//                } else n++;
//                step += ((double)n)*Math.pow(10,(stepping-3));
//                step = Math.floor(step*100)/100;
                int p = (int)Math.pow(10,stepping-1);
                int n = (step/p)%10;
                step-=n*p;
                if (n!=9) step+=(n+1)*p;
                ap = true;
            } else if (!gamepad1.a && ap) ap = false;

            if (gamepad1.b && !bp) {
//                int n=((int)Math.floor(step/Math.pow(10,(stepping-3))))%10;
//                step -= ((double)n)*Math.pow(10,(stepping-3));
//                if (n==0) {
//                    n = 9;
//                } else n--;
//                step += ((double)n)*Math.pow(10,(stepping-3));
//                step = Math.floor(step*100)/100;
                int p = (int)Math.pow(10,stepping-1);
                int n = (step/p)%10;
                step-=n*p;
                if (n>1) step+=(n-1)*p;
                if (n==0) step+=9*p;
                bp = true;
            } else if (!gamepad1.b && bp) bp = false;

            if (gamepad1.dpad_up && !dup) {
                try {
                    double sel = RGBLH[lowhigh][selection];
                    sel += ((double)step)/100;
                    if (sel > 255) sel = 255;
                    RGBLH[lowhigh][selection] = sel;
                } catch (ArrayIndexOutOfBoundsException e) {
                    double[] rgb = RGBLH[lowhigh];
                    for (int i = 0; i < rgb.length; i++) {
                        double sel = rgb[i];
                        sel += ((double)step)/100;
                        if (sel > 255) sel = 255;
                        rgb[i] = sel;
                    }
                    RGBLH[lowhigh] = rgb;
                }
                dup = true;
            } else if (!gamepad1.dpad_up && dup) dup = false;

            if (gamepad1.dpad_down && !ddp) {
                try {
                    double sel = RGBLH[lowhigh][selection];
                    sel -= ((double)step)/100;
                    if (sel < 0) sel = 0;
                    RGBLH[lowhigh][selection] = sel;
                } catch (ArrayIndexOutOfBoundsException e) {
                    double[] rgb = RGBLH[lowhigh];
                    for (int i = 0; i < rgb.length; i++) {
                        double sel = rgb[i];
                        sel -= ((double)step)/100;
                        if (sel < 0) sel = 0;
                        rgb[i] = sel;
                    }
                    RGBLH[lowhigh] = rgb;
                }
                ddp = true;
            } else if (!gamepad1.dpad_down && ddp) ddp = false;

            if (gamepad1.dpad_left && !dlp) {
                if (stepping == 4) {
                    stepping = 1;
                } else stepping++;
                dlp = true;
            } else if (!gamepad1.dpad_left && dlp) dlp = false;

            if (gamepad1.dpad_right && !drp) {
                if (stepping == 1) {
                    stepping = 4;
                } else stepping--;
                drp = true;
            } else if (!gamepad1.dpad_right && drp) drp = false;

            if (gamepad1.y && !yp) {
                vh.previewMask(vh.takeMatPicture(),
                        new Scalar(RGBLH[1][2],RGBLH[1][1],RGBLH[1][0]),
                        new Scalar(RGBLH[0][2],RGBLH[0][1],RGBLH[0][0]));
                yp = true;
            } else if (!gamepad1.y && yp) yp = false;

            if (gamepad1.right_bumper && !rbp) {
                if (lowhigh == 1) {
                    lowhigh = 0;
                } else lowhigh++;
                rbp = true;
            } else if (!gamepad1.right_bumper && rbp) rbp = false;

            telemetry.addData("HL", decodeHL(lowhigh));
            telemetry.addData("LRGB", RGBLH[1][0]+", "+RGBLH[1][1]+", "+RGBLH[1][2]);
            telemetry.addData("HRGB", RGBLH[0][0]+", "+RGBLH[0][1]+", "+RGBLH[0][2]);
            telemetry.addData("Selection", decodeSel(selection));
            telemetry.addData("Step", "S " + step + ", " + decodeStepping(stepping));
            telemetry.update();
        }

    }
    private String decodeHL(int hl) {
        if (hl == 1) return "HIGH";
        return "LOW";
    }

    private String decodeStepping(int stping) {
        switch (stping) {
            case 1:
                return "Hundredths";
            case 2:
                return "Tenths";
            case 3:
                return "Ones";
            case 4:
                return "Tens";
            default:
                return "ERROR";
        }
    }

    private String decodeSel(int sel) {
        switch (sel) {
            case 0:
                return "RED";
            case 1:
                return "GREEN";
            case 2:
                return "BLUE";
            case 3:
                return "ALL";
            default:
                return "ERROR";
        }
    }
}
