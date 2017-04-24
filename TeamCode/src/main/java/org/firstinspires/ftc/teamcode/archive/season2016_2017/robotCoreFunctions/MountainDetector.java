package org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions;

import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;

import com.qualcomm.ftccommon.DbgLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.all_opmodes_120916.CameraPreview;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.ByteArrayOutputStream;

public class MountainDetector implements Camera.PreviewCallback, CameraInterface {
    private android.graphics.Camera camera;
    public CameraPreview preview;
    private int width;
    private int height;
    private YuvImage yuvImage = null;
    public int redTolerance = 70;
    public int blueTolerance = 55;
    protected int orientation;

    public MountainDetector(FtcRobotControllerActivity activity, OpMode opMode, int orientation) {
        this.orientation = orientation;
        DbgLog.msg("MountainDetector > hardwareMap.appContext: " + opMode.hardwareMap.appContext);
        camera = null;//activity.camera; // todo - commented out camera for compiling
        DbgLog.msg("MountainDetector > camera: " + camera);
        //camera.setPreviewCallback(this); // todo - uncomment when camera is working again

     // todo    ((FtcRobotControllerActivity) opMode.hardwareMap.appContext).initPreview(camera, this, this);
    }

    public void onPreviewFrame(byte[] data, Camera camera) {
        Camera.Parameters parameters = camera.getParameters();
        width = parameters.getPreviewSize().width;
        height = parameters.getPreviewSize().height;
        yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
    }

    protected Bitmap convertImage() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();
        DbgLog.msg("image " + width + " x " + height);
        //DbgLog.msg(Arrays.toString(imageBytes));
        return BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }

    public Bitmap rotateBitmap(Bitmap bmp, int angle) {
        Matrix matrix = new Matrix();
        matrix.postRotate(angle);
        return Bitmap.createBitmap(bmp, 0, 0, bmp.getWidth(), bmp.getHeight(), matrix, true);
    }

    public int getRedLocation() {
        return getLocation('R', redTolerance);
    }

    public int getBlueLocation() {
        return getLocation('B', blueTolerance);
    }

    public int getLocation(char c) {
        if (c == 'R') {
            return getRedLocation();
        }
        if (c == 'B') {
            return getBlueLocation();
        } else throw new IllegalArgumentException("getLocation() accepts R or B, not " + c);
    }

    /**
     * center is 0, negative is left, positive is right
     */
    protected int getLocation(char color, int tolerance) {
        if (yuvImage != null) {
            Bitmap image = convertImage();
            if (orientation == Configuration.ORIENTATION_LANDSCAPE) {
                image = rotateBitmap(image, -90);
            }

            ImageAnalyst analyst = new ImageAnalyst(image);
            return analyst.analyzeDistribution(analyst.getVerticalDistribution(color, tolerance));
        } else {
            throw new NullPointerException("yuvImage is null!");
        }
    }

    @Override
    public void setPreview(CameraPreview preview) {
        this.preview = preview;
    }

    @Override
    public CameraPreview getPreview() {
        return preview;
    }
}
