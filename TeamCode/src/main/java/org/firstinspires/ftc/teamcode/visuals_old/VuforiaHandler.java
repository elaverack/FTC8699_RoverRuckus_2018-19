package org.firstinspires.ftc.teamcode.visuals_old;

// Created on 11/1/2017 at 7:52 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

public class VuforiaHandler {

    private OpMode op;

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private RelicRecoveryVuMark vuMark;
    private VuforiaLocalizer.Parameters parameters;

    private OpenGLMatrix pose;

    public VuforiaHandler(OpMode opmode) { op = opmode; init(true); }
    public VuforiaHandler(OpMode opmode, boolean doPreview) { op = opmode; init(doPreview); }

    private void init(boolean doPreview) {

        if (doPreview) {
            parameters = new VuforiaLocalizer.Parameters(
                    op.hardwareMap.appContext.getResources().
                            getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName())
            );
        } else { parameters = new VuforiaLocalizer.Parameters(); }

        parameters.vuforiaLicenseKey = "AfvDu9r/////AAAAGesE+mqXV0hVqVSqU52GJ10v5Scxwd9O/3bf1yzGciRlpe31PP3enyPDvcDbz7KEDxGCONmmpf7+1w7C0PJgkJLNzqxyuHE/pUZlkD37cwnxvJSozZ7I7mx1Vk4Lmw8fAeKlvBAtMCfSeBIPQ89lKkKCuXC7vIjzY66pMmrplByqaq/Ys/TzYkNp8hAwbupsSeykVODtbIbJtgmxeNnSM35zivwcV0hpc5S0oVOoRczJvVxKh5/tzMqH2oQ1fVlNwHhvSnyOGi5L2eoAHyQjsP/96H3vYniltziK13ZmHTM7ncaSC/C0Jt4jL9hHMxvNeFl2Rs7U1u4A+WYJKJ6psFBe2TLJzOwBuzM3KGfZxfkU";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        setupVuforia();
    }

    private void setupVuforia () {
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, false);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

//        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate");

        vuforia.setFrameQueueCapacity(5);
    }

    public void start() { relicTrackables.activate(); }

    public boolean anyVisible() {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark != RelicRecoveryVuMark.UNKNOWN;
    }

    public String lookingAt() { if (!anyVisible()) return ""; return vuMark.name(); }

    public RelicRecoveryVuMark lookingAtMark() { if (!anyVisible()) return null; return vuMark; }

    public Bitmap takePicture() {
        VuforiaLocalizer.CloseableFrame frames = null;
        try {
            frames = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {}
        if (frames == null) return null;
        long numImgs = frames.getNumImages();

        for (int i = 0; i < numImgs; i++) {
            if (frames.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image img = frames.getImage(i);
                Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(img.getPixels());
                return bm;
            }
        }

        return null;
    }

    public PosRot getRelativePosition() {
        if (!anyVisible()) return new PosRot();

        pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
        if (pose == null) return new PosRot();

        return new PosRot(pose);
    }
    public PosRot getRelPosWithAverage (double seconds) {
        PosRot ret = getRelativePosition();
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < seconds) ret.doAverage(getRelativePosition());
        return ret;
    }
    public void tellDriverRelPos(PosRot pr) {
        op.telemetry.addData("pos", String.format("x: %1$s, y: %2$s, z: %3$s",
                pr.position.rx(),
                pr.position.ry(),
                pr.position.rz()
        ));
        op.telemetry.addData("rot", String.format("x: %1$s, y: %2$s, z: %3$s",
                pr.rotation.rx(),
                pr.rotation.ry(),
                pr.rotation.rz()
        ));
    }
    public void tellDriverRelPos() { tellDriverRelPos(getRelativePosition()); }

    public Bitmap frontFrame = null;
    private Camera camera = null;
    private Camera.PictureCallback onPic = new Camera.PictureCallback() {

        @Override
        public void onPictureTaken(byte[] data, Camera camera) {
            frontFrame = BitmapFactory.decodeByteArray(data, 0, data.length);
//            try {
//                FileOutputStream fos = new FileOutputStream(pictureFile);
//                fos.write(data);
//                fos.close();
//            } catch (FileNotFoundException e) {
//                Log.d(TAG, "File not found: " + e.getMessage());
//            } catch (IOException e) {
//                Log.d(TAG, "Error accessing file: " + e.getMessage());
//            }
        }
    };
    public int switchToFrontCamera () {
        VisualsHandler.phoneLightOff();

        relicTrackables.deactivate();
        CameraDevice.getInstance().stop();
        CameraDevice.getInstance().deinit();
        CameraDevice.getInstance().init(CameraDevice.CAMERA_DIRECTION.CAMERA_DIRECTION_FRONT);
        CameraDevice.getInstance().start();
        CameraDevice.getInstance().stop();
        CameraDevice.getInstance().deinit();
        CameraDevice.getInstance().init();
        CameraDevice.getInstance().start();

        Vuforia.deinit();
        Vuforia.onResume();

        return CameraDevice.getInstance().getCameraDirection();
    }
    public void takeFrontPic () {
//        Intent cameraIntent = new Intent(android.provider.MediaStore.ACTION_IMAGE_CAPTURE);
//        ((Activity)op.hardwareMap.appContext).startActivityForResult(cameraIntent, CAMERA_REQUEST);
//        CameraDevice.getInstance().stop();
//        CameraDevice.getInstance().deinit();
        camera.takePicture(null, null, onPic);
    }

    public static class PosRot {
        public Vector3 position, rotation;
        public PosRot() { position = new Vector3(); rotation = new Vector3(); }
        public PosRot(VectorF pos, Orientation rot) { position = new Vector3(pos); rotation = new Vector3(rot); fix(); }
        public PosRot(OpenGLMatrix pose) {
            position = new Vector3(pose.getTranslation());
            rotation = new Vector3(Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
            fix();
        }
        public void doAverage (PosRot pr) {
            position = Vector3.average(position, pr.position);
            rotation = Vector3.average(rotation, pr.rotation);
        }
        public void toInches () { position.mmToInches(); }
        public void teleout (OpMode opmode) {
            position.teleout(opmode, "Position");
            rotation.teleout(opmode, "Rotation");
        }
        private void fix() {
            position = new Vector3(position.x, -position.z, position.y);
            rotation = new Vector3(rotation.x, rotation.z, rotation.y);
        }
    }

}
