package org.firstinspires.ftc.teamcode.visuals;

// Created on 11/1/2017 at 7:52 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    private OpenGLMatrix pose;

    public VuforiaHandler(OpMode opmode) { op = opmode; init(true); }
    public VuforiaHandler(OpMode opmode, boolean doPreview) { op = opmode; init(doPreview); }

    private void init(boolean doPreview) {

        VuforiaLocalizer.Parameters parameters;

        if (doPreview) {
            parameters = new VuforiaLocalizer.Parameters(
                    op.hardwareMap.appContext.getResources().
                            getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName())
            );
        } else { parameters = new VuforiaLocalizer.Parameters(); }

        parameters.vuforiaLicenseKey = "AfvDu9r/////AAAAGesE+mqXV0hVqVSqU52GJ10v5Scxwd9O/3bf1yzGciRlpe31PP3enyPDvcDbz7KEDxGCONmmpf7+1w7C0PJgkJLNzqxyuHE/pUZlkD37cwnxvJSozZ7I7mx1Vk4Lmw8fAeKlvBAtMCfSeBIPQ89lKkKCuXC7vIjzY66pMmrplByqaq/Ys/TzYkNp8hAwbupsSeykVODtbIbJtgmxeNnSM35zivwcV0hpc5S0oVOoRczJvVxKh5/tzMqH2oQ1fVlNwHhvSnyOGi5L2eoAHyQjsP/96H3vYniltziK13ZmHTM7ncaSC/C0Jt4jL9hHMxvNeFl2Rs7U1u4A+WYJKJ6psFBe2TLJzOwBuzM3KGfZxfkU";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, false);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        vuforia.setFrameQueueCapacity(5);



    }

    public void start() { relicTrackables.activate(); }

    public boolean anyVisible() {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark != RelicRecoveryVuMark.UNKNOWN;
    }

    public String lookingAt() { if (!anyVisible()) return ""; return vuMark.name(); }

    public RelicRecoveryVuMark lookingAtMark() { if (!anyVisible()) return null; return vuMark; }

    public Bitmap takePicture() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frames = vuforia.getFrameQueue().take();
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

    public class PosRot {
        public Vector3 position, rotation;
        public PosRot() { position = new Vector3(); rotation = new Vector3(); }
        public PosRot(VectorF pos, Orientation rot) { position = new Vector3(pos); rotation = new Vector3(rot); }
        public PosRot(OpenGLMatrix pose) {
            position = new Vector3(pose.getTranslation());
            rotation = new Vector3(Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
        }
        public void doAverage (PosRot pr) {
            position = Vector3.average(position, pr.position);
            rotation = Vector3.average(rotation, pr.rotation);
        }
        public void teleout (OpMode opmode) {
            position.teleout(opmode, "Position");
            rotation.teleout(opmode, "Rotation");
        }
    }

}
