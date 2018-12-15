package org.firstinspires.ftc.teamcode.visuals_old;

// Created on 2/10/2018 at 1:02 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.visuals

import org.firstinspires.ftc.teamcode.Enums_old;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class AlignmentCircle {

    private static int thickness = 10;

    public Point center;
    public int radius;

    public AlignmentCircle(Point center, int radius) { this.center = center; this.radius = radius; }

    public void draw(Mat mat, Scalar color) { Imgproc.circle(mat, center, radius, color, thickness); }
    public void draw(Mat mat, Scalar color, double scale) {
        Imgproc.circle(mat, new Point(center.x*scale, center.y*scale), (int)(radius*scale), color, thickness);
    }

    public void isolate(Mat mat) {
        Mat clone = mat.clone(), mask = new Mat(mat.rows(), mat.cols(), mat.type());
        Imgproc.circle(mask, center, radius, new Scalar(255,255,255), -1);
        Core.subtract(clone, mask, mat);
        Core.subtract(clone, mat, mat);
    }

    Mat getMask (Mat mat, Enums_old.Color color) {
        Mat clone = mat.clone();
        isolate(clone);
        Mat ret = clone.clone();
        if (color == Enums_old.Color.RED) {
            Core.inRange(clone, VisualsHandler.RED_L, VisualsHandler.RED_H, ret);
        } else if (color == Enums_old.Color.BLUE) {
            Core.inRange(clone, VisualsHandler.BLUE_L, VisualsHandler.BLUE_H, ret);
        }
        return ret;
    }

    public Enums_old.Color getJewelColor(Mat mat) {
        Mat red = new Mat(), blue = new Mat(), src = mat.clone();
        isolate(src);
        Core.inRange(src, VisualsHandler.RED_L, VisualsHandler.RED_H, red);
        Core.inRange(src, VisualsHandler.BLUE_L, VisualsHandler.BLUE_H, blue);
        Moments redm = Imgproc.moments(red, true),
                bluem = Imgproc.moments(blue, true);
        if (redm.get_m00() > bluem.get_m00()) return Enums_old.Color.RED;
        if (bluem.get_m00() > redm.get_m00()) return Enums_old.Color.BLUE;
        return Enums_old.Color.ERROR;
    }

}
