package org.firstinspires.ftc.teamcode.visuals;

// Created on 2/10/2018 at 1:02 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.visuals

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class AlignmentCircle {

    private static int thickness = 10;

    public Point center;
    public int radius;

    public AlignmentCircle(Point center, int radius) { this.center = center; this.radius = radius; }

    public void draw(Mat mat, Scalar color) { Imgproc.circle(mat, center, radius, color, thickness); }

}
