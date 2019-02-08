package org.firstinspires.ftc.teamcode.old.visuals_old;

// Created on 1/27/2018 at 3:41 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.visuals

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Vector3 {

    public float x = 0;
    public float y = 0;
    public float z = 0;

    public Vector3 () {}
    public Vector3 (float v) { x = v; y = v; z = v; }
    public Vector3 (float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
    public Vector3 (VectorF v) { x = v.get(0); y = v.get(1); z = v.get(2); }
    public Vector3 (Orientation rot) { x = rot.firstAngle; y = rot.secondAngle; z = rot.thirdAngle; }
    public Vector3 (Vector3 v) { x = v.x; y = v.y; z = v.z; }

    public float rx() { return round(x); }
    public float ry() { return round(y); }
    public float rz() { return round(z); }

    public void mmToInches() {
        x /= 25.4f;
        y /= 25.4f;
        z /= 25.4f;
    }
    public void inchesToMm() {
        x *= 25.4f;
        y *= 25.4f;
        z *= 25.4f;
    }

    public void teleout (OpMode opmode, String caption) {
        opmode.telemetry.addData(caption, String.format("x: %1$s, y: %2$s, z: %3$s", rx(), ry(), rz() ));
    }

    public static Vector3 sum(Vector3... vectors) {
        if (vectors.length < 2) return vectors[0];
        Vector3 ret = new Vector3(vectors[0]);
        for (int i = 1; i < vectors.length; i++) {
            ret.x += vectors[i].x;
            ret.y += vectors[i].y;
            ret.z += vectors[i].z;
        }
        return ret;
    }
    public static Vector3 dif(Vector3... vectors) { // NOTE: subtracts everything else FROM the first
        if (vectors.length < 2) return vectors[0];
        Vector3 ret = new Vector3(vectors[0]);
        for (int i = 1; i < vectors.length; i++) {
            ret.x -= vectors[i].x;
            ret.y -= vectors[i].y;
            ret.z -= vectors[i].z;
        }
        return ret;
    }
    public static Vector3 mult(Vector3... vectors) {
        if (vectors.length < 2) return vectors[0];
        Vector3 ret = new Vector3(vectors[0]);
        for (int i = 1; i < vectors.length; i++) {
            ret.x *= vectors[i].x;
            ret.y *= vectors[i].y;
            ret.z *= vectors[i].z;
        }
        return ret;
    }
    public static Vector3 div(Vector3... vectors) { // NOTE: divides THE FIRST BY everything else
        if (vectors.length < 2) return vectors[0];
        Vector3 ret = new Vector3(vectors[0]);
        for (int i = 1; i < vectors.length; i++) {
            ret.x /= vectors[i].x;
            ret.y /= vectors[i].y;
            ret.z /= vectors[i].z;
        }
        return ret;
    }

    public static Vector3 average(Vector3... vectors) {
        Vector3 ret = Vector3.sum(vectors);
        return Vector3.div(ret, new Vector3(vectors.length));
    }

    public static float round(float val) { return ((float)Math.round(val*1000f))/1000f; }

}
