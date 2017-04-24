package org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions;


import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.ftccommon.DbgLog;

public class ImageAnalyst {
    private Bitmap bmp;


    // todo - return value of Integer.MAX_VALUE if no color found at all.


    public ImageAnalyst(Bitmap bmp) {
        if (bmp == null) {
            throw new NullPointerException("bitmap is null");
        } else {
            this.bmp = bmp;
        }


    }

    public Bitmap getBitmap() {
        return bmp;
    }

    /**
     * takes distribution of a color and calculates location
     * For example: array [0,0,0,1,1,2,2,3,4,10,20,22,34,45] represents a high concentration of
     * color on the extreme right.
     * <p/>
     * This method takes the middle of the distribution and returns how much to the right or left
     * of the middle the blob is (negative for left, positive for right). It is not weighted as to
     * the values - maybe later.
     * <p/>
     * todo - if no red found, it returns center, which is wrong. Need a way to show no red found
     */

    public int analyzeDistribution(int[] distribution) {
        // todo -- should normalize or not?
        normalize(distribution);

        // find first index of red
        int startIndex = 0;
        for (int i = 0; i < distribution.length; i++) {
            if (distribution[i] > 0) {
                startIndex = i;
                break;
            }
        }

        if (startIndex == distribution.length - 1) {
            return Integer.MAX_VALUE; // no color found
        } else {
            // iterate backwards to find lastIndex of red
            int endIndex = distribution.length - 1;
            for (int i = distribution.length - 1; i >= startIndex; i--) {
                if (distribution[i] > 0) {
                    endIndex = i;
                    break;
                }
            }

            DbgLog.msg("start/end = " + startIndex + " / " + endIndex);
            // average start and end indexes
            int blobMiddle = (startIndex + endIndex) / 2;

            // get how much left or right the blob is
            // negative will be left, positive will be right
            int dirAmt = blobMiddle - bmp.getWidth() / 2;

            DbgLog.msg("blobMiddle = " + blobMiddle);
            DbgLog.msg("bmp middle =  " + bmp.getWidth() / 2);

            if ((dirAmt == 0 || dirAmt == 1 || dirAmt == -1) && blobMiddle - bmp.getWidth() < 2 && (endIndex - startIndex) < 200) {
                return 1;
            } else {
                DbgLog.msg("" + ((dirAmt == 0 || dirAmt == 1 || dirAmt == -1)) + ", " + (blobMiddle - bmp.getWidth() < 2) + ", " + (!((endIndex - startIndex)/2 - bmp.getWidth() < 3 && (endIndex - startIndex)/2 - bmp.getWidth() > -3)));
            }

            return scaleForImageWidth(dirAmt, bmp.getWidth());
        }

    }

    private int scaleForImageWidth(int dirAmt, int width) {
        return (dirAmt * 100) / (width / 2);
    }

    private void normalize(int[] distribution) {
        int average = 0;
        for (int i = 0; i < distribution.length; i++) {
            average += distribution[i];
        }
        average = average / distribution.length;
        //DbgLog.msg("average = " + average);
        // normalize distribution by subtracting average (but not allowing negative numbers. If
        // subtracting average leads to a negative number, then use 0.
        for (int i = 0; i < distribution.length; i++) {
            int val = distribution[i] - average;
            // DbgLog.msg("changing from " + distribution[i] + " to " + val);
            distribution[i] = val < 0 ? 0 : val;
        }
        //DbgLog.msg("Normalized Distribution: ");
        //DbgLog.msg(Arrays.toString(distribution));
    }

    /* creates array of size=bitmap width. Holds how many pixels in that vertical line are red or
    blue. So distribution[0] will equal how many pixels are red/blue in the first vertical line of
    the image.
     */
    public int[] getVerticalDistribution(char rb, int tolerance) throws IllegalArgumentException {
        int[] distribution = new int[bmp.getWidth()];
        for (int i = 0; i < bmp.getHeight(); i++) {
            for (int j = 0; j < bmp.getWidth(); j++) {
                int b = bmp.getPixel(j, i);
                if (rb == 'R') {
                    if (isRed(b, tolerance)) {
                        distribution[j]++;
                    }
                } else if (rb == 'B') {
                    if (isBlue(b, tolerance)) {
                        distribution[j]++;
                    }
                } else {
                    throw new IllegalArgumentException(rb + " is not a valid argument - use R for red or B for blue.");
                }
            }
        }

        //DbgLog.msg(Arrays.toString(distribution));
        return distribution;
    }

    // convenience methods
    public int superSimpleRedDetection(int tolerance) {
        return superSimpleColorDetection('R', tolerance);
    }


    public int superSimpleBlueDetection(int tolerance) {
        return superSimpleColorDetection('B', tolerance);
    }

    protected int superSimpleColorDetection(char rb, int tolerance) throws IllegalArgumentException {
        int left = 0;
        int right = 0;

        for (int i = 0; i < bmp.getHeight(); i++) {
            for (int j = 0; j < bmp.getWidth() / 2; j++) {
                int b = bmp.getPixel(j, i);
                if (isRed(b, tolerance)) {
                    left++;
                }
            }
        }

        for (int i = 0; i < bmp.getHeight(); i++) {
            for (int j = bmp.getWidth() / 2; j < bmp.getWidth(); j++) {
                int b = bmp.getPixel(j, i);
                if (rb == 'R') {
                    if (isRed(b, tolerance)) {
                        right++;
                    }
                } else if (rb == 'B') {
                    if (isBlue(b, tolerance)) {
                        right++;
                    }
                } else {
                    throw new IllegalArgumentException(rb + " is not a valid argument - use R for red or B for blue.");
                }
            }
        }
        DbgLog.msg("right: " + right);
        DbgLog.msg("left: " + left);
        return right - left;
    }

    public static boolean isRed(int b, int tolerance) {
        int blue = Color.blue(b);
        int red = Color.red(b);
        int green = Color.green(b);
        int blueGreenAvg = (blue + green) / 2;
        if ((red - blueGreenAvg) > tolerance) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean isBlue(int b, int tolerance) {
        int blue = Color.blue(b);
        int red = Color.red(b);
        int green = Color.green(b);
        int redGreenAvg = (red + green) / 2;
        if ((blue - redGreenAvg) > tolerance) {
            return true;
        } else {
            return false;
        }
    }

    public int getDominantColor() {
        if (null == bmp) return Color.TRANSPARENT;

        int redBucket = 0;
        int greenBucket = 0;
        int blueBucket = 0;
        int alphaBucket = 0;

        boolean hasAlpha = bmp.hasAlpha();
        int pixelCount = bmp.getWidth() * bmp.getHeight();
        int[] pixels = new int[pixelCount];
        bmp.getPixels(pixels, 0, bmp.getWidth(), 0, 0, bmp.getWidth(), bmp.getHeight());

        for (int y = 0, h = bmp.getHeight(); y < h; y++) {
            for (int x = 0, w = bmp.getWidth(); x < w; x++) {
                int color = pixels[x + y * w]; // x + y * width
                redBucket += (color >> 16) & 0xFF; // Color.red
                greenBucket += (color >> 8) & 0xFF; // Color.greed
                blueBucket += (color & 0xFF); // Color.blue
                if (hasAlpha) alphaBucket += (color >>> 24); // Color.alpha

            }


        }

        return Color.argb(
                (hasAlpha) ? (alphaBucket / pixelCount) : 255,
                redBucket / pixelCount,
                greenBucket / pixelCount,
                blueBucket / pixelCount);
    }

    public static int getDominantColor(Bitmap bmp) {
        if (null == bmp) return Color.TRANSPARENT;

        int redBucket = 0;
        int greenBucket = 0;
        int blueBucket = 0;
        int alphaBucket = 0;

        boolean hasAlpha = bmp.hasAlpha();
        int pixelCount = bmp.getWidth() * bmp.getHeight();
        int[] pixels = new int[pixelCount];
        bmp.getPixels(pixels, 0, bmp.getWidth(), 0, 0, bmp.getWidth(), bmp.getHeight());

        for (int y = 0, h = bmp.getHeight(); y < h; y++) {
            for (int x = 0, w = bmp.getWidth(); x < w; x++) {
                int color = pixels[x + y * w]; // x + y * width
                redBucket += (color >> 16) & 0xFF; // Color.red
                greenBucket += (color >> 8) & 0xFF; // Color.greed
                blueBucket += (color & 0xFF); // Color.blue
                if (hasAlpha) alphaBucket += (color >>> 24); // Color.alpha

            }


        }

        return Color.argb(
                (hasAlpha) ? (alphaBucket / pixelCount) : 255,
                redBucket / pixelCount,
                greenBucket / pixelCount,
                blueBucket / pixelCount);
    }

    }

