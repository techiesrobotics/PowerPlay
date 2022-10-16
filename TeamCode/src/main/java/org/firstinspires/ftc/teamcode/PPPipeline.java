package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PPPipeline extends OpenCvPipeline {
    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(320, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 75;
    public static int REGION_HEIGHT = 100;

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_green_bound = new Scalar(0, 100, 0, 255),
            upper_green_bound = new Scalar(0, 255, 0, 255),
            lower_purple_bound = new Scalar(0, 200, 200, 255),
            upper_purple_bound = new Scalar(150, 255, 255, 255),
            lower_brown_bound = new Scalar(170, 0, 170, 255),
            upper_brown_bound = new Scalar(255, 60, 255, 255);

    // Color definitions
    private final Scalar
            GREEN  = new Scalar(0, 255, 0),
            PURPLE    = new Scalar(163, 0, 163),

            BROWN = new Scalar(150, 75, 0);

    // Percent and mat definitions
    private double grePercent, purPercent, broPercent;
    private Mat greMat = new Mat(), purMat = new Mat(), broMat = new Mat(), blurredMat = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology - processing images based on shapes
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_green_bound, upper_green_bound, greMat);
        Core.inRange(blurredMat, lower_purple_bound, upper_purple_bound, purMat);
        Core.inRange(blurredMat, lower_brown_bound, upper_brown_bound, broMat);

        // Gets color specific values
        grePercent = Core.countNonZero(greMat);
        purPercent = Core.countNonZero(purMat);
        broPercent = Core.countNonZero(broMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(grePercent, Math.max(purPercent, broPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == grePercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
        } else if (maxPercent == purPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    PURPLE,
                    2
            );
        } else if (maxPercent == broPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BROWN,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        greMat.release();
        purMat.release();
        broMat.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}