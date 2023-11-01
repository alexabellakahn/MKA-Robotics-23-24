package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//keep
public class SleeveDetection extends OpenCvPipeline {
    
    // Color definitions
    private final Scalar YELLOW = new Scalar(255, 255, 0);
    private final Scalar CYAN = new Scalar(0, 255, 255);
    private final Scalar MAGENTA = new Scalar(255, 0, 255);

    // Determine the color for a given position
    private Scalar colorOfPosition() {
        switch (getPosition()) {
            case "CENTER":
                return CYAN;
            case "LEFT":
                return YELLOW;
            case "RIGHT":
                return MAGENTA;
            default:
                throw new IllegalStateException("Unknown position");
        }
    }

    private Telemetry telemetry = null;

    // Anchor point definitions
    
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(170, 28);
    public static int REGION_WIDTH = 70, REGION_HEIGHT = 70;
    
    private static Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);

    private static Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    private static Rect targetRect = new Rect(sleeve_pointA, sleeve_pointB);

    // Running variable storing the parking position
    private volatile String position = "LEFT";

    public SleeveDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Get the target rectangle (sub-matrix) frame, and then sum all the values
        Mat areaMat = input.submat(targetRect);

        // We sum the values, where each pixel has a brightness for each channel (with
        // 0 being the brightest). Given that, the smallest value will be the brightest
        // color represented.  We can test this by using white and black images and checking
        // the summed values?  I don't know, the white/black test suggests the opposite!
        Scalar sumColors = Core.sumElems(areaMat);

        // Get the minimum RGB value from every single channel
        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));


        telemetry.addData("sumColors 0", sumColors.val[0]);
        telemetry.addData("sumColors 1", sumColors.val[1]);
        telemetry.addData("sumColors 2", sumColors.val[2]);
        telemetry.update();

        // Change the bounding box color based on the sleeve color
        if (sumColors.val[0] == minColor)
            position = "CENTER";
        else if (sumColors.val[1] == minColor)
            position = "RIGHT";
        else
            position = "LEFT";

        // Change the bounding box color based on the sleeve color
        Imgproc.rectangle(
                input,
                sleeve_pointA,
                sleeve_pointB,
                colorOfPosition(),
                2
        );

        // Release and return input
        areaMat.release();
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public String getPosition() {
        return position;
    }
}