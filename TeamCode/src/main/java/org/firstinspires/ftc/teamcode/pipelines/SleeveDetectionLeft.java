package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//keep
public class SleeveDetectionLeft extends OpenCvPipeline {
    
    // Color definitions
    private final Scalar RED = new Scalar(255, 255, 0);
    private final Scalar BLUE = new Scalar(0, 255, 255);
    private final Scalar MAGENTA = new Scalar(255, 0, 255);

    private Telemetry telemetry = null;

    // Anchor point definitions
    public static int LR_WIDTH = 70, LR_HEIGHT = 70;
    public static int MID_WIDTH = 70, MID_HEIGHT = 70;

    private static Point TOPLEFT = new Point(70, 28);
    private static Point leftBoxA = new Point(
            TOPLEFT.x,
            TOPLEFT.y);

    private static Point leftBoxB = new Point(
            TOPLEFT.x + LR_WIDTH,
            TOPLEFT.y + LR_HEIGHT);

    private static Point TOPMID = new Point(150, 28);
    private static Point midBoxA = new Point(
            TOPMID.x,
            TOPMID.y);

    private static Point midBoxB = new Point(
            TOPMID.x + LR_WIDTH,
            TOPMID.y + LR_HEIGHT);

    private static Point TOPRIGHT = new Point(230, 28);

    private static Point rightBoxA = new Point(
            TOPRIGHT.x,
            TOPRIGHT.y);

    private static Point rightBoxB = new Point(
            TOPRIGHT.x + LR_WIDTH,
            TOPRIGHT.y + LR_HEIGHT);


    private static Rect leftRect = new Rect(leftBoxA, leftBoxB);
    private static Rect rightRect = new Rect(rightBoxA, rightBoxB);
    private static Rect midRect = new Rect(midBoxA, midBoxB);

    public SleeveDetectionLeft(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private static Scalar process(Mat input, Rect targetRect){
        Mat areaMat = input.submat(targetRect);
        Scalar sumColors = Core.sumElems(areaMat);
        areaMat.release();


        return sumColors;
    }

    @Override
    public Mat processFrame(Mat input) {

        Scalar leftColors = process(input, leftRect);
        Scalar rightColors = process(input, rightRect);
        Scalar midColors = process(input, midRect);

        telemetry.addData("sumColors 0", midColors.val[0]);
        telemetry.addData("sumColors 1", midColors.val[1]);
        telemetry.addData("sumColors 2", midColors.val[2]);
        telemetry.update();

        Imgproc.rectangle(
                input,
                leftBoxA,
                leftBoxB,
                RED,
                2
        );

        Imgproc.rectangle(
                input,
                rightBoxA,
                rightBoxB,
                BLUE,
                2
        );

        Imgproc.rectangle(
                input,
                midBoxA,
                midBoxB,
                RED,
                2
        );

        return input;
    }
}