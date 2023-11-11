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
    
    // Color definitions (R, G, B)
    private final Scalar RED = new Scalar(255, 0, 0);
    private final Scalar BLUE = new Scalar(0, 0, 255);
    private final Scalar MAGENTA = new Scalar(0, 0, 0);

    private Telemetry telemetry = null;

    // Anchor point definitions
    public static int LR_WIDTH = 550, LR_HEIGHT = 210;
    public static int MID_WIDTH = 250, MID_HEIGHT = 400;

    private static Point TOPLEFT = new Point(0, 200);
    private static Point leftBoxA = new Point(
            TOPLEFT.x,
            TOPLEFT.y);

    private static Point leftBoxB = new Point(
            TOPLEFT.x + LR_WIDTH,
            TOPLEFT.y + LR_HEIGHT);

    private static Point TOPMID = new Point(770, 28);
    private static Point midBoxA = new Point(
            TOPMID.x,
            TOPMID.y);

    private static Point midBoxB = new Point(
            TOPMID.x + MID_WIDTH,
            TOPMID.y + MID_HEIGHT);

    private static Point TOPRIGHT = new Point(230, 28);


    private static Rect leftRect = new Rect(leftBoxA, leftBoxB);
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
        Scalar midColors = process(input, midRect);

        telemetry.addData("LEFT",leftColors.val[0]);
        telemetry.addData("sumColors 1", midColors.val[1]);
        telemetry.addData("MID", midColors.val[0]);
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
                midBoxA,
                midBoxB,
                RED,
                2
        );

        return input;
    }
}