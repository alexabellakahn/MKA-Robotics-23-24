package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//keep
public class PropDetectionBlue extends OpenCvPipeline implements PropDetection{

    // Color definitions (R, G, B)
    private final Scalar RED = new Scalar(255, 0, 0);
    private final Scalar BLUE = new Scalar(0, 0, 255);
    private final Scalar GREEN = new Scalar(0, 255, 0);

    private String position = "right";

    private double minC = 60000; //Min amount of green allowed

    private Telemetry telemetry = null;

    // Anchor point definitions
    public static int LR_WIDTH = 550, LR_HEIGHT = 210;
    public static int MID_WIDTH = 375, MID_HEIGHT = 330;

    private static Point TOPLEFT = new Point(155, 450);
    private static Point leftBoxA = new Point(
            TOPLEFT.x,
            TOPLEFT.y);

    private static Point leftBoxB = new Point(
            TOPLEFT.x + LR_WIDTH,
            TOPLEFT.y + LR_HEIGHT);

    private static Point TOPMID = new Point(850,450);
    private static Point midBoxA = new Point(
            TOPMID.x,
            TOPMID.y);

    private static Point midBoxB = new Point(
            TOPMID.x + MID_WIDTH,
            TOPMID.y + MID_HEIGHT);

    private static Point TOPRIGHT = new Point(230, 28);


    private static Rect leftRect = new Rect(leftBoxA, leftBoxB);
    private static Rect midRect = new Rect(midBoxA, midBoxB);

    public PropDetectionBlue(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public String getPosition(){
        return position;
    }

    private static Scalar process(Mat input, Rect targetRect){
        Mat areaMat = input.submat(targetRect);
        Scalar sumColors = Core.sumElems(areaMat);
        areaMat.release();


        return sumColors;
    }
    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();
    public Scalar lower = new Scalar(92, 109, 128);
    public Scalar upper = new Scalar(123, 150, 180);

    public SimpleThresholdProcessor.ColorSpace colorSpace = SimpleThresholdProcessor.ColorSpace.RGB;

    @Override
    public Mat processFrame(Mat input) {
        Core.flip(input, input, +1);
        Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);
        Core.inRange(ycrcbMat, lower, upper, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);
        maskedInputMat.copyTo(input);
        binaryMat.release();
        ycrcbMat.release();


        Scalar leftColors = process(input, leftRect);
        Scalar midColors = process(input, midRect);

        if (leftColors.val[2] < midColors.val[2] && midColors.val[2] > minC) {
            position = "left";
        }
        else if(leftColors.val[2] > midColors.val[2] && leftColors.val[2] > minC) {
            position = "mid";
        }
        else{
            position = "right";
        }


        telemetry.addData("leftRed",leftColors.val[0]);
        telemetry.addData("midRed", midColors.val[0]);
        telemetry.addData("leftGreen",leftColors.val[1]);
        telemetry.addData("midGreen", midColors.val[1]);
        telemetry.addData("leftBlue",leftColors.val[2]);
        telemetry.addData("midBlue", midColors.val[2]);
        telemetry.addData("position", position);
        telemetry.update();

        Imgproc.rectangle(
                input,
                leftBoxA,
                leftBoxB,
                BLUE,
                2
        );

        Imgproc.rectangle(
                input,
                midBoxA,
                midBoxB,
                BLUE,
                2
        );

        return input;
    }
}