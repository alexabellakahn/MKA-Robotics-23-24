package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public interface PropDetection {
    public String getPosition();
    public Mat processFrame(Mat input);

}
