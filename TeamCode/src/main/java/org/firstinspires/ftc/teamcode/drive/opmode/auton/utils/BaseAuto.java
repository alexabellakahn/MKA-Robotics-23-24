package org.firstinspires.ftc.teamcode.drive.opmode.auton.utils;

import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.PropDetection;
import org.firstinspires.ftc.teamcode.vision.PropDetectionBlue;
import org.firstinspires.ftc.teamcode.vision.PropDetectionRed;
import org.firstinspires.ftc.teamcode.vision.SleeveDetectionBlue;
import org.firstinspires.ftc.teamcode.vision.SleeveDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.task.core.BaseTaskApi;

public abstract class BaseAuto extends LinearOpMode {
    protected DcMotorEx slide;
    protected Servo lift, leftGrip;

    OpenCvCamera camera;
    protected boolean cameraInitialized = false;

    protected enum Alliance {
        RED,
        BLUE
    }

    protected Alliance alliance;

    protected enum GripStage {
        INNER,
        OUTER
    }

     protected enum PixelDestination {
        SpikeMark,
        Backdrop
    }

    protected  enum PropLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    protected PropLocation propLocation;

    protected PropDetection propDetection;

    protected enum States {
        CAMERA,
        SPIKE_MARK,
        STAGE_DOOR,
        BACKDROP,
        PARK
    }

    States state = States.CAMERA;

    protected void setAlliance(Alliance alliance) {
        this.alliance = alliance;

        switch (alliance) {
            case BLUE:
                propDetection = new PropDetectionBlue(telemetry);
                break;
            case RED:
                propDetection = new PropDetectionRed(telemetry);
                break;
        }

    }

    protected void closeGrip() {
        leftGrip.setPosition(.2825);
        lift.setPosition(.6775);
    }

    protected void openGrip(GripStage stage) {
        switch (stage) {
            case INNER:
                leftGrip.setPosition(.365);
                break;

            case OUTER:
                leftGrip.setPosition(.43);
                break;
        }
    }

    protected void dropPixel(PixelDestination pixelDestination) {
       switch (pixelDestination) {
           case SpikeMark:
               openGrip(GripStage.INNER);
               sleep(150);
               closeGrip();
               break;

           case Backdrop:
               lift.setPosition(.7265);
               sleep(500);
               openGrip(GripStage.OUTER);
               sleep(500);
               closeGrip();
               break;
       }
    }

    protected void slideMove(int targetpos, double power) {
        slide.setTargetPosition(targetpos);
        slide.setPower(power);
        while (slide.isBusy()) {
            if (Math.abs(slide.getCurrentPosition() - targetpos) < 50) {
                break;
            }
        };
        slide.setPower(0);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline((OpenCvPipeline) propDetection);
                cameraInitialized = true;
            }

            @Override
            public void onError(int errorCode) {
                cameraInitialized = false;
            }
        });
    }

    protected void closeOpenCV() {
        camera.closeCameraDevice();
        cameraInitialized = false;
    }


}
