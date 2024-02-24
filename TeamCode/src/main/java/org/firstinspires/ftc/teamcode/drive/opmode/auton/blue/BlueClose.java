package org.firstinspires.ftc.teamcode.drive.opmode.auton.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.google.ar.core.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SleeveDetectionBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueClose", group = "Linear Opmode")
public class BlueClose extends LinearOpMode {
    private DcMotorEx carousel;
    private Servo lift, leftGrip, rightGrip;
    //private DistanceSensor distanceSensor;

    private boolean liftToggle = false;
    private boolean gripToggle = false;

    private enum PropPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }; // 0: left, 1: middle, 2: right

    private PropPosition position;

    SleeveDetectionBlue sleeveDetection = null;




    private void closeGrip() { //close grabber
        leftGrip.setPosition(.2825); // .25
        rightGrip.setPosition(.91);
        gripToggle = true;
    }

    private void openGrip2() { //drop outer pixel
        leftGrip.setPosition(.365); // .25
        rightGrip.setPosition(.91);
        gripToggle = true;
    }


    private void openGrip1() { //open grabber
        leftGrip.setPosition(.43); //need to test
        rightGrip.setPosition(.76);
        gripToggle = false;
    }

    private void dropPixel(int pixelNumber) {
        switch (pixelNumber) {
            case 0:
                openGrip2();
                sleep(150);
                closeGrip();
                break;
            case 1:
                up();
                sleep(500);
                openGrip1();
                sleep(500);
                closeGrip();
                down();
                break;

        }
    }

    private void up() {
        lift.setPosition(.7265);
        liftToggle = true;
    }

    private void down() {
        lift.setPosition(.6775);
        liftToggle = false;
    }

    private void slideMove(int targetpos, double power) {
        carousel.setTargetPosition(targetpos);
        carousel.setPower(power);
        while (carousel.isBusy()) {
            if (Math.abs(carousel.getCurrentPosition() - targetpos) < 50) {
                break;
            }
        };
        carousel.setPower(0);
    }


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = hardwareMap.get(Servo.class, "lift");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setTargetPosition(0);
        carousel.setPower(1);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (carousel.isBusy()) {idle();};
        carousel.setPower(0);
        //camera stuff
        OpenCvCamera camera; //initialize camera var
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
                sleeveDetection = new SleeveDetectionBlue(telemetry);
                camera.setPipeline(sleeveDetection);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

//        boolean selected = false;

        String objectLocation = "";

        while(!isStopRequested() && !opModeIsActive()) {
            if (sleeveDetection != null) {
                objectLocation = sleeveDetection.getPosition();
                telemetry.addData("objectLocation", objectLocation);
                telemetry.update();
            }
        }

        waitForStart();

        if(isStopRequested()) return;


        telemetry.update();
        camera.closeCameraDevice();

        switch (objectLocation) {
            case  "left":
                position = PropPosition.LEFT;
                break;
            case "mid":
                position = PropPosition.MIDDLE;
                break;
            case "right":
                position = PropPosition.RIGHT;
                break;
        }



        drive.setPoseEstimate(new Pose2d());


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(27)
                .build();

        Trajectory right1 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(5.5)
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(8)
                .build();

        Trajectory right3 = drive.trajectoryBuilder(right2.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .strafeRight(2.5)
                .build();

        Trajectory middle1 = drive.trajectoryBuilder(traj1.end())
                .forward(4.5)
                .build();

        Trajectory left1 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(5.5)
                .build();

        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .back(8)
                .build();

        Trajectory left3 = drive.trajectoryBuilder(left2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .strafeLeft(2.5)
                .build();


        Trajectory lastTraj = traj1;

        Trajectory traj4 = drive.trajectoryBuilder(lastTraj.end())
                .back(30)
                .build();

        switch (position) {
            case LEFT:
                traj4 = drive.trajectoryBuilder(left3.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(9.5)
                        .build();
                break;
            case MIDDLE:
                traj4 = drive.trajectoryBuilder(middle1.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(13)
                        .build();
                break;
            case RIGHT:
                traj4 = drive.trajectoryBuilder(right3.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(9.5)
                        .build();
                break;
        }

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(24.5)
                .build();

        Trajectory leftEnd = drive.trajectoryBuilder(traj5.end())
                .strafeRight(13.5)
                .build();

        Trajectory middleEnd = drive.trajectoryBuilder(traj5.end())
                .strafeRight(7.75)
                .build();

        Trajectory rightEnd = drive.trajectoryBuilder(traj5.end())
                .strafeRight(15)
                .build();

        switch (position) {
            case LEFT:
                lastTraj = traj5; //leftEnd;
                break;
            case MIDDLE:
                lastTraj = middleEnd;
                break;
            case RIGHT:
                lastTraj = rightEnd;
                break;
        }

        Trajectory traj7 = drive.trajectoryBuilder(lastTraj.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .back(17.5)
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .forward(5)
                .build();






        telemetry.addData("Object Position", position);
        telemetry.update();


        waitForStart();

        if(isStopRequested()) return;

        down();
        sleep(250);
        closeGrip();
        sleep(250);
        carousel.setPower(1);
        carousel.setTargetPosition(390);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (carousel.isBusy()) {idle();}
        carousel.setPower(0);

        drive.followTrajectory(traj1);

        switch (position) {
            case LEFT:
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(left1);
                sleep(150);
                dropPixel(0);
                sleep(150);
                drive.followTrajectory(left2);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(left3);
                break;
            case MIDDLE:
                sleep(150);
                drive.followTrajectory(middle1);
                dropPixel(0);
                break;
            case RIGHT:
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(right1);
                sleep(150);
                dropPixel(0);
                sleep(150);
                drive.followTrajectory(right2);
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(right3);
                break;
        }

        drive.followTrajectory(traj4);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj5);

        switch (position) {
            case LEFT:
                //drive.followTrajectory(leftEnd);
                break;
            case MIDDLE:
                drive.followTrajectory(middleEnd);
                break;
            case RIGHT:
                drive.followTrajectory(rightEnd);
                break;
        }

        drive.turn(Math.toRadians(180));
        drive.followTrajectory(traj7);


//        while(distance > 2.5) {
//            backup = drive.trajectoryBuilder(lastTraj2.end())
//                    .back(0.75)
//                    .build();
//
//            drive.followTrajectory(backup);
//            lastTraj2 = backup;
//
//            cycles += 1;
//            distance = distanceSensor.getDistance(DistanceUnit.INCH);
//        }

        Trajectory park = drive.trajectoryBuilder(traj8.end())
                .strafeRight(12)
                .build();

        switch (position) {
            case LEFT:
                park = drive.trajectoryBuilder(traj8.end())
                    .strafeRight(12)
                    .build();
                break;

            case MIDDLE:
                park = drive.trajectoryBuilder(traj8.end())
                    .strafeRight(21.5)
                    .build();
                break;

            case RIGHT:
                park = drive.trajectoryBuilder(traj8.end())
                        .strafeRight(31)
                        .build();
                break;
        }



        //carousel.setTargetPosition(2700);

        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setPower(1);

        //while (carousel.isBusy()) {idle();};
        sleep(1000);
        carousel.setPower(0);

        sleep(250);



        dropPixel(1);
        sleep(1000);

        carousel.setPower(1);
        carousel.setTargetPosition(0);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (carousel.getCurrentPosition() > 3000) {};
        drive.followTrajectory(traj8);



        drive.followTrajectory(park);
        while (carousel.isBusy()) {idle();};
        carousel.setPower(0);


        // move slide
        // drop pixel
        // move slide






    }
}


