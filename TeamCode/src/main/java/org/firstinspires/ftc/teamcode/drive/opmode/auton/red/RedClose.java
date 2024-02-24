package org.firstinspires.ftc.teamcode.drive.opmode.auton.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.google.ar.core.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SleeveDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedClose", group = "Linear Opmode")
public class RedClose extends LinearOpMode {
    private DcMotorEx carousel;
    private Servo lift, leftGrip, rightGrip;
    private DistanceSensor distanceSensor;

    boolean liftToggle = false;
    boolean gripToggle = false;

    int objectPosition = 1; // 0: left, 1: middle, 2: right
    SleeveDetectionRed sleeveDetection = null;




    public void closeGrip() { //close grabber
        leftGrip.setPosition(.2825); // .25
        rightGrip.setPosition(.91);
        gripToggle = true;
    }

    private void openGrip2() { //drop outer pixel
        leftGrip.setPosition(.365); // .365
        rightGrip.setPosition(.91);
        gripToggle = true;
    }


    public void openGrip1() { //open grabber
        leftGrip.setPosition(.43); //need to test
        rightGrip.setPosition(.76);
        gripToggle = false;
    }

    public void dropPixel(int pixelNumber) {
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

    public void up() {
        lift.setPosition(.7265);
        liftToggle = true;
    }

    public void down() {
        lift.setPosition(.6775);
        liftToggle = false;
    }

    public void slideMove(int targetpos, double power) {
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
                sleeveDetection = new SleeveDetectionRed(telemetry);
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
                objectPosition = 0;
                break;
            case "mid":
                objectPosition = 1;
                break;
            case "right":
                objectPosition = 2;
                break;
        }



//
//        boolean selected = false;
//
//        while (!selected) {
//            if (gamepad1.dpad_left) {
//                objectPosition = 0;
//                selected = true;
//            } else if (gamepad1.dpad_up) {
//                objectPosition = 1;
//                selected = true;
//            } else if (gamepad1.dpad_right) {
//                objectPosition = 2;
//                selected = true;
//            }
//        }



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

        switch (objectPosition) {
            case 0:
                traj4 = drive.trajectoryBuilder(left3.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(11)
                        .build();
                break;
            case 1:
                traj4 = drive.trajectoryBuilder(middle1.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(14.5)
                        .build();
                break;
            case 2:
                traj4 = drive.trajectoryBuilder(right3.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(11)
                        .build();
                break;
        }

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(-90)))) // red
                .forward(24.5)
                .build();

        Trajectory leftEnd = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(12)
                .build();

        Trajectory middleEnd = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(4.75)
                .build();

        Trajectory rightEnd = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(13.5)
                .build();

        switch (objectPosition) {
            case 0:
                lastTraj = leftEnd;
                break;
            case 1:
                lastTraj = middleEnd;
                break;
            case 2:
                lastTraj = traj5;// rightEnd;
                break;
        }

        Trajectory traj7 = drive.trajectoryBuilder(lastTraj.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .back(17.5)
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .forward(5)
                .build();






        telemetry.addData("Object Position", objectPosition);
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
        while (carousel.isBusy()) {idle();};
        carousel.setPower(0);

        drive.followTrajectory(traj1);

        switch (objectPosition) {
            case 0:
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(left1);
                sleep(150);
                dropPixel(0);
                sleep(150);
                drive.followTrajectory(left2);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(left3);
                break;
            case 1:
                sleep(150);
                drive.followTrajectory(middle1);
                dropPixel(0);
                break;
            case 2:
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
        drive.turn(Math.toRadians(-90)); // red
        drive.followTrajectory(traj5);

        switch (objectPosition) {
            case 0:
                drive.followTrajectory(leftEnd);
                break;
            case 1:
                drive.followTrajectory(middleEnd);
                break;
            case 2:
                //drive.followTrajectory(rightEnd);
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
                .strafeLeft(7.5+((2-objectPosition)*7.25))
                .build();


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


