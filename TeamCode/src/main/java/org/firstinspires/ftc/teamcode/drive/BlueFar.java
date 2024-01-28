package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.google.ar.core.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//camera imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


@Autonomous(name = "BlueFar", group = "Linear Opmode")
public class BlueFar extends LinearOpMode {
    private DcMotorEx carousel;
    private Servo lift, leftGrip, rightGrip;
    private DistanceSensor distanceSensor;

    boolean liftToggle = false;
    boolean gripToggle = false;

    int objectPosition = 0; // 0: left, 1: middle, 2: right

    OpenCvCamera camera; //initialize camera var


    public void closeGrip() { //close grabber
        leftGrip.setPosition(.25);
        rightGrip.setPosition(.91);
        gripToggle = true;
    }

    public void openGrip() { //open grabber
        leftGrip.setPosition(.48); //need to test
        rightGrip.setPosition(.76);
        gripToggle = false;
    }

    public void up() {
        lift.setPosition(1);
        liftToggle = true;
    }

    public void down() {
        lift.setPosition(.64);
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
    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";
    SleeveDetectionLeft sleeveDetection;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = hardwareMap.get(Servo.class, "lift");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        //camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetectionLeft(telemetry);

        camera.setPipeline(sleeveDetection);

        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        //carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        drive.setPoseEstimate(new Pose2d());


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(27)
                .build();

        Trajectory right1 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(7.5)
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(7.5)
                .build();

        Trajectory middle1 = drive.trajectoryBuilder(traj1.end())
                .forward(8)
                .build();

        Trajectory left1 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(7.5)
                .build();

        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .back(7.5)
                .build();


        Trajectory lastTraj = traj1;

        Trajectory traj4 = drive.trajectoryBuilder(lastTraj.end())
                .forward(30)
                .build();

        switch (objectPosition) {
            case 0:
                lastTraj = left2;
                traj4 = drive.trajectoryBuilder(lastTraj.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                        .forward(30)
                        .build();
                break;
            case 1:
                lastTraj = middle1;
                traj4 = drive.trajectoryBuilder(lastTraj.end())
                        .forward(28)
                        .build();
                break;
            case 2:
                lastTraj = right2;
                traj4 = drive.trajectoryBuilder(lastTraj.end().plus(new Pose2d(10, 0, Math.toRadians(90))))
                        .forward(30)
                        .build();
                break;
        }

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(80)
                .build();

        Trajectory leftEnd = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(38)
                .build();

        Trajectory middleEnd = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(35)
                .build();

        Trajectory rightEnd = drive.trajectoryBuilder(traj5.end())
                .strafeLeft(32)
                .build();

        switch (objectPosition) {
            case 0:
                lastTraj = leftEnd;
                break;
            case 1:
                lastTraj = middleEnd;
                break;
            case 2:
                lastTraj = rightEnd;
                break;
        }

        Trajectory traj7 = drive.trajectoryBuilder(lastTraj.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .back(5)
                .build();







        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);

        switch (objectPosition) {
            case 0:
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(left1);
                sleep(300);
                drive.followTrajectory(left2);
                drive.turn(Math.toRadians(-90));
                break;
            case 1:
                sleep(300);
                drive.followTrajectory(middle1);
                break;
            case 2:
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(right1);
                sleep(300);
                drive.followTrajectory(right2);
                drive.turn(Math.toRadians(90));
                break;
        }

        drive.followTrajectory(traj4);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj5);

        switch (objectPosition) {
            case 0:
                drive.followTrajectory(leftEnd);
                break;
            case 1:
                drive.followTrajectory(middleEnd);
                break;
            case 2:
                drive.followTrajectory(rightEnd);
                break;
        }

        drive.turn(Math.toRadians(180));
        drive.followTrajectory(traj7);

        Trajectory backup;
        int cycles = 0;
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);

        Trajectory lastTraj2 = traj7;

        while(distance > 2.5) {
            backup = drive.trajectoryBuilder(lastTraj2.end())
                    .back(0.75)
                    .build();

            drive.followTrajectory(backup);
            lastTraj2 = backup;

            cycles += 1;
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
        }

        Trajectory park = drive.trajectoryBuilder(lastTraj2.end())
                .strafeRight(38-objectPosition*3)
                .build();

        drive.followTrajectory(park);

        // move slide
        // drop pixel
        // move slide






    }
}


