package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.google.ar.core.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

@Autonomous(name = "BlueClose", group = "Linear Opmode")
public class BlueClose extends LinearOpMode {
    private DcMotorEx carousel;
    private Servo lift, leftGrip, rightGrip;
    //private DistanceSensor distanceSensor;

    boolean liftToggle = false;
    boolean gripToggle = false;

    int objectPosition = 1; // 0: left, 1: middle, 2: right




    public void closeGrip() { //close grabber
        leftGrip.setPosition(.2825); // .25
        rightGrip.setPosition(.91);
        gripToggle = true;
    }

    private void openGrip2() { //drop outer pixel
        leftGrip.setPosition(.365); // .25
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
        lift.setPosition(.7165);
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
        carousel.setTargetPosition(0);
        carousel.setPower(.8);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (carousel.isBusy()) {};
        carousel.setPower(0);


        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        boolean selected = false;

        while (!selected) {
            if (gamepad1.dpad_left) {
                objectPosition = 0;
                selected = true;
            } else if (gamepad1.dpad_up) {
                objectPosition = 1;
                selected = true;
            } else if (gamepad1.dpad_right) {
                objectPosition = 2;
                selected = true;
            }
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

        switch (objectPosition) {
            case 0:
                traj4 = drive.trajectoryBuilder(left3.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(23)
                        .build();
                break;
            case 1:
                traj4 = drive.trajectoryBuilder(middle1.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(26.5)
                        .build();
                break;
            case 2:
                traj4 = drive.trajectoryBuilder(right3.end().plus(new Pose2d(0, 0, Math.toRadians(0))))
                        .back(23)
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
                .strafeRight(21.25)
                .build();

        Trajectory rightEnd = drive.trajectoryBuilder(traj5.end())
                .strafeRight(28.5)
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
        sleep(750);
        carousel.setPower(.8);
        carousel.setTargetPosition(390);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (carousel.isBusy()) {telemetry.addData("Slide Pos", carousel.getCurrentPosition());
            telemetry.update();};
        carousel.setPower(0);

        drive.followTrajectory(traj1);

        switch (objectPosition) {
            case 0:
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(left1);
                sleep(300);
                dropPixel(0);
                sleep(150);
                drive.followTrajectory(left2);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(left3);
                break;
            case 1:
                sleep(300);
                drive.followTrajectory(middle1);
                dropPixel(0);
                break;
            case 2:
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(right1);
                sleep(300);
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
                .strafeRight(12+((objectPosition)*9.5))
                .build();


        carousel.setPower(.8);
        carousel.setTargetPosition(3410);
        sleep(500);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (carousel.isBusy()) {telemetry.addData("Slide Pos", carousel.getCurrentPosition());
            telemetry.update();};
        sleep(250);
        carousel.setPower(0);

        sleep(250);



        dropPixel(1);

        carousel.setPower(.8);
        carousel.setTargetPosition(0);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (carousel.getCurrentPosition() > 3000) {};
        drive.followTrajectory(traj8);



        drive.followTrajectory(park);
        while (carousel.isBusy()) {};
        carousel.setPower(0);


        // move slide
        // drop pixel
        // move slide






    }
}


