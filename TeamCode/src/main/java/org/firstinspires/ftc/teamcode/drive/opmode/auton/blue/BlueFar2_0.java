package org.firstinspires.ftc.teamcode.drive.opmode.auton.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.auton.utils.BaseAuto;
import org.firstinspires.ftc.teamcode.drive.opmode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueFar2.0", group = "Linear Opmode")
public class BlueFar2_0 extends BaseAuto {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = hardwareMap.get(Servo.class, "lift");
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");

        slide = hardwareMap.get(DcMotorEx.class, "carousel");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        Pose2d startPose = new Pose2d(-35.25, 62.7, Math.toRadians(270));

//        TrajectorySequence leftSequence = drive.trajectorySequenceBuilder(startPose)
//                .forward(27)
//                .turn(Math.toRadians(90))
//                .forward(5)
////                .addTemporalMarker(() -> {
////                    dropPixel(PixelDestination.SpikeMark);
////                })
//                .back(7.5)
//                .turn(Math.toRadians(-90))
//                .strafeLeft(2.5)
//                .forward(26)
//                .turn(Math.toRadians(90))
////                .addTemporalMarker(() -> {
////                    slideMove(200, 1);
////                })
//                .forward(72.5)
//                .strafeLeft(28.5)
//                .turn(Math.toRadians(180))
//                .back(17.5)
////                .addTemporalMarker(() -> {
////                    slideMove(2700, 1);
////                    dropPixel(PixelDestination.Backdrop);
////                    slideMove(0, 1);
////                })
//                .forward(5)
//                .strafeLeft(28)
//                .build();

        TrajectorySequence leftSequence = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30.25, 35.7, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-30.25, 9.7), Math.toRadians(0))
                //9.7
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(38.75, 9.7, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(56.25, 38.2))
                .lineToConstantHeading(new Vector2d(51.25, 10.2))
                .build();

        TrajectorySequence midSequence = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .build();

        TrajectorySequence rightSequence = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(5)
                .build();

        setAlliance(Alliance.BLUE);

        initOpenCV();

        while (!isStopRequested() && !opModeIsActive()) {
            if (cameraInitialized) {
                switch (propDetection.getPosition()) {
                    case "left":
                        propLocation = PropLocation.LEFT;
                        break;
                    case "mid":
                        propLocation = PropLocation.MIDDLE;
                        break;
                    case "right":
                        propLocation = PropLocation.RIGHT;
                        break;
                }
                telemetry.addData("Detected Location", propLocation);
                telemetry.update();
            }
        }

        waitForStart();

        if (isStopRequested()) return;

        telemetry.update();
        closeOpenCV();

        drive.setPoseEstimate(startPose);

        //closeGrip();
        //slideMove(390, 1);

        switch (propLocation) {
            case LEFT:
                drive.followTrajectorySequence(leftSequence);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(midSequence);
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightSequence);
                break;
        }
    }
}
