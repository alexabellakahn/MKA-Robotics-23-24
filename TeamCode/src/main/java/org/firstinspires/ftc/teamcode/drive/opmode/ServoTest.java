package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class ServoTest extends LinearOpMode {
    private Servo plane = null;

    @Override
    public void runOpMode() throws InterruptedException {

        plane = hardwareMap.get(Servo.class, "plane");
        plane.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        double pos = 0;
        double step = 0.001;

        while (!isStopRequested()) {
            if (gamepad1.left_bumper) {
                while (gamepad1.left_bumper && pos > 0.0) {
                    pos -= step;
                    plane.setPosition(pos);
                }
            }
            else if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper && pos < 1.0) {
                    pos += step;
                    plane.setPosition(pos);
                }
            }

            telemetry.addData("Servo Position", plane.getPosition());
            telemetry.update();
        }
    }
}
