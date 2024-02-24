package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.opmode.roadrunner.SampleMecanumDrive;

@Autonomous(name = "liftTest", group = "Linear Opmode")
public class liftTest extends LinearOpMode {
    private DcMotorEx carousel;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setTargetPosition(0);


        //carousel.setTargetPosition(2700);

        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setPower(1);

        //while (carousel.isBusy()) {idle();};
        sleep(1000);
        carousel.setPower(0);

        carousel.setPower(1);
        carousel.setTargetPosition(0);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}


