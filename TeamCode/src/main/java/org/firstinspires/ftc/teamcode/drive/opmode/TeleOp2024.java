package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "MAIN", group = "Linear Opmode")
public class TeleOp2024 extends LinearOpMode {

    boolean liftToggle = false;
    boolean gripToggle = false;

    int liftUpPos = -5725;
    boolean isCalibrated = false;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    //private DcMotor revArm = null;
    private DcMotor rightRear = null;

    private Servo plane = null;
    private DcMotor carousel = null;
    private DcMotor winch = null;
    private Servo lift = null;
    private Servo rightGrip = null;
    private Servo leftGrip = null;

    //private DistanceSensor distanceSensor = null;
    private TouchSensor touchSensor = null;

    private Encoder leftEncoder = null;
    private Encoder rightEncoder = null;
    private Encoder frontEncoder = null;

    private boolean nearBoard;
    private boolean resettingSlide = false;

    public void movement() {
        double modifier = 1;//nearBoard ? 0.65 : 1;
        double Lpower = 1*modifier;
        double Rpower = 1; //0.52*modifier;//*modifier;
        boolean reverseStick = true;

        double r = Lpower * Math.hypot((!reverseStick) ? gamepad1.left_stick_x : gamepad1.right_stick_x, (!reverseStick) ? -gamepad1.left_stick_y : -gamepad1.right_stick_y);
        double robotAngle = Math.atan2((!reverseStick) ? -gamepad1.left_stick_y : -gamepad1.right_stick_y, (!reverseStick) ? gamepad1.left_stick_x : gamepad1.right_stick_x) + 3 * Math.PI / 4;
        double rightX = Rpower * ((!reverseStick) ? gamepad1.right_stick_x : gamepad1.left_stick_x) * 1;
        double rightY = Rpower * ((!reverseStick) ? gamepad1.right_stick_y : gamepad1.left_stick_y) * 1;
        double v1 = r * Math.cos(robotAngle) - rightX + rightY;
        double v2 = r * Math.sin(robotAngle) + rightX + rightY;
        double v3 = r * Math.sin(robotAngle) - rightX + rightY;
        double v4 = r * Math.cos(robotAngle) + rightX + rightY;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }

    /*
    Important numbers:
    lift down: .64
    lift up: 1
    leftGrip open: .58
    leftGrip closed: .38
    rightGrip open: .8
    rightGrip closed: .81
     */
    public void slideMove(int targetpos, double power) {
        carousel.setTargetPosition(targetpos);
        carousel.setPower(power);
        while (carousel.isBusy()) {
            movement();
            if (Math.abs(carousel.getCurrentPosition() - targetpos) < 50) {
                break;
            }
        }
        carousel.setPower(0);
    }

    public void closeGrip() { //close grabber
        leftGrip.setPosition(.285); // .25
        rightGrip.setPosition(.91);
        gripToggle = true;
    }

    public void openGrip() { //open grabber
        leftGrip.setPosition(.43); //need to test .48
        rightGrip.setPosition(.76);
        gripToggle = false;
    }

    public void up() {
        lift.setPosition(.7165);
        liftToggle = true;
    }

    public void down() {
        lift.setPosition(.6775);
        liftToggle = false;
    }

    public void middle() {
        lift.setPosition(.84);
        liftToggle = true;
    }

    public void calibrateSlide() {
        while(!touchSensor.isPressed()) {
            carousel.setPower(1);
        }
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void liftSlide() {
        if (gamepad1.left_trigger != 0) {
            if (resettingSlide || carousel.getCurrentPosition() <= 0) {
                carousel.setPower(gamepad1.left_trigger);
            } else {
                carousel.setPower(0);
            }

        }
        else if (gamepad1.right_trigger != 0) {
            if (resettingSlide || carousel.getCurrentPosition() > -8390) {
                carousel.setPower(-gamepad1.right_trigger);
            } else {
                carousel.setPower(0);
            }
        }
    }





    public void launchPlane(){

        plane.setPosition(0.95);

        //plane.setPosition(0.20);
        sleep(800);

        plane.setPosition(.325); //.775


    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized", "haggis");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(Servo.class, "lift");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        plane = hardwareMap.get(Servo.class, "plane");
        carousel = hardwareMap.get(DcMotor.class, "carousel");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);



        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setDirection(DcMotor.Direction.REVERSE);


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        myLocalizer.setPoseEstimate(new Pose2d());



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        leftGrip.setPosition(.285);
        rightGrip.setPosition(.81);
        down(); //lift.setPosition(.713);
        plane.setPosition(.325);
        calibrateSlide();

        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            myLocalizer.update();

            Pose2d myPose = myLocalizer.getPoseEstimate();



            nearBoard = myPose.getX() > 35;

            movement();


            /*
            Carousel code commented out
            if(gamepad1.left_bumper){
                carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                carousel.setTargetPosition(0);

            }
            if (gamepad1.right_trigger > 0) {
                carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                carousel.setPower(gamepad1.right_trigger);
                carousel.setTargetPosition(carousel.getCurrentPosition());
            }
            else if (gamepad1.right_trigger <= 0) {
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(cp);
            }
            if (gamepad1.left_trigger > 0) {
                carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                carousel.setDirection(DcMotor.Direction.FORWARD);

                carousel.setPower(gamepad1.left_trigger);
                carousel.setTargetPosition(carousel.getCurrentPosition());
            }
            else if (gamepad1.left_trigger <= 0) {
                carousel.setDirection(DcMotor.Direction.REVERSE);
                carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                carousel.setPower(cp);
            }


            if (gamepad1.a) {
                carousel.setTargetPosition(carousel.getCurrentPosition() - 600 );
                while(carousel.isBusy()){
                    if(!gamepad1.a){
                        break;
                    }
                }
            }
            else if (gamepad1.b) {
                carousel.setTargetPosition(carousel.getCurrentPosition() + 100 );
                while(carousel.isBusy()){
                    if(!gamepad1.b)
                        break;
                }
            }

            if(gamepad1.y && gamepad1.a) launchPlane();

            if(gamepad1.dpad_right){ // right pressed = slider up
                slideMove(3050, 1);
            }
            if(gamepad1.dpad_up){ // up pressed =
                lift.setPosition(0);//test
                slideMove(2323, 1);
                lift.setPosition(0);//test
            }

            if(gamepad1.dpad_left){
                slideMove(1230, 1);
            }
            if(gamepad1.dpad_down){ // down pressed = slide down
                slideMove(0, 1);
            }

             */
            if(gamepad1.left_bumper && gamepad1.right_bumper) launchPlane();

            if (gamepad1.y) openGrip();

            else if (gamepad1.a) closeGrip();

            if (gamepad1.dpad_up) up();

            else if (gamepad1.dpad_down) {
                closeGrip();
                down();
            }
            /*
            if (gamepad1.left_bumper) {
                winch.setPower(1);
            } else if (gamepad1.right_bumper) {
                winch.setPower(-1);
            } else {
                winch.setPower(0);
            }
            */


            if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {liftSlide();}
            else {carousel.setPower(0);}

            if (gamepad1.share && gamepad1.options) {
                resettingSlide = !resettingSlide;

                if (!resettingSlide) {
                    carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    carousel.setDirection(DcMotor.Direction.REVERSE);
                }
            }

            if (carousel.getCurrentPosition() > liftUpPos) {
                down();
//                if (carousel.getPower() > 0) {
//                    closeGrip();
//                }
            }

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                liftUpPos = carousel.getCurrentPosition();
                isCalibrated = true;
            }



            telemetry.addData("Calibrated", isCalibrated);
            telemetry.addData("resettingSlide", resettingSlide);
            if (isCalibrated) {telemetry.addData("Calibrated liftUpPos", liftUpPos
            );}
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.addData("Near Board", nearBoard);
            telemetry.addData("Slide Power", carousel.getPower());
            telemetry.addData("Slide Position", carousel.getCurrentPosition());
            telemetry.addData("RIGHT", rightGrip.getPosition());
            telemetry.addData("LEFT", leftGrip.getPosition());
            telemetry.addData("RightD", rightGrip.getDirection());
            telemetry.addData("LeftD", leftGrip.getDirection());
            telemetry.addData("LIFT", lift.getPosition());
            telemetry.addData("LiftD", lift.getDirection());
            telemetry.addData("LT", gamepad1.right_trigger);
            telemetry.addData("RT", gamepad1.left_trigger);
            telemetry.addData("liftToggle", liftToggle);
            telemetry.addData("gripToggle", gripToggle);
            telemetry.addData("LiftD", lift.getDirection());
            telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
            telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
            telemetry.addData("middleEncoder", frontEncoder.getCurrentPosition());
            telemetry.addData("slideDown", touchSensor.isPressed());
            //telemetry.addData("distanceSensor", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}