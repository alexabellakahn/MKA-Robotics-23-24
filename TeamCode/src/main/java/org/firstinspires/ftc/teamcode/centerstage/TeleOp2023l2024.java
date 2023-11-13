package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "MAIN", group = "Linear Opmode")
public class TeleOp2023l2024 extends LinearOpMode {

    boolean liftToggle = false;
    boolean gripToggle = false;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    //private DcMotor revArm = null;
    private DcMotor rightRear = null;

    private Servo plane = null;
    private DcMotor carousel = null;
    private Servo lift = null;
    private Servo rightGrip = null;
    private Servo leftGrip = null;

    public void movement() {
        double Lpower = 0.58;
        double Rpower = 0.3;
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

    public void side() {
        lift.setPosition(.84);
        liftToggle = true;
    }

    public void slideUp() {
        while (gamepad1.right_trigger != 0) {
            carousel.setPower(-gamepad1.right_trigger);
        }
        carousel.setPower(0);
    }

    public void slideDown() {
        while (gamepad1.left_trigger != 0) {
            carousel.setPower(gamepad1.left_trigger);
        }
        carousel.setPower(0);
    }



    public void launchPlane(){
    plane.setPosition(1);// need to test
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


        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setDirection(DcMotor.Direction.REVERSE);


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        leftGrip.setPosition(.38);
        rightGrip.setPosition(.81);
        lift.setPosition(.64);

        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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

            if (gamepad1.left_trigger > 0) {
                slideDown();
            }
            else if (gamepad1.right_trigger > 0) slideUp();

            telemetry.addData("Power", carousel.getPower());
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
            telemetry.update();
        }
    }
}