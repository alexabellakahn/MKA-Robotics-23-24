package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MAIN", group="Linear Opmode")
public class TeleOp2023l2024 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor carousel = null;
    //private DcMotor revArm = null;
    private Servo grip = null;

    private Servo lift = null;

    private Servo plane = null;

    private Servo rightGrip = null;

    private Servo leftGrip = null;

    private BNO055IMU imu = null;

    public void movement(){
        double Lpower = 0.58;
        double Rpower = 0.3;
        boolean reverseStick = true;

        double r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
        double robotAngle = Math.atan2((!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) + 3 * Math.PI / 4;
        double rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)*1;
        double rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)*1;
        double v1 = r * Math.cos(robotAngle) -rightX+rightY;
        double v2 = r * Math.sin(robotAngle) +rightX+rightY;
        double v3 = r * Math.sin(robotAngle) -rightX+rightY;
        double v4 = r * Math.cos(robotAngle) +rightX+rightY;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }
    public void slideMove(int targetpos, double power){
        carousel.setTargetPosition(targetpos);
        carousel.setPower(power);
        while (carousel.isBusy()) {
            movement();
            if(Math.abs(carousel.getCurrentPosition() - targetpos) < 50){
                break;
            }
        }
    }
    double leftOpen = 0;
    double rightOpen = 0;

    public void openGrip(){ //open grabber
        leftGrip.setPosition(0); //need to test
        rightGrip.setPosition(0);
    }
    public void closeGrip(){ //close grabber
        leftGrip.setPosition(0); //need to test
        rightGrip.setPosition(0);
    }

    public void launchPlane(){
        plane.setPosition(0);// need to test
    }



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized", "haggis");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(Servo.class, "lift");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        plane = hardwareMap.get(Servo.class, "plane");



        carousel  = hardwareMap.get(DcMotor.class, "carousel");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition(0);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            movement();

            double cp = 1.0;

            double lp=1.0;
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
            carousel.setPower(cp);

            if(gamepad1.y && gamepad1.a) { // y and a pressed at same time = plane l
                launchPlane();
            }
            if(gamepad1.y){// y button pressed = open grip
                openGrip();
            }

            if(gamepad1.b){ // b button pressed = close grip
                closeGrip();
            }
            if(gamepad1.dpad_right){ // right pressed = slider up
                slideMove(3050, cp);
            }
            if(gamepad1.dpad_up){ // up pressed =
                lift.setPosition(0);//test
                slideMove(2323, cp);
                lift.setPosition(0);//test

            }

            if(gamepad1.dpad_left){
                slideMove(1230, cp);
            }
            if(gamepad1.dpad_down){ // down pressed = slide down
                slideMove(0, cp);
            }
            int z = carousel.getCurrentPosition();
            telemetry.addData("Encoder"," CurrPos: %d, Power: %f, TargetPos: %d",
                    z, carousel.getPower(), carousel.getTargetPosition());
            telemetry.addData("gripPos", Double.toString(grip.getPosition()));
            telemetry.update();
        }
    }
}