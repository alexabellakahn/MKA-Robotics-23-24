package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

//keep
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

@TeleOp(name="TestTeleOp2023A", group="Linear Opmode")
public class TestTeleOp2023A extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
   
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor carousel = null;
    //private DcMotor revArm = null;
    private Servo gripLeft = null;
    private Servo gripRight = null;
    private BNO055IMU imu = null;

    private Servo lift = null;

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
       
        //revArm = hardwareMap.get(DcMotor.class, "revArm");
        lift = hardwareMap.get(Servo.class, "lift");
       
        gripLeft = hardwareMap.get(Servo.class, "leftGrip");
        gripRight = hardwareMap.get(Servo.class, "rightGrip");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        gripLeft.setDirection(Servo.Direction.FORWARD);
        gripRight.setDirection(Servo.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.REVERSE);
       
        gripLeft.setPosition(.48);
        gripRight.setPosition(.48);
       
       
        //lift.setDirection(DcMotor.Direction.FORWARD);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //revArm.setDirection(DcMotor.Direction.REVERSE);
        //revArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
       

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double c = 1.5; 
        double Lpower = 0.58*c;
        double Rpower = 0.3*c;
        boolean reverseStick = true;
        boolean armUp = false;
       
        double increment = 0.3;
        double maxPosition = 0 + increment;
        double minPosition = 0 - increment;
       
        boolean toggle = true;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           
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
           
            //Above is Wheel Code
            /*
            double cp=0.75;
            if (gamepad1.a) {
                carousel.setPower(-cp);
                idle();
            } else if (gamepad1.b){
                carousel.setPower(cp);
                idle();
            }
            //else carousel.setPower(0.0);
           
            carousel.setPower(0.0);
            */
           
            if (gamepad1.x && gripLeft.getPosition() != minPosition){
                if(toggle){
                    gripLeft.setPosition(gripLeft.getPosition() - (increment + -.05));
                    gripRight.setPosition(gripRight.getPosition() - (increment + -.05));
                    toggle = !toggle;
                }
            }
            else if (gamepad1.y && gripLeft.getPosition() != maxPosition) {
                if(!toggle){
                    gripLeft.setPosition(gripLeft.getPosition() + (increment -.05));
                    gripRight.setPosition(gripRight.getPosition() + (increment -.05));
                    toggle = !toggle;
                }
            }
            if(gamepad1.dpad_up){
                rightFront.setPower(1);
                leftFront.setPower(1);
                rightRear.setPower(1);
                leftRear.setPower(1);

            }
            /*
            if (gamepad1.right_stick_button) {
                double startAngle = imu.getAngularOrientation().firstAngle;
                telemetry.addData("Start Angle", startAngle);
                telemetry.update();
                double currentAngle = 0;
                
                 while (Math.abs(startAngle) - Math.abs(currentAngle) > 180) {
                    leftFront.setPower(0.5);
                    rightFront.setPower(-0.5);
                    leftRear.setPower(0.5);
                    rightRear.setPower(-0.5);
                    currentAngle = imu.getAngularOrientation().firstAngle;
                    telemetry.addData("Current Angle", currentAngle);
                    telemetry.update();
                 }
    
            }
            */
           
           
            double cp=1;
            /*
            if(gamepad1.dpad_left || gamepad1.dpad_right){
                int movement = gamepad1.dpad_left ? 200 : -200;
                carousel.setPower(cp);
                carousel.setTargetPosition(carousel.getTargetPosition() + movement);
                while(carousel.isBusy()){
                    idle();
                }
                //carousel.setPower(0);
            }
            */
           
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
            lp = 1.0; 
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
               
           
           
            if(gamepad1.dpad_right){
                carousel.setTargetPosition(3050);
                carousel.setPower(cp);
                while(carousel.isBusy()){
            r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
            robotAngle = Math.atan2((!reverseStick)?gamepad1.left_stick_y:gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) +3*Math.PI / 4;
            rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)*1;
            rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)*1;
            v1 = r * Math.cos(robotAngle) -rightX+rightY;
            v2 = r * Math.sin(robotAngle) +rightX+rightY;
            v3 = r * Math.sin(robotAngle) -rightX+rightY;
            v4 = r * Math.cos(robotAngle) +rightX+rightY;
           
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
            if(Math.abs(carousel.getCurrentPosition() - 3200) < 50){
                break;
                }
                //carousel.setPower(0);
            }
            }
            if(gamepad1.dpad_up){
                carousel.setTargetPosition(2323);
                carousel.setPower(cp);
                while(carousel.isBusy()){
            r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
            robotAngle = Math.atan2((!reverseStick)?gamepad1.left_stick_y:gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) +3*Math.PI / 4;
            rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)*1;
            rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)*1;
            v1 = r * Math.cos(robotAngle) -rightX+rightY;
            v2 = r * Math.sin(robotAngle) +rightX+rightY;
            v3 = r * Math.sin(robotAngle) -rightX+rightY;
            v4 = r * Math.cos(robotAngle) +rightX+rightY;
           
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
            if(Math.abs(carousel.getCurrentPosition() - 2323) < 50){
                break;
                }
                //carousel.setPower(0);
            }
            }
           
            if(gamepad1.dpad_left){
                carousel.setTargetPosition(1230);
                carousel.setPower(cp);
                while(carousel.isBusy()){
            r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
            robotAngle = Math.atan2((!reverseStick)?gamepad1.left_stick_y:gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) +3*Math.PI / 4;
            rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)*1;
            rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)*1;
            v1 = r * Math.cos(robotAngle) -rightX+rightY;
            v2 = r * Math.sin(robotAngle) +rightX+rightY;
            v3 = r * Math.sin(robotAngle) -rightX+rightY;
            v4 = r * Math.cos(robotAngle) +rightX+rightY;
           
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
            if(Math.abs(carousel.getCurrentPosition() - 1230) < 50){
                break;
                }
            }
            }
            if(gamepad1.dpad_down){
                carousel.setTargetPosition(0);
                carousel.setPower(cp);
                while (carousel.isBusy()) {
            r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
            robotAngle = Math.atan2((!reverseStick)?gamepad1.left_stick_y:gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) +3*Math.PI / 4;
            rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)*1;
            rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)*1;
            v1 = r * Math.cos(robotAngle) -rightX+rightY;
            v2 = r * Math.sin(robotAngle) +rightX+rightY;
            v3 = r * Math.sin(robotAngle) -rightX+rightY;
            v4 = r * Math.cos(robotAngle) +rightX+rightY;
           
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
                if(Math.abs(carousel.getCurrentPosition()) < 50){
                    break;
                    }
                }
                
            }
            int z = carousel.getCurrentPosition();
            telemetry.addData("Encoder"," CurrPos: %d, Power: %f, TargetPos: %d",
                z, carousel.getPower(), carousel.getTargetPosition());
            telemetry.addData("right", Double.toString(gripRight.getPosition()));
            telemetry.addData("left", Double.toString(gripLeft.getPosition()));
            telemetry.update();
        }
    }
}