package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="SeanTestBranchBackup", group="Linear Opmode")
public class SeanTestBranchBackup extends LinearOpMode {

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
       
        //revArm = hardwareMap.get(DcMotor.class, "revArm");
        //lift = hardwareMap.get(DcMotor.class, "lift");
       
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        gripLeft.setDirection(Servo.Direction.FORWARD);
        gripRight.setDirection(Servo.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.REVERSE);
       
        gripLeft.setPosition(.8);
        gripRight.setPosition(.8);
       
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double Lpower = 0.58;
        double Rpower = 0.3;
        boolean reverseStick = true;
        boolean armUp = false;
       
        double increment = 0.2;
        double maxPosition = 0 + increment;
        double minPosition = 0 - increment;
       
        boolean toggle = true;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           
            double r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
            double robotAngle = Math.atan2((!reverseStick)?gamepad1.left_stick_y:gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) +3*Math.PI / 4;
            double rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)*1;
            double rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)*-1;
            double v1 = r * Math.cos(robotAngle) -rightX+rightY;
            double v2 = r * Math.sin(robotAngle) +rightX+rightY;
            double v3 = r * Math.sin(robotAngle) -rightX+rightY;
            double v4 = r * Math.cos(robotAngle) +rightX+rightY;
           
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);

            // Start of grab mechanism           
            if (gamepad1.x && gripLeft.getPosition() != minPosition){
                if(toggle){
                    gripLeft.setPosition(gripLeft.getPosition() - increment);
                    gripRight.setPosition(gripRight.getPosition() - increment);
                    toggle = !toggle;
                }
            }
            else if (gamepad1.y && gripLeft.getPosition() != maxPosition) {
                if(!toggle){
                    gripLeft.setPosition(gripLeft.getPosition() + increment );
                    gripRight.setPosition(gripRight.getPosition() + increment );
                    toggle = !toggle;
                }
            }
            // End of grab mechanism
     
            double cp=.5;
            double lp=1.0;
            if (gamepad1.a) {
                carousel.setTargetPosition(carousel.getCurrentPosition() - 100 );
                while(carousel.isBusy()){
                    if(!gamepad1.a)
                        break;
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
             
            if(gamepad1.dpad_right) {
                carousel.setTargetPosition(1050);
                carousel.setPower(cp);
                while (carousel.isBusy()) {
                    if(Math.abs(carousel.getCurrentPosition() - 1050) < 50)
                        break;
                //carousel.setPower(0);
                }
            }
            if(gamepad1.dpad_up) {
                carousel.setTargetPosition(850);
                carousel.setPower(cp);
                while(carousel.isBusy()) {
                    if(Math.abs(carousel.getCurrentPosition() - 850) < 50)
                        break;
                //carousel.setPower(0);
                }
            }
           
            if(gamepad1.dpad_left){
                carousel.setTargetPosition(550);
                carousel.setPower(cp);
                while(carousel.isBusy()) {
                    if (Math.abs(carousel.getCurrentPosition() - 550) < 50)
                    break;
                //carousel.setPower(0);
                }
            }
           
            if(gamepad1.dpad_down){
                carousel.setTargetPosition(0);
                carousel.setPower(cp);
                while(carousel.isBusy()) {
                    if (Math.abs(carousel.getCurrentPosition()) < 50)
                        break;
                }
                //carousel.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Grip Pos: ", Double.toString(pos));
            //telemetry.addData("Status","\n"+Boolean.toString(armUp) +"\n"+ Float.toString(rp) +"\n"+ Float.toString(lp) +"\n"+ Integer.toString(revArm.getCurrentPosition()));
            //telemetry.addData("Wheels","\n"+Boolean.toString(armUp) +"\n"+ Float.toString(rp) +"\n"+ Float.toString(lp) +"\n"+ Double.toString(v4));
            int z = carousel.getCurrentPosition();
            telemetry.addData("Encoder"," CurrPos: %d, Power: %f, TargetPos: %d",
                z, carousel.getPower(), carousel.getTargetPosition());
            telemetry.update();
        }
    }
}