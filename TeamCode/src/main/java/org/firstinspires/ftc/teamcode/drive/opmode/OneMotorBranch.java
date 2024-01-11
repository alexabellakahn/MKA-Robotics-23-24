package org.firstinspires.ftc.teamcode.drive.opmode;

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
 * Remove or comment  out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OneMotorBranch", group="Linear Opmode")
public class OneMotorBranch extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    /*private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;*/
    private DcMotor carousel = null;
    //private DcMotor revArm = null;
    //private DcMotor lift = null; 
    //private Servo grip = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized", "haggis");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        
         
        carousel  = hardwareMap.get(DcMotor.class, "carousel");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        
        carousel.setDirection(DcMotor.Direction.REVERSE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double Lpower = 0.58;
        double Rpower = 0.3;
        boolean reverseStick = true;
        boolean armUp = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
            double robotAngle = Math.atan2((!reverseStick)?gamepad1.left_stick_y:gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) +3*Math.PI / 4;
            double rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)*1;
            double rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)*-1;
            double v1 = r * Math.cos(robotAngle) -rightX+rightY;
            double v2 = r * Math.sin(robotAngle) +rightX+rightY;
            double v3 = r * Math.sin(robotAngle) -rightX+rightY;
            double v4 = r * Math.cos(robotAngle) +rightX+rightY;


        
            //Above is Wheel Code
            double cp=1.0;
            if (gamepad1.a) carousel.setPower(-cp);
            else if (gamepad1.b) carousel.setPower(cp);
            else carousel.setPower(0.0);
            
           
                
            

            
            

            // Show the elapsed game time and wheel power.
            int z = carousel.getCurrentPosition(); 
            telemetry.addData("Encoder"," CurrPos: %d, Power: %f, TargetPos: %d",
                z, carousel.getPower(), carousel.getTargetPosition());
            telemetry.update();
        }
    }
}


/*
                    int i=0;
                    while (gamepad1.right_bumper){
                        revArm.setTargetPosition(i);
                        i++;
                        sleep(30);
                        if (i>360) i=-360;
                        telemetry.addData("revArm",Integer.toString(i));
                        telemetry.update();
                    }
                    
                    revArm.setTargetPosition(-100);
                    sleep(1000); 
                    revArm.setTargetPosition(0);
                    sleep(1000); 
                    revArm.setTargetPosition(200);
                    sleep(1000); 
                    revArm.setTargetPosition(0);
                    sleep(1000);
                    revArm.setTargetPosition(-300);
                    sleep(1000); 
                    revArm.setTargetPosition(0);
                    sleep(1000);
                    revArm.setTargetPosition(360);
                    sleep(1000);
                    
                    if (gamepad1.dpad_down){ 
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setTargetPosition(-1000);
                rightFront.setTargetPosition(-1000);
                leftFront.setPower(0.5);
                rightFront.setPower(0.5);
                leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(5000);
                
            }
                    
                    */
                    
                    
                    
                    
