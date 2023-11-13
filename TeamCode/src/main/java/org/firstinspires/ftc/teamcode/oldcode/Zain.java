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

@TeleOp(name="Test", group="Linear Opmode")
public class Zain extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
   
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    
    public boolean strafe(int Rotations, double pow, boolean right){
        int p1 = leftFront.getTargetPosition();
        int p2 = rightFront.getTargetPosition();
        int p3 = leftRear.getTargetPosition();
        int p4 = rightRear.getTargetPosition();
        leftFront.setTargetPosition(-(Rotations+p1)* (right ? 1 : -1));
        rightFront.setTargetPosition((Rotations+p2) * (right ? 1 : -1));
        leftRear.setTargetPosition((Rotations+p3)* (right ? 1 : -1));
        rightRear.setTargetPosition(-(Rotations+p4)* (right ? 1 : -1));
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftRear.setPower(pow);
        rightRear.setPower(pow);
        while(leftFront.isBusy()){idle();}
        return true; 
    }
    public void Forward(int Rotations, double power, boolean right){
        int p1 = leftFront.getTargetPosition();
        int p2 = rightFront.getTargetPosition();
        int p3 = leftRear.getTargetPosition();
        int p4 = rightRear.getTargetPosition();
        leftFront.setTargetPosition((p1+Rotations)* (right ? 1 : -1));
        rightFront.setTargetPosition((p2+Rotations) * (right ? 1 : -1));
        leftRear.setTargetPosition((p3+Rotations)* (right ? 1 : -1));
        rightRear.setTargetPosition((p4+Rotations)* (right ? 1 : -1));
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        while(leftFront.isBusy()){idle();}
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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        double Lpower = 0.58;
        double Rpower = 0.3;
        boolean reverseStick = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           /*            
            double r = Lpower*Math.hypot(gamepad1.right_stick_x*1.2, gamepad1.right_stick_y);
            double robotAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x*1.2) +3*Math.PI / 4;
            double rightX = Rpower*(gamepad1.left_stick_x)*1;
            double rightY = Rpower*(-gamepad1.left_stick_y)*-1;
            double v1 = r * Math.cos(robotAngle) -rightX+rightY;
            double v2 = r * Math.sin(robotAngle) +rightX+rightY;
            double v3 = r * Math.sin(robotAngle) -rightX+rightY;
            double v4 = r * Math.cos(robotAngle) +rightX+rightY;


            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
           */
           boolean FirstTime = true;
           if (FirstTime){
               Forward(400,.2,true);
               FirstTime = false; 
           }
           telemetry.update();
            
            
        }
    }
}