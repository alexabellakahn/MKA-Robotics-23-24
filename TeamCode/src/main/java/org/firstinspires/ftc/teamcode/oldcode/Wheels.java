package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Wheels", group="Linear Opmode")
public class Wheels extends LinearOpMode {
    
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;


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
         

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double Lpower = 0.4;
        double Rpower = 0.3;
        boolean reverseStick = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Lpower*Math.hypot((!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x, (!reverseStick)?-gamepad1.left_stick_y:-gamepad1.right_stick_y);
            double robotAngle = Math.atan2((!reverseStick)?gamepad1.left_stick_y:gamepad1.right_stick_y, (!reverseStick)?gamepad1.left_stick_x:gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = Rpower*((!reverseStick)?gamepad1.right_stick_x:gamepad1.left_stick_x)/1;
            double rightY = Rpower*((!reverseStick)?gamepad1.right_stick_y:gamepad1.left_stick_y)/1;
            double v1 = r * Math.sin(robotAngle) +rightX+rightY;
            double v2 = r * Math.cos(robotAngle) -rightX+rightY;
            double v3 = r * Math.cos(robotAngle) +rightX+rightY;
            double v4 = r * Math.sin(robotAngle) -rightX+rightY;

            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status","\n"+Double.toString(v1) +"\n"+ Double.toString(v2) +"\n"+ Double.toString(v3) +"\n"+ Double.toString(v4));
            telemetry.update();
        }
    }
}

                    
                    
                    
