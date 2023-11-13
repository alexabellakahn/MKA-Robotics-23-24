package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "SeanTestBranch2", group = "Concept")
public class SeanTestBranch2 extends LinearOpMode{

    //private ColorSensor cSensor;
    
    private ElapsedTime runtime = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor carousel = null;
    private DcMotor revArm = null;
    private DcMotor lift = null; 
    private Servo grip=null;

    //movement variables
    private double power;
    private double robotAngle;
    private double rightX;
    private double v1;
    private double v2;
    private double v3;
    private double v4;

    private double powRatio=1.0;
    private double rotRatio=1.0;
    private boolean reverseStrafe=false;
    
    //1000 milliseconds = 1 second
    //0 degrees is forward
    public void strafe(double pow, int millis, double degrees){
        power = pow*powRatio;
        robotAngle = degrees*Math.PI/180 + 1 * Math.PI / 4;
        v1 = reverseStrafe?power*Math.sin(robotAngle):power*Math.cos(robotAngle);
        v2 = reverseStrafe?power*Math.cos(robotAngle):power*Math.sin(robotAngle);
        v3 = reverseStrafe?power*Math.cos(robotAngle):power*Math.sin(robotAngle);
        v4 = reverseStrafe?power*Math.sin(robotAngle):power*Math.cos(robotAngle);
        
        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
        
        sleep(millis);
        setStrafe(0.0,0);
    }
    public void setStrafe(double pow, double degrees){
        power = pow*powRatio;
        robotAngle = degrees*Math.PI/180 + 1 * Math.PI / 4;
        v1 = reverseStrafe?power*Math.sin(robotAngle):power*Math.cos(robotAngle);
        v2 = reverseStrafe?power*Math.cos(robotAngle):power*Math.sin(robotAngle);
        v3 = reverseStrafe?power*Math.cos(robotAngle):power*Math.sin(robotAngle);
        v4 = reverseStrafe?power*Math.sin(robotAngle):power*Math.cos(robotAngle);
        
        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }
     
    //0-radius rotation, (speed, milliseconds running)
    //positive power turns right, negative power turns left
    public void rotate(double pow, int millis){
        power = pow*rotRatio;
        v1 = power;
        v2 = -power;
        v3 = power;
        v4 = -power;
        
        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
        
        sleep(millis);
        setStrafe(0.0,0);
    }
    public void gripOn(){
        grip.setDirection(Servo.Direction.REVERSE);
        grip.setPosition(0.9);
    }
    public void gripOff() {
        grip.setDirection(Servo.Direction.FORWARD);
        grip.setPosition(0.9);
    }
    
    @Override public void runOpMode() throws InterruptedException {
        runtime= new ElapsedTime();
        runtime.reset();
        
        //assign configurations
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        carousel  = hardwareMap.get(DcMotor.class, "carousel");
        revArm = hardwareMap.get(DcMotor.class, "revArm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        grip = hardwareMap.get(Servo.class, "grip");
        
        
        //cSensor = hardwareMap.get(ColorSensor.class, "cs");
        //cSensor.enableLed(true);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        revArm.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        revArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        opModeIsActive();
        
        //telemetry.addData("Time:", Runtime);
        
        strafe(1,500,0);
        rotate(.5, 1000);
        rotate(.5, 1000);
        grip.setDirection(Servo.Direction.FORWARD);
        grip.setPosition(0.1);
        sleep(2000);
        
        grip.setDirection(Servo.Direction.REVERSE);
        grip.setPosition(0.9);
        sleep(2000);
        
    }
}