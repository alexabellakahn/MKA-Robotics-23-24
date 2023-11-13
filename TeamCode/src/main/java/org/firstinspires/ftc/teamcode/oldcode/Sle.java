package org.firstinspires.ftc.teamcode.oldcode;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;



import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;



import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

@Autonomous(name="Sle", group ="Concept")

public class Sle extends LinearOpMode {
    //IntegratingGyroscope gyro;
    //ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    //DigitalChannel digitalTouch;
    //NormalizedColorSensor colorSensor;
    //ColorSensor colorSensor1;
    private ColorSensor cSensor;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor carousel = null;
    private DcMotor revArm = null;
    private DcMotor lift = null; 
    private Servo gripLeft = null;
    private Servo gripRight = null;

    
    //movement variables
    private double power;
    private double robotAngle;
    private double rightX;
    private double v1;
    private double v2;
    private double v3;
    private double v4;
    
    private double powMaster=1.0;
    private double powRatio=1.0;
    private double rotRatio=1.0;
    private boolean reverseStrafe=false;
    
     //move(speed, milliseconds running, degrees from 0 to 360)
     //1000 milliseconds = 1 second
     //0 degrees is forward
     
     //550 encoder is approximately 24in so 23 per inch 
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
     //0-radius rotation, (speed, milliseconds running)
     //positive power turns right, negative power turns left
     public void rotate(int Rotations, double pow, boolean right){
        int p1 = leftFront.getTargetPosition();
        int p2 = rightFront.getTargetPosition();
        int p3 = leftRear.getTargetPosition();
        int p4 = rightRear.getTargetPosition();
        leftFront.setTargetPosition((Rotations+p1) * (right ? -1 : 1));
        rightFront.setTargetPosition((Rotations+p2) * (!right ? -1 : 1));
        leftRear.setTargetPosition((Rotations+p3)* (right ? -1 : 1));
        rightRear.setTargetPosition((Rotations+p4) * (!right ? -1 : 1));
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftRear.setPower(pow);
        rightRear.setPower(pow);
        while(leftFront.isBusy()){idle();}
     }
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
     public void stop(int millis){
        v1 = 0.0;
        v2 = 0.0;
        v3 = 0.0;
        v4 = 0.0;
        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
     }
     

@Override public void runOpMode() throws InterruptedException {
        
        //assign configurations
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        carousel  = hardwareMap.get(DcMotor.class, "carousel");
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");

        cSensor = hardwareMap.get(ColorSensor.class, "cs");
        cSensor.enableLed(true);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.REVERSE);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition(0);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(0);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setTargetPosition(0);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setTargetPosition(0);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setTargetPosition(0);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        gripLeft.setPosition(.55);
        gripRight.setPosition(.55);
        opModeIsActive();

        strafe(600,.2,false);
        Forward(1350,.2,true);
        strafe(300,.2,false);
        

        
         
         
    
    telemetry.update();
    }
}

