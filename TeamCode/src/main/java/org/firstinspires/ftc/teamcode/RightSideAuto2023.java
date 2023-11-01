package org.firstinspires.ftc.teamcode;

//keep
//import android.graphics.Bitmap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.Servo;


/*
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.vuforia.Frame;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
*/

@Autonomous(name="RightSideAuto2023", group ="Concept")

public class RightSideAuto2023 extends LinearOpMode {
    //IntegratingGyroscope gyro;
    //ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    //DigitalChannel digitalTouch;
    //NormalizedColorSensor colorSensor;
    //ColorSensor colorSensor1;
    //private ColorSensor cSensor;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor carousel = null;
    private DcMotor revArm = null;
    private DcMotor lift = null;
    private Servo gripLeft = null;
    private Servo gripRight = null;
    
    private BNO055IMU imu = null;


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
        

    public void forward(int Rotations, double pow, boolean forward){
        int p1 = leftFront.getTargetPosition();
        int p2 = rightFront.getTargetPosition();
        int p3 = leftRear.getTargetPosition();
        int p4 = rightRear.getTargetPosition();
        leftFront.setTargetPosition((p1 + (Rotations * (forward ? 1 : -1))));
        rightFront.setTargetPosition((p2 + (Rotations * (forward ? 1 : -1))));
        leftRear.setTargetPosition((p3 + (Rotations * (forward ? 1 : -1))));
        rightRear.setTargetPosition((p4 + (Rotations * (forward ? 1 : -1))));
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftRear.setPower(pow);
        rightRear.setPower(pow);
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){idle();}
        stop(0);
    }
    //0-radius rotation, (speed, milliseconds running)
    //positive power turns right, negative power turns left
    public void rotate(int Rotations, double pow, boolean right){
        int p1 = leftFront.getTargetPosition();
        int p2 = rightFront.getTargetPosition();
        int p3 = leftRear.getTargetPosition();
        int p4 = rightRear.getTargetPosition();
        leftFront.setTargetPosition(p1 + Rotations * (right ? -1 : 1));
        rightFront.setTargetPosition(p2 + Rotations * (!right ? -1 : 1));
        leftRear.setTargetPosition(p3 + Rotations * (right ? -1 : 1));
        rightRear.setTargetPosition(p4 + Rotations * (!right ? -1 : 1));
        leftFront.setPower(pow);
        rightFront.setPower(pow);
        leftRear.setPower(pow);
        rightRear.setPower(pow);
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){idle();}
        stop(0);
    }
    public boolean strafe(int Rotations, double p, boolean right){
        int p1 = leftFront.getTargetPosition();
        int p2 = rightFront.getTargetPosition();
        int p3 = leftRear.getTargetPosition();
        int p4 = rightRear.getTargetPosition();

        leftFront.setTargetPosition((p1 + (-Rotations * (right ? 1 : -1))));
        rightFront.setTargetPosition((p2 + (Rotations * (right ? 1 : -1))));
        leftRear.setTargetPosition((p3 + (Rotations * (right ? 1 : -1))));
        rightRear.setTargetPosition((p4 + (-Rotations * (right ? 1 : -1))));


        /*leftFront.setTargetPosition(-(Rotations+p1) * (!right ? 1 : -1));
        rightFront.setTargetPosition((Rotations+p2) * (!right ? 1 : -1));
        leftRear.setTargetPosition((Rotations+p3) * (!right ? 1 : -1));
        rightRear.setTargetPosition(-(Rotations+p4) * (!right ? 1 : -1));

         */
        leftFront.setPower(p);
        rightFront.setPower(p);
        leftRear.setPower(p);
        rightRear.setPower(p);
        //&& rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            idle();}
        stop(0);
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


    public static void openGrip(Servo gripLeft, Servo gripRight){
        gripLeft.setPosition(.6);
        gripRight.setPosition(.6);
    }

    public void closeGrip(Servo gripLeft, Servo gripRight){
        gripLeft.setPosition(.28);
        gripRight.setPosition(.28);
    }
    
    public void accurateRotate(double degrees, double p) {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double currentAngle = imu.getAngularOrientation().firstAngle;
        telemetry.addData("Starting Angle", currentAngle);
        while (!(currentAngle <= degrees + p*2 && currentAngle >= degrees-p*2)) {
            currentAngle = imu.getAngularOrientation().firstAngle;
            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();
            leftFront.setPower((degrees < 1) ? -p : p);
            rightFront.setPower((degrees < 1) ? p : -p);
            leftRear.setPower((degrees < 1) ? -p : p);
            rightRear.setPower((degrees < 1) ? p : -p);
        }

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    @Override public void runOpMode() throws InterruptedException {
        
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        //assign configurations
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        carousel  = hardwareMap.get(DcMotor.class, "carousel");
        //cSensor = hardwareMap.get(ColorSensor.class, "cs");
        //cSensor.enableLed(true);
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");
        gripRight.setDirection(Servo.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.REVERSE);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setPower(.5);
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
        

        closeGrip(gripLeft, gripRight);
        sleep(200);
        carousel.setTargetPosition(100);
        //End of assigning configurations 


        waitForStart();
        opModeIsActive();
        
        //Beginning of Autonomus instructions
        
        //carousel.setTargetPosition(750);
        
        while(carousel.isBusy()){idle();}
        
        //strafe(25, .4, true);
        
        
        //place robot at 8.2in from inner side of square
        
        /*
        
        Working auton
        
        forward(1209, .2, true);
        carousel.setTargetPosition(1100);
        rotate(349, .2, false);

        sleep(300);
        rotate(224, .2, false);
        sleep(300);
        */
        
        
        
        //while(carousel.isBusy()){idle();}
        
        //forward and rotate switch ahhhhhhhhhhhhhhhhh
        
        
        forward(1465, .3, true);
        
        
        
        /*
        carousel.setTargetPosition(1100);
        while(carousel.isBusy()){idle();}
        forward(235, .1, true);
        openGrip(gripLeft, gripRight);
        forward(300, .2, false);
        sleep(150);
        carousel.setTargetPosition(390);
        //while(carousel.isBusy()){idle();}
        //strafe(0, 0.0, true);
        strafe(255, .3, false);
        sleep(100);
        rotate(1155, .5, true);
        sleep(200);
        //forward(1, 0.0001, true);
        forward(650, .3, true);
        closeGrip(gripLeft, gripRight);
        */
        
        //535 is  a 90 degee rotation
        
        
    

 
        //End of Autonomus
        
        
        
        
        
        /*
        
        /////////Sean Sim Code///////////
        
        //carousel.setTargetPosition(750);
        //while(carousel.isBusy()){idle();}

        strafe(2250, .2, false);
        forward(4700, .2, true);
        strafe(1000, .2, true);

        //carousel.setTargetPosition(1050);
        //while(carousel.isBusy()){idle();}

        forward(150, .2, true);
        sleep(1000);
        //openGrip(gripLeft, gripRight);
        forward(260, .2, false);
        strafe(1000, .2, true);


        forward(2100, .2, false);
        strafe(2050, .2, true);

        /*
        * case 1:
        *
        * strafe(2050, .2, false);
        * Forward(2100, .2, false);
        *
        *case 2:
        *
        * Forward(2100, .2, false);
        *
        * case 3:
        *
        * strafe(2050, .2, true);
        * Forward(2100, .2, false);
        * */
        





        /*
        boolean a,b,c;
        if(cSensor.red() > 50){
            a = true;
            b = false;
            c = false;
        }
        else if(cSensor.blue() > 50){
            a = false;
            b = true;
            c = false;
        }
        else if(cSensor.green() > 50){
            a = false;
            b = false;
            c = true;
        }


    telemetry.addData("tot", Integer.toString(cSensor.alpha()));
    telemetry.addData("red", Integer.toString(cSensor.red()));
    telemetry.addData("green", Integer.toString(cSensor.green()));
    telemetry.addData("blue", Integer.toString(cSensor.blue()));
    */

        telemetry.update();
    }
}
