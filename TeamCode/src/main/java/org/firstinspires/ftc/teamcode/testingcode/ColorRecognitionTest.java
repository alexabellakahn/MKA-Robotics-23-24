package org.firstinspires.ftc.teamcode.testingcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//keep
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.SleeveDetectionLeft;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Signal Sleeve Test")
public class ColorRecognitionTest extends LinearOpMode {
    
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor carousel = null;
    private DcMotor revArm = null;
    private DcMotor lift = null;
    private Servo leftgrip = null;
    private Servo rightgrip = null;


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
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            idle();
        }
            
        stop(0);
        sleep(50);
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
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            idle();
        }
        stop(0);
        sleep(50);
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
        while(leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){idle();}
        stop(0);
        sleep(50);
        return true;
    }
    
    //Telemetry telemetry; 
    SleeveDetectionLeft sleeveDetectionLeft;
    OpenCvCamera camera;
    
    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetectionLeft = new SleeveDetectionLeft(telemetry);
        camera.setPipeline(sleeveDetectionLeft);
        
        //assign configurations
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        carousel  = hardwareMap.get(DcMotor.class, "carousel");
        //cSensor = hardwareMap.get(ColorSensor.class, "cs");
        //cSensor.enableLed(true);
        leftgrip = hardwareMap.get(Servo.class, "leftGrip");
        rightgrip = hardwareMap.get(Servo.class, "rightGrip");
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


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(960,544, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        String state = "";
        while (!isStarted()) {
        }
        

        waitForStart();
        opModeIsActive();
        
        strafe(80,.8,false);
        forward(570, .2, false);
        
        
        switch(state){
            case "LEFT":
                strafe(625,.2,false);
                break;
            case "RIGHT":
                strafe(625,.2,true);
                break;
        
        }
        //forward(170, .2, true);
        
        
    }
}
