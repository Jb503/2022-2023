package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto", preselectTeleOp = "TeleOp")
public class Auto extends LinearOpMode {

    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private DcMotorEx frontleft;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx frontright;
    private DcMotorEx armmotor;
    private Servo Claw;

    Recognition recognition;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    // UNITS ARE METERS
    double tagsize = 0.0381; // 1.5 inches

    int ID_TAG_OF_INTEREST  = 4; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 5; // Tag ID 2 from the 36h11 family
    int ID_TAG_OF_INTEREST3 = 6; // Tag ID 3 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        int index;
        int inToTicks = 119;
        int team = 0;
        int side = 0;
        int autoParkPosition = 0;
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backright = hardwareMap.get(DcMotorEx.class, "back right");
        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        armmotor = hardwareMap.get(DcMotorEx.class, "arm motor");
        Claw = hardwareMap.get(Servo.class, "Claw");

        // https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback
        // Put initialization blocks here.
        // 3.85827- wheel
        // 1440- motor
        // 119 ticks per inch

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        armmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Stop_and_reset();
        //Arm_Reset();
        while(!opModeIsActive() && !isStopRequested()){
            if(gamepad1.a){
                telemetry.addData("team", "red");
                telemetry.addData("side", "left");
                telemetry.update();
                team=0;
                side=0;
            }
            else if (gamepad1.x){
                telemetry.addData("team", "red");
                telemetry.addData("side", "right");
                telemetry.update();
                team=0;
                side=1;
            }
            else if (gamepad1.b){
                telemetry.addData("team", "blue");
                telemetry.addData("side", "left");
                telemetry.update();
                team=1;
                side=0;
            }
            else if (gamepad1.y){
                telemetry.addData("team", "blue");
                telemetry.addData("side", "right");
                telemetry.update();
                team=1;
                side=1;
            }

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST) {
                        autoParkPosition = 0;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST2) {
                        autoParkPosition = 1;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST3) {
                        autoParkPosition = 2;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            //Gets inputs before init for gui
            telemetry.addData("Detected: ", autoParkPosition);
        }


        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        // Wait for start command from Driver Station.
        waitForStart();
        camera.stopStreaming();
        if (opModeIsActive()) {
            //Stop_and_reset();
            if (team == 0){ //red
                if (side==0){ //left
                    if (autoParkPosition == 0) {
                        redLeft();
                        strafeLeft(20);
                        armDown(35);

                    } else if (autoParkPosition == 1) {
                        redLeft();
                        strafeLeft(10);
                        armDown(35);

                    } else if (autoParkPosition == 2) {
                        redLeft();
                        strafeRight(10);
                        armDown(35);

                    } else {
                        telemetry.update();
                    }

                }
                else{ //red right
                    if (autoParkPosition == 0) {
                        redRight();
                        strafeLeft(10);
                        armDown(35);

                    } else if (autoParkPosition == 1) {
                        redRight();
                        strafeRight(6);
                        armDown(35);

                    } else if (autoParkPosition == 2) {
                        redRight();
                        strafeRight(15);
                        sleep(300);
                        Backward(7);
                        strafeRight(8);
                        armDown(35);

                    } else {
                        telemetry.update();
                    }

                }
            }
            else if (team==1){ //blue
                if(side==0){ //left
                    if (autoParkPosition == 0) {
                        blueLeft();
                        strafeLeft(25);
                        armDown(35);

                    } else if (autoParkPosition == 1) {
                        blueLeft();
                        strafeLeft(10);
                        armDown(35);

                    } else if (autoParkPosition == 2) {
                        blueLeft();
                        strafeRight(10);
                        armDown(35);

                    } else {
                        telemetry.update();
                    }

                }
                else{ //right
                    if (autoParkPosition == 0) {
                        blueRight();
                        strafeLeft(8);
                        armDown(35);

                    } else if (autoParkPosition == 1) {
                        blueRight();
                        strafeRight(8);
                        armDown(35);

                    } else if (autoParkPosition == 2) {
                        blueRight();
                        strafeRight(12);
                        sleep(300);
                        Backward(9);
                        strafeRight(8);
                        Backward(5);
                        armDown(35);

                    } else {
                        telemetry.update();
                    }

                }
            }
        }

    }




    public void Score() {
        // forward
        // strafe right
        // raise arm
        // open claw
        // move back
        // strafe left
        // go to starting position
    }

    public void Stop_and_reset() {

        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Arm_Reset() {
        armmotor.setPower(0);
        armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Arm_Run() {
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armmotor.setPower(0);
        while (armmotor.isBusy() && opModeIsActive()) {
        }
        //Arm_Reset();
    }

    public void Forward(long inches) {
        backleft.setPower(.75);
        backright.setPower(.75);
        frontleft.setPower(.75);
        frontright.setPower(.75);
        sleep((long) ((inches*(1000 / 19.2 )*3)/2));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void Backward(long inches) {
        backleft.setPower(-0.75);
        backright.setPower(-0.75);
        frontleft.setPower(-0.75);
        frontright.setPower(-0.75);
        sleep((long) ((inches*(1000 / 19.2 )*3)/2));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void strafeRight(long inches) {
        backleft.setPower(-.7);
        backright.setPower(.7);
        frontleft.setPower(.7);
        frontright.setPower(-.7);
        sleep((long) (inches*(1000/15.1)*2));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void strafeLeft(long inches) {
        backleft.setPower(.7);
        backright.setPower(-.7);
        frontleft.setPower(-.7);
        frontright.setPower(.7);
        sleep((long) (inches*(1000/15.1)*2));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void turnRight(long degrees){
        backleft.setPower(.7);
        backright.setPower(-.7);
        frontleft.setPower(-.7);
        frontright.setPower(.7);
        sleep((long) (degrees*(1000/15.1)*2));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void armUp(double inches) {
        armmotor.setPower(.75);
        sleep((long) (inches*(1000/9.2)*1.5));
        armmotor.setPower(0);
    }
    public void armDown(double inches) {
        armmotor.setPower(-.75);
        sleep((long) (inches*(1000/9.2)));
        armmotor.setPower(0);
    }

    public void redLeft(){
        Claw.setPosition(0.85);
        Forward(43);
        strafeRight(9);
        armUp(32.5);
        Forward(4);
        sleep(700);
        Claw.setPosition(1);
        sleep(500);
        Backward(8);
    }
    public void blueLeft(){
        Claw.setPosition(0.85);
        Forward(40);
        strafeRight(10);
        armUp(32.5);
        Forward(3);
        sleep(700);
        Claw.setPosition(1);
        sleep(500);
        Backward(6);
    }
    public void redRight(){
        Claw.setPosition(0.85);
        Forward(44);
        strafeLeft(6);
        armUp(32);
        Forward(3);
        sleep(700);
        Claw.setPosition(1);
        sleep(700);
        Backward(5);
        sleep (500);
    }
    public void blueRight(){
        Claw.setPosition(0.85);
        Forward(42);
        strafeLeft(3);
        armUp(32);
        Forward(4);
        sleep(700);
        Claw.setPosition(1);
        sleep(700);
        Backward(4);
        sleep(500);
    }


//    public int inToTicks(double distance_in) {
//        double doubleticks = distance_in * (1120/(3.89827 * 3.14));
//        int ticksint = (int) Math.round(doubleticks);
//        return ticksint;
//    }
//    public void Forward(double inches) {
//        backleft.setTargetPosition(-inToTicks(inches));
//        backright.setTargetPosition(-inToTicks(inches));
//        frontleft.setTargetPosition(-inToTicks(inches));
//        frontright.setTargetPosition(-inToTicks(inches));
//        Run_to_position();
//    }
//
//    public void Right(double inches) {
//        backleft.setTargetPosition(inToTicks(inches));
//        backright.setTargetPosition(-inToTicks(inches));
//        frontleft.setTargetPosition(-inToTicks(inches));
//        frontright.setTargetPosition(inToTicks(inches));
//        Run_to_position();
//    }
//
//    public void Backward(double inches) {
//        backleft.setTargetPosition(inToTicks(inches));
//        backright.setTargetPosition(inToTicks(inches));
//        frontleft.setTargetPosition(inToTicks(inches));
//        frontright.setTargetPosition(inToTicks(inches));
//        Run_to_position();
//    }
//
//    public void Left(double inches) {
//        backleft.setTargetPosition(-inToTicks(inches));
//        backright.setTargetPosition(inToTicks(inches));
//        frontleft.setTargetPosition(inToTicks(inches));
//        frontright.setTargetPosition(-inToTicks(inches));
//        Run_to_position();
//    }
//
//    public void turnClockwise(double inches) {
//        backleft.setTargetPosition(-inToTicks(inches));
//        backright.setTargetPosition(inToTicks(inches));
//        frontleft.setTargetPosition(-inToTicks(inches));
//        frontright.setTargetPosition(inToTicks(inches));
//        Run_to_position();
//    }
//
//    public void turnCounterClockwise(double inches) {
//        backleft.setTargetPosition(inToTicks(inches));
//        backright.setTargetPosition(-inToTicks(inches));
//        frontleft.setTargetPosition(inToTicks(inches));
//        frontright.setTargetPosition(-inToTicks(inches));
//        Run_to_position();
//    }
//
//    public void Run_to_position() {
//        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backleft.setPower(1);
//        backright.setPower(1);
//        frontleft.setPower(1);
//        frontright.setPower(1);
//        while (backleft.isBusy() && backright.isBusy() && frontleft.isBusy() && frontright.isBusy() && opModeIsActive()) {
//            telemetry.addData("Motor ticks", backleft.getCurrentPosition());
//            telemetry.addData("Motor ticks", backright.getCurrentPosition());
//            telemetry.addData("Motor ticks", frontleft.getCurrentPosition());
//            telemetry.addData("Motor ticks", frontright.getCurrentPosition());
//            telemetry.update();
//        }
//        backleft.setPower(0);
//        backright.setPower(0);
//        frontleft.setPower(0);
//        frontright.setPower(0);
//        Stop_and_reset();
//    }
    void tagToTelemetry(AprilTagDetection detection) {
      telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
      telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
      telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
      telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
      telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
      telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
     telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
