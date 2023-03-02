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

@Autonomous(name = "Remote", preselectTeleOp = "TeleOp")
public class Remote extends LinearOpMode {

    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private DcMotorEx frontleft;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx frontright;
    private DcMotorEx armmotor;
    private Servo Claw;

    Recognition recognition;

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

        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        armmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Stop_and_reset();
        Arm_Reset();

        while(!opModeIsActive() && !isStopRequested()){
            if(gamepad1.a){
                telemetry.addData("High", "Junction");
                telemetry.update();
                team=0;
                side=0;
            }
            else if (gamepad1.x){
                telemetry.addData("Medium", "Junction");
                telemetry.update();
                team=0;
                side=1;
            }
            else if (gamepad1.b){
                telemetry.addData("Small", "Junction");
                telemetry.update();
                team=1;
                side=0;
            }
            else if (gamepad1.y){
                telemetry.addData("Corner", "Terminals");
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
            if (team == 0){ //High Junction- a
                if (side==0){
                    if (autoParkPosition == 0) {
//                        frontleft.setTargetPosition(30000);
//                        frontright.setTargetPosition(-30000);
//                        backleft.setTargetPosition(30000);
//                        backright.setTargetPosition(-30000);
//                        Run_to_position();
                        highJunction();
                        strafeLeft(40);
                        Backward(4);
                        armRunDown(33);
                    } else if (autoParkPosition == 1) {
                        highJunction();
                        strafeLeft(12);
                        armRunDown(33);

                    } else if (autoParkPosition == 2) {
                        highJunction();
                        strafeRight(15);
                        armRunDown(33);

                    } else {
                        telemetry.update();
                    }

                }
                else{ //Medium Junction- x
                    if (autoParkPosition == 0) {
                        medJunct();
                        strafeRight(40);
                        Forward(25);
                        armRunDown(21);

                    } else if (autoParkPosition == 1) {
                        medJunct();
                        strafeRight(14);
                        Forward(25);
                        armRunDown(21);

                    } else if (autoParkPosition == 2) {
                        medJunct();
                        strafeLeft(15);
                        Forward(25);
                        armRunDown(21);

                    } else {
                        telemetry.update();
                    }

                }
            }
            else if (team==1){ //Small Junction- b
                if(side==0){
                    if (autoParkPosition == 0) {
                        smallJunct();
                        strafeLeft(40);
                        Backward(4);
                        armRunDown(20);

                    } else if (autoParkPosition == 1) {
                        smallJunct();
                        strafeLeft(12);
                        armRunDown(24);

                    } else if (autoParkPosition == 2) {
                        smallJunct();
                        strafeRight(15);
                        armRunDown(24);

                    } else {
                        telemetry.update();
                    }

                }
                else{ //Terminals
                    if (autoParkPosition == 0) {
                        blueRight();
                        strafeLeft(30);
                        armRunDown(33);

                    } else if (autoParkPosition == 1) {
                        blueRight();
                        armRunDown(33);

                    } else if (autoParkPosition == 2) {
                        blueRight();
                        strafeRight(30);
                        Backward(4);
                        armRunDown(33);

                    } else {
                        telemetry.update();
                    }

                }
            }
        }

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

//    public void Forward(long inches) {
//        backleft.setPower(.75);
//        backright.setPower(.75);
//        frontleft.setPower(.75);
//        frontright.setPower(.75);
//        sleep((long) ((inches*(1000 / 19.2 )*3)/2));
//        backleft.setPower(0);
//        backright.setPower(0);
//        frontleft.setPower(0);
//        frontright.setPower(0);
//    }
//    public void Backward(long inches) {
//        backleft.setPower(-0.75);
//        backright.setPower(-0.75);
//        frontleft.setPower(-0.75);
//        frontright.setPower(-0.75);
//        sleep((long) ((inches*(1000 / 19.2 )*3)/2));
//        backleft.setPower(0);
//        backright.setPower(0);
//        frontleft.setPower(0);
//        frontright.setPower(0);
//    }
//    public void strafeRight(long inches) {
//        backleft.setPower(-.7);
//        backright.setPower(.7);
//        frontleft.setPower(.7);
//        frontright.setPower(-.7);
//        sleep((long) (inches*(1000/15.1)*2));
//        backleft.setPower(0);
//        backright.setPower(0);
//        frontleft.setPower(0);
//        frontright.setPower(0);
//    }
//    public void strafeLeft(long inches) {
//        backleft.setPower(.7);
//        backright.setPower(-.7);
//        frontleft.setPower(-.7);
//        frontright.setPower(.7);
//        sleep((long) (inches*(1000/15.1)*2));
//        backleft.setPower(0);
//        backright.setPower(0);
//        frontleft.setPower(0);
//        frontright.setPower(0);
//    }
//    public void turnRight(long degrees){
//        backleft.setPower(.7);
//        backright.setPower(-.7);
//        frontleft.setPower(-.7);
//        frontright.setPower(.7);
//        sleep((long) (degrees*(1000/15.1)*2));
//        backleft.setPower(0);
//        backright.setPower(0);
//        frontleft.setPower(0);
//        frontright.setPower(0);
//    }
//    public void armUp(double inches) {
//        armmotor.setPower(.75);
//        sleep((long) (inches*(1000/9.2)*1.5));
//        armmotor.setPower(0);
//    }
//    public void armDown(double inches) {
//        armmotor.setPower(-.75);
//        sleep((long) (inches*(1000/9.2)));
//        armmotor.setPower(0);
//    }

    public void highJunction(){ //SR, F, SR, AU,
        //cone 1
        Claw.setPosition(0.77);
        Forward(48);
        strafeRight(17);
        armRunUp(33);
        Forward(8);
        sleep(300);
        Claw.setPosition(1);
        sleep(100);
        Backward(4);
        //cone 2
        turnCounterClockwise(86);
        Forward (40);
        armRunDown(29);
        Claw.setPosition(0.77);
        armRunUp(6);
        Claw.setPosition(.75);
        Backward(36);
        turnClockwise(95);
        armRunUp(24);
        Forward(5);
        sleep(300);
        Claw.setPosition(1);
        sleep(100);
        Backward(4);
    }

    public void medJunct(){ //
        //cone 1
        Claw.setPosition(0.77);
        Forward(60);
        Backward(5);
        turnClockwise(178);
        strafeLeft(18);
        armRunUp(23);
        Forward(5);
        sleep(200);
        Claw.setPosition(1);
        sleep(100);
        Backward(5);
        //cone 2
        turnClockwise(85);
        strafeLeft(1.5);
        Forward (42);
        armRunDown(19);
        Claw.setPosition(0.77);
        armRunUp(6);
        Claw.setPosition(.75);
        Backward(39);
        turnCounterClockwise(95);
        Backward(2);
        strafeLeft(2);
        armRunUp(14);
        Forward(5);
        sleep(200);
        Claw.setPosition(1);
        sleep(100);
        Backward(7);
        turnCounterClockwise(6);
    }
    public void smallJunct(){ //CO, F50, R10, up, F5, CO, B4, TL90*, F37, down, B37, TR90*, F5, park
        //cone 1
        Claw.setPosition(0.77);
        Forward(48);
        strafeRight(17);
        armRunUp(15);
        Forward(8);
        sleep(200);
        Claw.setPosition(1);
        sleep(100);
        Backward(4);
        //cone 2
        turnCounterClockwise(86);
        Forward (40);
        armRunDown(8);
        Claw.setPosition(0.77);
        armRunUp(6);
        Claw.setPosition(.75);
        Backward(36);
        turnClockwise(95);
        Forward(5);
        sleep(200);
        Claw.setPosition(1);
        sleep(100);
        Backward(4);
        //cone 3
        turnCounterClockwise(90);
        Forward (40);
        armRunDown(5);
        Claw.setPosition(0.77);
        armRunUp(6);
        Claw.setPosition(.75);
        Backward(36);
        turnClockwise(95);
        Forward(5);
        sleep(200);
        Claw.setPosition(1);
        sleep(100);
        Backward(4);
    }
    public void blueRight(){
        Claw.setPosition(0.8);
        Forward(51);
        strafeLeft(15);
        armRunUp(33);
        Forward(5.5);
        sleep(200);
        Claw.setPosition(1);
        sleep(100);
        Backward(8);
        //cone 2
        turnClockwise(95);
        Forward (41);
        armRunDown(29);
        Claw.setPosition(0.85);
        armRunUp(6);
        Claw.setPosition(.77);
        Backward(30);
        turnCounterClockwise(140);
        armRunUp(24);
        Forward(3);
        sleep(200);
        Claw.setPosition(1);
        sleep(100);
        Backward(9);
        turnClockwise(45);
    }


    public int inToTicks(double distance_in) {
        double doubleticks = (distance_in * (1440/(3.89827 * 3.14))) * (48/47);
        int ticksint = (int) Math.round(doubleticks);
        return ticksint;
    }

    public int degToTicks(int degrees) {
        double Ddoubleticks = (degrees/360.0 * (30000/3.55));
        int ticksint = (int) Math.round(Ddoubleticks);
        telemetry.addData("here ticksint", ticksint);
        telemetry.update();
        return ticksint;
    }

    public void Forward(double inches) {
        backleft.setTargetPosition(-inToTicks(inches));
        backright.setTargetPosition(-inToTicks(inches));
        frontleft.setTargetPosition(inToTicks(inches));
        frontright.setTargetPosition(inToTicks(inches));
        Run_to_position();
    }

    public void strafeLeft(double inches) {
        backleft.setTargetPosition(inToTicks(inches));
        backright.setTargetPosition(-inToTicks(inches));
        frontleft.setTargetPosition(-inToTicks(inches));
        frontright.setTargetPosition(inToTicks(inches));
        Run_to_position();
    }

    public void Backward(double inches) {
        backleft.setTargetPosition(inToTicks(inches));
        backright.setTargetPosition(inToTicks(inches));
        frontleft.setTargetPosition(-inToTicks(inches));
        frontright.setTargetPosition(-inToTicks(inches));
        Run_to_position();
    }

    public void strafeRight(double inches) {
        backleft.setTargetPosition(-inToTicks(inches));
        backright.setTargetPosition(inToTicks(inches));
        frontleft.setTargetPosition(inToTicks(inches));
        frontright.setTargetPosition(-inToTicks(inches));
        Run_to_position();
    }

    public void turnClockwise(int degrees) {
        backleft.setTargetPosition(degToTicks(degrees));
        backright.setTargetPosition(-degToTicks(degrees));
        frontleft.setTargetPosition(degToTicks(degrees));
        frontright.setTargetPosition(-degToTicks(degrees));
        Run_to_position();
    }

    public void turnCounterClockwise(int degrees) {
        backleft.setTargetPosition(-degToTicks(degrees));
        backright.setTargetPosition(degToTicks(degrees));
        frontleft.setTargetPosition(-degToTicks(degrees));
        frontright.setTargetPosition(degToTicks(degrees));
        Run_to_position();
    }

    public void Arm_Reset() {
        armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotor.setTargetPosition(0);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armmotor.setPower(0);
    }

    public void  armRunUp (double inches) {
        armmotor.setTargetPosition(inToTicks(inches)*2);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armmotor.setPower(1);
        while (armmotor.isBusy() && opModeIsActive()) {
        }
        Arm_Reset();
//        armmotor.setPower(1);
//        sleep(5500);
//        armmotor.setPower(0);
    }
    public void  armRunDown (double inches) {
        armmotor.setTargetPosition(-inToTicks(inches)*2);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armmotor.setPower(1);
        while (armmotor.isBusy() && opModeIsActive()) {
        }
        Arm_Reset();
//        armmotor.setPower(-1);
//        sleep(5500);
//        armmotor.setPower(0);
    }

    public void Run_to_position() {
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setPower(1);
        backright.setPower(1);
        frontleft.setPower(1);
        frontright.setPower(1);
        while (backleft.isBusy() && backright.isBusy() && frontleft.isBusy() && frontright.isBusy() && opModeIsActive()) {
            telemetry.addData("Motor ticks", backleft.getCurrentPosition());
            telemetry.addData("Motor ticks", backright.getCurrentPosition());
            telemetry.addData("Motor ticks", frontleft.getCurrentPosition());
            telemetry.addData("Motor ticks", frontright.getCurrentPosition());
            telemetry.update();
        }
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
        Stop_and_reset();
    }

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

