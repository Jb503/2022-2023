package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

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
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        int index;
        int inToTicks = 119;
        int team = 0;
        int side = 0;
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
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

        Stop_and_reset();
        Arm_Reset();
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
        }
        waitForStart();

        // Sample TFOD Op Mode using a Custom Model
        // The following block uses a webcam.
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Initialize TFOD before waitForStart.
        // Use the Manage page to upload your custom model.
        // In the next block, replace
        // YourCustomModel.tflite with the name of your
        // custom model.
        // Set isModelTensorFlow2 to true if you used a TensorFlow
        // 2 tool, such as ftc-ml, to create the model.
        //
        // Set isModelQuantized to true if the model is
        // quantized. Models created with ftc-ml are quantized.
        //
        // Set inputSize to the image size corresponding to the model.
        // If your model is based on SSD MobileNet v2
        // 320x320, the image size is 300 (srsly!).
        // If your model is based on SSD MobileNet V2 FPNLite 320x320, the image size is 320.
        // If your model is based on SSD MobileNet V1 FPN 640x640 or
        // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
        tfod.useModelFromFile("model_20221201_133353.tflite", JavaUtil.createListWith("1Dot", "2Dots", "3Dots"), true, true, 300);
        // If the robot isn't detecting/detecting something
        // that isn't there, then play with minConfidence
        // Lower=false positives, Higher=false negatives
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
        tfod.setClippingMargins(0, 80, 0, 0);
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(1.5, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        // Wait for start command from Driver Station.
        waitForStart();
        if (opModeIsActive()) {
            Stop_and_reset();
            if (team == 0){ //red
                if (side==0){ //left
                    //Forward(53);
                    //strafeLeft(12);
                    //Forward(4);
                    armmotor.setPower(1);
                    sleep(3000);
                    armmotor.setPower(1);
                }
                else{ //right

                }
            }
            else if (team==1){ //blue
                if(side==0){ //left

                }
                else{ //right

                }
            }
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Get a list of recognitions from TFOD.
                recognitions = tfod.getRecognitions();
                // Creating a variable to link camera recognition
                // to code I can actually use... hopefully
                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (JavaUtil.listLength(recognitions) == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                } else {
                    index = 0;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                    }
                    // Goes to the purple Action function
                    // So that I can separate based on
                    // The task the robot is doing
                    Actions();
                }
                telemetry.update();
            }
        }
        // Deactivate TFOD.
        tfod.deactivate();

        vuforiaPOWERPLAY.close();
        tfod.close();

    }

    public void Actions() {

        if (recognition.getLabel().equals("1Dot")) {

        } else if (recognition.getLabel().equals("2Dots")) {

        } else if (recognition.getLabel().equals("3Dots")) {

        } else {
            telemetry.update();
        }

    }

    /**
     * Describe this function...
     */
    public void displayInfo(int i) {
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Label: " + recognition.getLabel() + ", Confidence: " + recognition.getConfidence(), "X: " + Math.round(JavaUtil.averageOfList(JavaUtil.createListWith(Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)), Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0))))) + ", Y: " + Math.round(JavaUtil.averageOfList(JavaUtil.createListWith(Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)), Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0))))));
    }

    /**
     * Describe this function...
     */
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
        Arm_Reset();
    }

    public void Forward(long inches) {
        backleft.setPower(1);
        backright.setPower(.9);
        frontleft.setPower(1);
        frontright.setPower(.9);
        sleep((long) (inches*(1000 / 19.2 )));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void Backward(long inches) {
        backleft.setPower(-0.5);
        backright.setPower(-0.5);
        frontleft.setPower(-0.5);
        frontright.setPower(-0.5);
        sleep((long) (inches*(1000 / 19.2 )));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void strafeRight(long inches) {
        backleft.setPower(-1);
        backright.setPower(1);
        frontleft.setPower(1);
        frontright.setPower(-1);
        sleep((long) (inches*(1000/15.1)));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void strafeLeft(long inches) {
        backleft.setPower(.8);
        backright.setPower(-1);
        frontleft.setPower(-1);
        frontright.setPower(.8);
        sleep((long) (inches * (1000 / 15.1)));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }
    public void armUp(long inches) {
        armmotor.setPower(1);
        sleep((long) (inches*(1000/0)));
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
}
