package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "Sample", preselectTeleOp = "TeleOp")
public class Sample extends LinearOpMode {

    private DcMotorEx frontleft;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx frontright;
    private DcMotorEx armmotor;
    private Servo Claw;

    @Override
    public void runOpMode() {
        frontleft = hardwareMap.get(DcMotorEx.class, "front left");
        backleft = hardwareMap.get(DcMotorEx.class, "back left");
        backright = hardwareMap.get(DcMotorEx.class, "back right");
        frontright = hardwareMap.get(DcMotorEx.class, "front right");
        armmotor = hardwareMap.get(DcMotorEx.class, "arm motor");
        Claw = hardwareMap.get(Servo.class, "Claw");

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        armmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int team = 0;
        int side = 0;

        while(!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                telemetry.addData("team", "red");
                telemetry.addData("side", "left");
                telemetry.update();
                team = 0;
                side = 0;
            } else if (gamepad1.x) {
                telemetry.addData("team", "red");
                telemetry.addData("side", "right");
                telemetry.update();
                team = 0;
                side = 1;
            } else if (gamepad1.b) {
                telemetry.addData("team", "blue");
                telemetry.addData("side", "left");
                telemetry.update();
                team = 1;
                side = 0;
            } else if (gamepad1.y) {
                telemetry.addData("team", "blue");
                telemetry.addData("side", "right");
                telemetry.update();
                team = 1;
                side = 1;
            }
        }

        if (opModeIsActive()) {
            //Stop_and_reset();
            if (team == 0){ //red
                if (side==0){ //left
                        redLeft();
                        strafeRight(9);
                        armDown(35);
                    }

                else{ //right
                        redRight();
                        strafeRight(18);
                        armDown(35);
                }
            }
            else if (team==1){ //blue
                if(side==0){ //left
                        blueLeft();
                        strafeRight(9);
                        armDown(35);
                    }
                else{ //right
                        blueRight();
                        strafeRight(9);
                        armDown(35);
                    }
                }
            }
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
        Forward(41);
        strafeRight(10);
        armUp(32.5);
        Forward(3);
        sleep(700);
        Claw.setPosition(1);
        sleep(500);
        Backward(8);
    }
    public void redRight(){
        Claw.setPosition(0.85);
        Forward(40);
        strafeLeft(4);
        armUp(32);
        Forward(4);
        sleep(700);
        Claw.setPosition(1);
        sleep(700);
        Backward(5);
        sleep (500);
    }
    public void blueRight(){
        Claw.setPosition(0.85);
        Forward(44);
        strafeLeft(4);
        armUp(32);
        Forward(4);
        sleep(700);
        Claw.setPosition(1);
        sleep(700);
        Backward(5);
        sleep(500);
    }
}
