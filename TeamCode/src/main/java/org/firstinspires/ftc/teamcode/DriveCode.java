package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "DriveCode")
public class DriveCode extends LinearOpMode {

    private Servo Claw;
    private DcMotor armmotor;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor frontright;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float y;
        float x;
        double rx;
        float armdown;
        float armup;
        double denominator;

        Claw = hardwareMap.get(Servo.class, "Claw");
        armmotor = hardwareMap.get(DcMotor.class, "arm motor");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");

        // Put initialization blocks here.
        Claw.setPosition(1);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                y = -gamepad1.right_stick_x;
                x = gamepad1.left_stick_y;
                rx = gamepad1.left_stick_x * 1.1;
                armdown = gamepad1.left_trigger;
                armup = gamepad1.right_trigger;
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), null, 1));
                // Creates levels of speed based on how far the
                // joysticks are from the center (orgin... or 0,0)
                backleft.setPower((y + x + rx) / denominator);
                backright.setPower(((y - x) + rx) / denominator);
                frontleft.setPower(((y + x) - rx) / denominator);
                frontright.setPower(((y - x) - rx) / denominator);
                armmotor.setPower(armdown - armup);
                // You can change the position numbers
                // Based on what you want the servo to do.
                if (gamepad1.left_bumper) {
                    Claw.setPosition(0.85);
                }
                if (gamepad1.right_bumper) {
                    Claw.setPosition(1);
                }
                telemetry.update();
            }
        }
    }
}