package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        double liftlevelintake = 0;
        double liftlevellow = 0;
        double liftlevelmid = 0;
        double liftlevelhigh = 0;
        Claw = hardwareMap.get(Servo.class, "Claw");
        armmotor = hardwareMap.get(DcMotor.class, "arm motor");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        backright = hardwareMap.get(DcMotor.class, "back right");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        // Put initialization blocks here.
        Claw.setPosition(1);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral =  gamepad1.right_stick_x;
                double yaw     =  gamepad1.left_stick_x;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }
                // Send calculated power to wheels
                frontleft.setPower(leftFrontPower);
                frontright.setPower(rightFrontPower);
                backleft.setPower(-leftBackPower);
                backright.setPower(-rightBackPower);
                armdown = gamepad1.left_trigger;
                armup = gamepad1.right_trigger;

                armmotor.setPower(armdown - armup);
                if (gamepad1.left_bumper) {
                    Claw.setPosition(0.85);
                }
                if (gamepad1.right_bumper) {
                    Claw.setPosition(1);
                }
                telemetry.addData("FL",frontleft.getCurrentPosition()); //   For testing if encoders are working
                telemetry.addData("BL",backleft.getCurrentPosition());
                telemetry.addData("FR",frontright.getCurrentPosition());
                telemetry.addData("BR",backright.getCurrentPosition());
                telemetry.addData("Lift Encoders", armmotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
