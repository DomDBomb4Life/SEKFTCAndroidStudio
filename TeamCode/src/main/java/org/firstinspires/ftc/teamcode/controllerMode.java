package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "controllerMode (Lift)")
public class controllerMode extends LinearOpMode {

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double ServoPosition;
        double ServoSpeed;
        float leftStickY;
        float leftStickX;
        float pivot;

        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
        DcMotor frontL = hardwareMap.get(DcMotor.class, "FrontL");
        DcMotor backL = hardwareMap.get(DcMotor.class, "BackL");
        DcMotor frontR = hardwareMap.get(DcMotor.class, "FrontR");
        DcMotor backR = hardwareMap.get(DcMotor.class, "BackR");
        Servo left_hand = hardwareMap.get(Servo.class, "left_hand");

        // Set servo to mid position
        ServoPosition = 0.06;
        ServoSpeed = 0.05;
        frontL.setDirection(DcMotorSimple.Direction.REVERSE);
        backL.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Put run blocks here.
                leftStickY = -gamepad1.left_stick_y;
                leftStickX = gamepad1.left_stick_x;
                pivot = gamepad1.right_stick_x;
                frontR.setPower(-pivot + (leftStickY - leftStickX));
                backR.setPower(-pivot + leftStickY + leftStickX);
                frontL.setPower(pivot + leftStickY + leftStickX);
                backL.setPower(pivot + (leftStickY - leftStickX));
                telemetry.update();
                // Use bumpers to open/close servo
                if (gamepad1.right_bumper) {
                    ServoPosition += ServoSpeed;
                }
                if (gamepad1.left_bumper) {
                    ServoPosition -= ServoSpeed;
                }
                // Keep Servo position in valid range
                ServoPosition = Math.min(Math.max(ServoPosition, 0.06), 0.4);
                left_hand.setPosition(ServoPosition);

                // use up & down to lift
                if (gamepad1.dpad_down){
                    lift.setPower(-1);
                }
                if (gamepad1.dpad_up){
                    lift.setPower(1);
                }
                telemetry.update();
                sleep(20);
            }
        }
    }
}