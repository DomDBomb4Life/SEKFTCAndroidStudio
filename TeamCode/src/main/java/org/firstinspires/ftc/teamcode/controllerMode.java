package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "controllerMode (Blocks to Java)")
public class controllerMode extends LinearOpMode {

    private DcMotor lift;
    private DcMotor FrontL;
    private DcMotor BackL;
    private DcMotor FrontR;
    private DcMotor BackR;
    private Servo left_hand;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double ServoPosition;
        double ServoSpeed;
        float leftStickY;
        float leftStickx;
        float pivot;

        lift = hardwareMap.get(DcMotor.class, "lift");
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        left_hand = hardwareMap.get(Servo.class, "left_hand");

        // Set servo to mid position
        ServoPosition = 0.06;
        ServoSpeed = 0.05;
        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Put run blocks here.
                leftStickY = -gamepad1.left_stick_y;
                leftStickx = gamepad1.left_stick_x;
                pivot = gamepad1.right_stick_x;
                FrontR.setPower(-pivot + (leftStickY - leftStickx));
                BackR.setPower(-pivot + leftStickY + leftStickx);
                FrontL.setPower(pivot + leftStickY + leftStickx);
                BackL.setPower(pivot + (leftStickY - leftStickx));
                telemetry.update();
                // Use bumpers to open/close servo
                if (gamepad1.right_bumper) {
                    ServoPosition += ServoSpeed;
                }
                if (gamepad1.left_bumper) {
                    ServoPosition += -ServoSpeed;
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