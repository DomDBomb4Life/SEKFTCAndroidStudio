package org.firstinspires.ftc.teamcode.Vuforia;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for a three wheel omni-bot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left drive"
 * Motor channel:  Right drive motor:        "right drive"
 * Motor channel:  Rear  drive motor:        "back drive"
 *
 * These motors correspond to three drive locations spaced 120 degrees around a circular robot.
 * Each motor is attached to an omni-wheel. Two wheels are in front, and one is at the rear of the robot.
 *
 * Robot motion is defined in three different axis motions:
 * - Axial    Forward/Backwards      +ve = Forward
 * - Lateral  Side to Side strafing  +ve = Right
 * - Yaw      Rotating               +ve = CCW
 */


public class Robot_OmniDrive
{
    // Private Members
    private LinearOpMode myOpMode;
    private DcMotor frontLDrive= null;
    private DcMotor backLDrive= null;
    private DcMotor frontRDrive=null;
    private DcMotor backRDrive=null;

   /* private DcMotor  leftDrive      = null;
    private DcMotor  rightDrive     = null;
    private DcMotor  backDrive      = null;*/

    /*private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW*/
    double driveLeftStickY=0;
    double driveLeftStickX=0;
    double drivePivot=0;


    /* Constructor */
    public Robot_OmniDrive(){

    }


    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {

        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        /*front        = myOpMode.hardwareMap.get(DcMotor.class, "left drive");
        rightDrive       = myOpMode.hardwareMap.get(DcMotor.class, "right drive");
        backDrive        = myOpMode.hardwareMap.get(DcMotor.class, "back drive");*/
        frontLDrive = myOpMode.hardwareMap.get(DcMotor.class, "FrontL");
        backLDrive = myOpMode.hardwareMap.get(DcMotor.class, "BackL");
        frontRDrive = myOpMode.hardwareMap.get(DcMotor.class, "FrontR");
        backRDrive = myOpMode.hardwareMap.get(DcMotor.class, "BackR");

        /*leftDrive.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Positive input rotates counter clockwise
        backDrive.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise*/

        //use RUN_USING_ENCODERS because encoders are installed.
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop all robot motion by setting each axis value to zero
        moveRobot(0, 0, 0) ;
    }

    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        /*setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);*/
        setLeftStickY(myOpMode.gamepad1.left_stick_y);
        setLeftStickX(myOpMode.gamepad1.left_stick_x);
        setPivot(myOpMode.gamepad1.right_stick_x);
    }



    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param lY     Speed in Fwd Direction
     * @param lX   Speed in lateral direction (+ve to right)
     * @param p       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double lY, double lX, double p) {
        setLeftStickY(lY);
        setLeftStickX(lX);
        setPivot(p);
        moveRobot();
    }

    /*
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
        double backR = -drivePivot + (driveLeftStickY - driveLeftStickX);
        double backL = -drivePivot + driveLeftStickY + driveLeftStickX;
        double frontR = drivePivot + driveLeftStickY + driveLeftStickX;
        double frontL = drivePivot + (driveLeftStickY - driveLeftStickX);

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(backR), Math.abs(backL));
        max = Math.max(max, Math.abs(frontR));
        max = Math.max(max, Math.abs(frontL));
        if (max > 1.0)
        {
            frontR /= max;
            frontL /= max;
            backR /= max;
            backL /= max;

        }

        // Set drive motor power levels.
        frontRDrive.setPower(frontR);
        frontLDrive.setPower(frontL);
        backRDrive.setPower(backR);
        backLDrive.setPower(backL);



        // Display Telemetry
        myOpMode.telemetry.addData("Contols", "LY[%+5.2f], LX[%+5.2f], P[%+5.2f]", driveLeftStickY, driveLeftStickX, drivePivot);
        myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], B[%+5.2f]", frontR, frontL, backR, backL);
    }


    public void setLeftStickY(double lY)      {driveLeftStickY = Range.clip(lY, -1, 1);}
    public void setLeftStickX(double lX)  {driveLeftStickX = Range.clip(lX, -1, 1); }
    public void setPivot(double p)          {drivePivot = Range.clip(p, -1, 1); }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        backRDrive.setMode(mode);
        backLDrive.setMode(mode);
        frontRDrive.setMode(mode);
        frontLDrive.setMode(mode);
    }
}

