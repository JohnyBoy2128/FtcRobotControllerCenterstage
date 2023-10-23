package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CS: Outreach TeleOp", group="Linear Opmode")
//@Disabled
public class CenterstageOutreachMainTeleOp extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;


    // the power of the motors are multiplied by this
    double motorPowerFactor = 1;


    // gamepad setup, and copies of the gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();


    @Override
    public void runOpMode() { //---------------PRESSES INITIALIZE---------------

        // initialize the motors
        initMotors();

        // adds telemetry that the robot has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ////////////////////////////////////////////////////////////////////////////////////////////

        // resets the runtime
        runtime.reset();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) { //---------------PRESSES PLAY---------------

            // make a copy of the gamepad
            copyGamepad();

            // does the drive motor stuff
            calcMotorPowerFactor();
            updateDriveMotors();


            // do telemetry
            doTelem();
        }
    }

    // initialize the motors
    public void initMotors() {

        // initialize the motor hardware variables
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");

        // reverses some of the motor directions
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        // use braking to slow the drive motors down faster
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // makes copies of the gamepad
    public void copyGamepad() {
        try {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1 to be used in this loop iteration
            previousGamepad1.copy(currentGamepad1);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1 to be used for the entirety of this loop iteration
            currentGamepad1.copy(gamepad1);
        }
        catch (Exception e) {
            // Swallow the possible exception, it should not happen as
            // currentGamepad1 are being copied from valid Gamepads
        }
    }

    // calculates/updates motorPowerFactor
    public void calcMotorPowerFactor() {
        // rising edge detector for dpad_up; increases motorPowerFactor
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            motorPowerFactor = Math.min(motorPowerFactor + .1, 1);
        }

        // rising edge detector for dpad_down; decreases motorPowerFactor
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            motorPowerFactor = Math.max(motorPowerFactor - .1, 0.1);
        }
    }

    // calculates and updates the power of the drive motors
    public void updateDriveMotors() {

        // gamepad inputs
        double lsy = -gamepad1.left_stick_y; // Remember, this is reversed!
        double rsx = gamepad1.right_stick_x;

        // disregards lsy if its absolute value is less than 0.1
        if (Math.abs(lsy) < 0.1) {
            lsy = 0.0;
        }

        // gets the signs of the stick values
        double lsySign = lsy / Math.abs(lsy);
        double rsxSign = rsx / Math.abs(rsx);

        // ensures the stick value signs aren't NaN
        if (Double.isNaN(lsySign)) {lsySign = 0;}
        if (Double.isNaN(rsxSign)) {rsxSign = 0;}

        // joystick values used to determine drive movement
        // they're squared to allow for finer control at low speeds
        double y = Math.pow(lsy, 2) * lsySign;
        double rx = Math.pow(rsx, 2) * rsxSign;

        // set the power variables for left and right side
        // motors to the left stick +/- right stick
        double leftPower    = Range.clip(y + rx, -1.0, 1.0) ;
        double rightPower   = Range.clip(y - rx, -1.0, 1.0) ;

        // Send calculated power to wheels
        motorFR.setPower(rightPower);
        motorFL.setPower(leftPower);
        motorBL.setPower(leftPower);
        motorBR.setPower(rightPower);

        //add motor power telemetry
        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);

        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
    }

    public void planeLauncher() {
        String controller = gamepad1.toString().toLowerCase();
        telemetry.addData("Gamepad Status", controller);
        telemetry.update();

        double power = 0;

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            power = 0;
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            power = 1;
        }

        motorFR.setPower(power);
        //ID: xxxx user: 1 lx: 0.00ly: 0.00rx: 0.00ry: 0.00 rt:0.00
    }

    // does the telemetry
    public void doTelem() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("motorPowerFactor", motorPowerFactor);
        telemetry.update();
    }
}
