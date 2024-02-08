package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;


@TeleOp(name="CS: 1 Player TeleOp", group="Linear Opmode")
//@Disabled
public class CenterstageOnePlayerTeleOp extends LinearOpMode {

    // setup class for grabber and arm movements
    protected ScoringMechanism mechanism = new ScoringMechanism();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;


    // the power of the motors are multiplied by this
    double motorPowerFactor = 0.7;


    // copies of the gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() { //---------------PRESSES INITIALIZE---------------
        // initialize the motors and servos
        initMotorsAndServos();

        // closes servos on grabber to hold pixels before running
        mechanism.closeLeftClaw();
        mechanism.closeRightClaw();

        // **************       REMOVE WHEN ADDING AUTONOMOUS           ************
        // lower the pixel pusher to load the purple pixel for autonomous
        mechanism.lowerPixelPusher();
        mechanism.lowerPlaneLauncher();


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

            // updates the grabber
            updateGrabberServos();

            // updates the arm motors
            updateArmMotors();

            // do telemetry
            doTelem();

            // extra methods for calibration
            // calibrateServos();

            ////////            ENABLE FOR SCRIMMAGEEEEEEE                 ///////////
            calibrateMotors();

        }
    }


    // initialize the motors and servos
    public void initMotorsAndServos() {

        // innitialize arm and servo motors
        mechanism.init(hardwareMap, false);

        // initialize the motor hardware variables
        //DRIVE MOTORS
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
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        } catch (Exception e) {
            // Swallow the possible exception, it should not happen as
            // currentGamepad1 are being copied from valid Gamepads
        }

    }

    // calculates/updates motorPowerFactor
    public void calcMotorPowerFactor() {
        // rising edge detector for dpad_up; increases motorPowerFactor
        if (currentGamepad1.back && !previousGamepad1.back) {
            motorPowerFactor = Math.min(motorPowerFactor + .1, 1);
        }

        // rising edge detector for dpad_down; decreases motorPowerFactor
        if (currentGamepad1.start && !previousGamepad1.start) {
            motorPowerFactor = Math.max(motorPowerFactor - .1, 0.1);
        }
    }

    // calculates and updates the power of the drive motors
    public void updateDriveMotors() {
        // gamepad inputs
        double lsy = -currentGamepad1.left_stick_y; // Remember, this is reversed!
        double lsx = currentGamepad1.left_stick_x;
        double rsx = currentGamepad1.right_stick_x;

        double y = gamepadSticksMath(lsy);
        double x = gamepadSticksMath(lsx);
        double rx = gamepadSticksMath(rsx);

        double powerFR = (y - x - rx) * motorPowerFactor;
        double powerFL = (y + x + rx) * motorPowerFactor;
        double powerBL = (y - x + rx) * motorPowerFactor;
        double powerBR = (y + x - rx) * motorPowerFactor;

        // Send calculated power to wheels
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);

        //originally commented
        telemetry.addData("powerFR", powerFR);
        telemetry.addData("powerFL", powerFL);
        telemetry.addData("powerBL", powerBL);
        telemetry.addData("powerBR", powerBR);

        telemetry.addData("y", y);
        telemetry.addData("x", x);
        telemetry.addData("rx", rx);
    }

    // updates the grabber position if in regular servo mode
    public void updateGrabberServos() {

        // opens the left grabber with trigger, closes with more precise bumper button
        if (currentGamepad1.left_trigger > .3) {
            mechanism.openLeftClaw();
        }
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            mechanism.closeLeftClaw();
        }

        // opens the right grabber with trigger, closes with more precise bumper button
        if (currentGamepad1.right_trigger > .3) {
            mechanism.openRightClaw();
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            mechanism.closeRightClaw();
        }

    }

    public void updateArmMotors() {

        if (currentGamepad1.cross && !previousGamepad1.cross) {
            mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR);
        }
        if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button){
            mechanism.moveToLevel(ScoringMechanism.boardLevels.PICKUP);
        }
        if (currentGamepad1.square && !previousGamepad1.square) {
            mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1);
        }
        if (currentGamepad1.circle && !previousGamepad1.circle) {
            mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL2AND3);
        }
        if (currentGamepad1.triangle && !previousGamepad1.triangle) {
            mechanism.launchPlane();
        }
        if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
            mechanism.moveToLevel(ScoringMechanism.boardLevels.LIFTTOHANG);
        }
        /*
        if ( /* make some sort of button mapping *\ ) {
            mechanism.moveToLevel(ScoringMechanism.boardLevels.PULLUP);
        }
        */
    }

    // method to do all the math for the gamepad stick
    public double gamepadSticksMath(double stick) {


        // disregards lsy or lsx if their absolute value is less than 0.1
        if (Math.abs(stick) < 0.1) {
            stick = 0.0;

        }

        // gets the signs of the stick values
        double stickSign = stick / Math.abs(stick);

        // ensures the stick value signs aren't NaN
        if (Double.isNaN(stickSign)) {
            stickSign = 0;
        }

        // joystick values used to determine drive movement
        // they're squared to allow for finer control at low speeds
        double power = Math.pow(stick, 2) * stickSign;

        // return the calculated value
        return power;
    }
    // does the telemetry
    public void doTelem() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Parallel encoder position: ", motorFR.getCurrentPosition());
        telemetry.addData("SPEED!!!", motorPowerFactor * 100);
        telemetry.addData("Arm Position", mechanism.getArmPosition());
        telemetry.addData("Actuator Position", mechanism.getActuatorPosition());
        telemetry.update();
    }


    public void calibrateMotors() {

        // moves actuator motors by 1 rotation, and arm motors by 100 ticks
        if (currentGamepad1.dpad_up) {
            // right bumper pressed, increase motor position
            mechanism.extendActuator();
        }
        if (currentGamepad1.dpad_down) {
            // left bumper pressed, decrease servo position
            mechanism.retractActuator();
        }
        if (currentGamepad1.dpad_left) {
            // circle pressed, increase motor position
            mechanism.moveArmBack();
        }
        if (currentGamepad1.dpad_right) {
            // left bumper pressed, decrease servo position
            mechanism.moveArmForward();
        }

    }

    // cannot be used with this class, unless changing gp1 to gp2
    public void calibrateServos() {

        // moves actuator motors by 1 rotation, and arm motors by 100 ticks
        if (currentGamepad2.right_bumper) {
            // right bumper pressed, increase motor position
            mechanism.moveServoPlusLittle();
        }
        if (currentGamepad2.left_bumper) {
            // left bumper pressed, decrease servo position
            mechanism.moveServoMinusLittle();
        }
        if (currentGamepad2.dpad_right) {
            // circle pressed, increase motor position
            mechanism.lowerPlaneLauncher();
        }
        if (currentGamepad2.dpad_left) {
            // left bumper pressed, decrease servo position
            mechanism.launchPlane();
        }

    }



}
