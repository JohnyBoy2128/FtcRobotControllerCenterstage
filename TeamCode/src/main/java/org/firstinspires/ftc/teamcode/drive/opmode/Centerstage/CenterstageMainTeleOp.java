package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CS: 1 Driver TeleOp", group="Linear Opmode")
//@Disabled
public class CenterstageMainTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;
    private DcMotor motorActuator = null;
    private DcMotor motorWormGear = null;
    private Servo servoGrabberL = null;
    private Servo servoGrabberR = null;
    private Servo servoRotatorL = null;
    private Servo servoRotatorR = null;


    // the power of the motors are multiplied by this
    double motorPowerFactor = 1;

    // variables for servo positions
    double positionL = 0;
    double positionR = 0;



    // how many encoder tics make one full actuator motor rotation
    static final double ARM_TICS_IN_ROT = 537.7;
    // the number of mm the slides move from one motor rotation
    static final double ARM_MM_FROM_ROT = 8.3;
    // number of tics to move slide by 1cm
    static final double ARM_TICS_IN_CM = ARM_TICS_IN_ROT / (ARM_MM_FROM_ROT / 10);

    // the slide's level; 0-3, 0 being ground, 3 being highest pole
    int slideLvl = 0;




    // copies of the gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() { //---------------PRESSES INITIALIZE---------------
        // initialize the motors and servos
        initMotorsAndServos();

        // adds telemetry that the robot has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ////////////////////////////////////////////////////////////////////////////////////////////

        // resets the runtime
        runtime.reset();

        // enables pwm for the grabber servos

        servoGrabberL.getController().pwmEnable();
        servoGrabberR.getController().pwmEnable();


        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) { //---------------PRESSES PLAY---------------
            // make a copy of the gamepad
            copyGamepad();

            // does the drive motor stuff
            calcMotorPowerFactor();
            //updateDriveMotors();

            // updates the grabber
            //updateGrabberServos();

            calibrateServos();


            // updates the arm motors
            updateArmMotors();

            // do telemetry
            doTelem();

        }
    }

    // method to find out servo directions and such
    public void calibrateServos() {

        //      CALIBRATION METHOD
        // method to find out servo directions and such


        if (gamepad1.triangle && !previousGamepad1.triangle) {
            servoRotatorL.setPosition(servoRotatorL.getPosition() - 0.01);      // Clockwise
        }
        if (gamepad1.cross && !previousGamepad1.cross) {
            servoRotatorL.setPosition(servoRotatorL.getPosition() + 0.01);      // Counterclockwise
        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
            servoRotatorR.setPosition(servoRotatorR.getPosition() + 0.01);      // Counterclockwise
        }
        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            servoRotatorR.setPosition(servoRotatorR.getPosition() - 0.01);      // Clockwise
        }

        // string containing each servos position
         telemetry.addData("rotatorPosition", "L: " + Double.toString(servoRotatorL.getPosition()) + ",  R: " + Double.toString(servoRotatorR.getPosition()));

    }

    public void calibrateMotors() {



    }

    //CODE BELOW IS FINAL


    // initialize the motors and servos
    public void initMotorsAndServos() {
        // initialize the motor hardware variables
                //DRIVE MOTORS
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");

                //ARM MOTORS
        motorActuator = hardwareMap.get(DcMotor.class, "actuator");
        motorWormGear = hardwareMap.get(DcMotor.class, "wormy");

                //GRABBER SERVOS
        servoGrabberL = hardwareMap.get(Servo.class, "grabberL");
        servoGrabberR = hardwareMap.get(Servo.class, "grabberR");
        servoRotatorL = hardwareMap.get(Servo.class, "rotatorL");
        servoRotatorR = hardwareMap.get(Servo.class, "rotatorR");




        // reverses some of the motor directions
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        // use braking to slow the drive motors down faster
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        /* MAY USE THIS FOR ARM MOTORS/SERVOS

        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize the grabber servos variable
        grabberL = hardwareMap.get(Servo.class, "grabberL");
        grabberR = hardwareMap.get(Servo.class, "grabberR");

         */
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
        double lsx = gamepad1.left_stick_x;
        double rsx = gamepad1.right_stick_x;

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


        // Rising edge detector for right bumper.
        // This moves to the closed position.
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            servoGrabberL.setPosition(0.16); // smaller = closed
            servoGrabberR.setPosition(0.93); // larger = closed
        }

        // Rising edge detector for left bumper.
        // This moves to the open position.
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            servoGrabberL.setPosition(0.29); // smaller = closed
            servoGrabberR.setPosition(0.79); // larger = closed
        }



        // Code used to calibrate the servo positions for open and closed
        /*
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            // right bumper pressed, increase servo position
            servoGrabberL.setPosition(servoGrabberL.getPosition() + 0.01);
            servoGrabberR.setPosition(servoGrabberR.getPosition() + 0.01);
        }
        else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            // left bumper pressed, decrease servo position
            servoGrabberL.setPosition(servoGrabberL.getPosition() - 0.01);
            servoGrabberR.setPosition(servoGrabberR.getPosition() - 0.01);
        }

         */

    }

    /*
    // updates the grabber position if in continuous rotation mode
    public void updateGrabberContinuousMode() {
        if (currentGamepad1.right_bumper && !currentGamepad1.left_bumper) {
            // makes the wheels spin inward
            grabberL.setPosition(-10);
            grabberR.setPosition(10);

            // starts the wheels moving
            grabberL.getController().pwmEnable();
            grabberR.getController().pwmEnable();
        } else if (currentGamepad1.left_bumper && !currentGamepad1.right_bumper) {
            // makes the wheels spin outward
            grabberL.setPosition(10);
            grabberR.setPosition(-10);

            // starts the wheels moving
            grabberL.getController().pwmEnable();
            grabberR.getController().pwmEnable();
        } else {
            // stops the wheels from turning
            grabberL.getController().pwmDisable();
            grabberR.getController().pwmDisable();
        }
    }

    // move the slides a number of cm
    public void moveSlidesToLvl(int newSlideLvl) {
        int newHeight = 0;

        if (newSlideLvl == 1) {
            newHeight = (int)(36 * SLIDE_TICS_IN_CM);
        } else if (newSlideLvl == 2) {
            newHeight = (int)(62 * SLIDE_TICS_IN_CM);
        } else if (newSlideLvl == 3) {
            newHeight = (int)(87 * SLIDE_TICS_IN_CM);
        } else if (newSlideLvl == 4) {
            newHeight = (int)(3.25 * SLIDE_TICS_IN_CM);
        }

        // moves the slides to the desired position
        slideL.setTargetPosition(newHeight);
        slideR.setTargetPosition(newHeight);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (newSlideLvl < slideLvl) {
            // if the slides are moving downward...
            slideL.setPower(0.7);
            slideR.setPower(0.7);
        } else {
            // if the slides are moving upward or are the same level...
            slideL.setPower(1);
            slideR.setPower(1);
        }

        // set the slideLvl to be what the slides have just moved to
        slideLvl = newSlideLvl;
    }

    // updates the slides level
    public void updateSlidesLvl() {
        // decides power of the slides
        if (currentGamepad1.a && !previousGamepad1.a) {
            moveSlidesToLvl(0);
        } else if (currentGamepad1.b && !previousGamepad1.b) {
            moveSlidesToLvl(1);
        } else if (currentGamepad1.y && !previousGamepad1.y) {
            moveSlidesToLvl(2);
        } else if (currentGamepad1.x && !previousGamepad1.x) {
            moveSlidesToLvl(3);
        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            moveSlidesToLvl(4);
        }
    }

     */

    // updates the actuator and the worm gear to move the arm out and around
    public void updateArmMotors() {

        int currentActuatorPosition = motorActuator.getCurrentPosition();

        // limits for top and bottom of actuator
        int bottomActuatorLimit = 0;  // Replace with your desired bottom limit
        int topActuatorLimit = 13000;  // Replace with your desired top limit

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            if (currentActuatorPosition < topActuatorLimit - ARM_TICS_IN_ROT) {
                // right bumper pressed, increase motor position
                motorActuator.setTargetPosition(currentActuatorPosition + 538);
                motorActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorActuator.setPower(1);
            }
        }
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            if (currentActuatorPosition > bottomActuatorLimit + ARM_TICS_IN_ROT) {
                // left bumper pressed, decrease servo position
                motorActuator.setTargetPosition(currentActuatorPosition - 538);
                motorActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorActuator.setPower(1);
            }
        }

        telemetry.addData("currentActuatorPosition", currentActuatorPosition);




        int currentArmPosition = motorWormGear.getCurrentPosition();

        // limits for top and bottom of worm gear
        int bottomArmLimit = 0;  // Replace with your desired bottom limit
        int topArmLimit = 1000;  // Replace with your desired top limit

        if (currentGamepad2.circle && !previousGamepad2.circle) {
            if (currentArmPosition < topArmLimit - 99) {
                // circle pressed, increase motor position
                motorWormGear.setTargetPosition(currentArmPosition + 100);
                motorWormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorWormGear.setPower(.5);
            }
        }
        if (currentGamepad2.square && !previousGamepad2.square) {
            if (currentArmPosition > bottomArmLimit + 99) {
                // left bumper pressed, decrease servo position
                motorWormGear.setTargetPosition(currentArmPosition - 100);
                motorWormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorWormGear.setPower(.5);
            }
        }

        telemetry.addData("currentArmPosition", currentArmPosition);




        /*
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            // right bumper pressed, increase motor position
            motorActuator.setTargetPosition(currentArmPosition + 10);
        }
        else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            // left bumper pressed, decrease servo position
            motorActuator.setTargetPosition(currentArmPosition - 10);
        }
        /*   CODE FOR WHEN we find out positons
        while (opModeIsActive()) {
            // Assuming you have a method to get the current position from the encoder
            currentPosition = motorActuator.getCurrentPosition();

            if (currentPosition <= bottomLimit) {
                // Stop the motor and prevent it from moving down further
                motorActuator.setPower(0);
            } else if (currentPosition >= topLimit) {
                // Stop the motor and prevent it from moving up further
                motorActuator.setPower(0);
            } else {
                // No limit reached, continue normal motor control
                linearMotor.setPower(gamepad1.left_stick_y);
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
        /*

        telemetry.addData("slideLvl", slideLvl);
        telemetry.addData("slideL position", slideL.getCurrentPosition());
        telemetry.addData("slideR position", slideR.getCurrentPosition());

         */


        telemetry.addData("motorPowerFactor", motorPowerFactor);
        telemetry.update();
    }
}
