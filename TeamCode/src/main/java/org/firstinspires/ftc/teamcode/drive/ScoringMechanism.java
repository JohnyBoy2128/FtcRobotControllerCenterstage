package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringMechanism {

    // declare different parts for scoring mechanism
    private DcMotor motorActuator = null;
    private DcMotor motorWormGear = null;
    private Servo servoGrabberL = null;
    private Servo servoGrabberR = null;
    private Servo servoRotatorL = null;
    private Servo servoRotatorR = null;


    // how many encoder tics make one full actuator motor rotation
    static final double ARM_TICS_IN_ROT = 537.7;
    // the number of mm the slides move from one motor rotation
    static final double SLIDE_MM_FROM_ROT = 8.28;
    // number of tics to move slide by 1cm
    static final double ARM_TICS_IN_CM = ARM_TICS_IN_ROT / (SLIDE_MM_FROM_ROT / 10);

    // the slide's level; 0-3, 0 being ground, 3 being highest pole
    int slideLvl = 0;


    // called in CenterstageMainTeleOp to initialize servo and arm motors
    public void init(HardwareMap hwmap) {

            //  ARM MOTORS
        motorActuator = hwmap.get(DcMotor.class, "actuator");
        motorWormGear = hwmap.get(DcMotor.class, "wormy");

            //  GRABBER SERVOS
        servoGrabberL = hwmap.get(Servo.class, "grabberL");
        servoGrabberR = hwmap.get(Servo.class, "grabberR");
        servoRotatorL = hwmap.get(Servo.class, "rotatorL");
        servoRotatorR = hwmap.get(Servo.class, "rotatorR");

        // setup encoders on arm and worm gear motors
        motorActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // enables pwm for the grabber servos
        servoGrabberL.getController().pwmEnable();
        servoGrabberR.getController().pwmEnable();

    }

    // methods for opening and closing each servo
    // smaller numbers on the left servo are closed, while
    // smaller numbers on the right servo are open
    public void closeLeftClaw() {
        servoGrabberL.setPosition(0.16);
    }
    public void closeRightClaw() {
        servoGrabberR.setPosition(0.93);
    }
    public void openLeftClaw() {
        servoGrabberL.setPosition(0.29);
    }
    public void openRightClaw() {
        servoGrabberR.setPosition(0.79);
    }

    //      CALIBRATION METHOD
    // method to find out servo directions and such
    public String calibrateServos(String state) {

        // take different states passed to the method and move each
        // servo clockwise and counterclockwise
        if (state.equals("leftCounter")) {
            servoRotatorL.setPosition(servoRotatorL.getPosition() + 0.01);      // Counterclockwise
        }
        if (state.equals("leftClock")) {
            servoRotatorL.setPosition(servoRotatorL.getPosition() - 0.01);      // Clockwise
        }
        if (state.equals("rightCounter")) {
            servoRotatorR.setPosition(servoRotatorR.getPosition() + 0.01);      // Counterclockwise
        }
        if (state.equals("rightClock")) {
            servoRotatorR.setPosition(servoRotatorR.getPosition() - 0.01);      // Clockwise
        }

        // string containing each servos position
        return "L: " + Double.toString(servoRotatorL.getPosition()) + ",  R: " + Double.toString(servoRotatorR.getPosition());

    }

    public void extendActuator() {
        int currentActuatorPosition = motorActuator.getCurrentPosition();
        motorActuator.setTargetPosition(currentActuatorPosition + 538);
        motorActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorActuator.setPower(1);
    }
    public void retractActuator() {
        int currentActuatorPosition = motorActuator.getCurrentPosition();
        motorActuator.setTargetPosition(currentActuatorPosition - 538);
        motorActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorActuator.setPower(1);
    }
    public void moveArmBack() {
        int currentArmPosition = motorWormGear.getCurrentPosition();
        motorWormGear.setTargetPosition(currentArmPosition + 100);
        motorWormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorWormGear.setPower(1);
    }
    public void moveArmForward() {
        int currentArmPosition = motorWormGear.getCurrentPosition();
        motorWormGear.setTargetPosition(currentArmPosition - 100);
        motorWormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorWormGear.setPower(1);
    }



    // updates the actuator and the worm gear to move the arm out and around




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

