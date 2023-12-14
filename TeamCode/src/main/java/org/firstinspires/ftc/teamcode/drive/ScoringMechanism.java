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
    private Servo servoHookL = null;
    private Servo servoHookR = null;


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
        servoHookL = hwmap.get(Servo.class, "hookL");
        servoHookR = hwmap.get(Servo.class, "hookR");


        // setup encoders on arm and worm gear motors
        motorActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // enables pwm for the grabber servos
        //servoGrabberL.getController().pwmEnable();
        //servoGrabberR.getController().pwmEnable();

    }

    //methods for retrieving postiton information
    public String getHookServoPosition() {
        return "L: " + Double.toString(servoHookL.getPosition()) + ",  R: " + Double.toString(servoHookR.getPosition());
    }

    // methods for opening and closing each servo
    // smaller numbers on the left servo are closed, while
    // smaller numbers on the right servo are open
    public void closeLeftClaw() {
        servoGrabberL.setPosition(0.16);
    }
    public void closeRightClaw() {
        servoGrabberR.setPosition(0.92);
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
            servoHookL.setPosition(servoHookL.getPosition() + 0.01);      // Counterclockwise
        }
        if (state.equals("leftClock")) {
            servoHookL.setPosition(servoHookL.getPosition() - 0.01);      // Clockwise
        }
        if (state.equals("rightCounter")) {
            servoHookR.setPosition(servoHookR.getPosition() + 0.01);      // Counterclockwise
        }
        if (state.equals("rightClock")) {
            servoHookR.setPosition(servoHookR.getPosition() - 0.01);      // Clockwise
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

    public void moveHooks() {


    }


    public void moveToLevel(boardLevels level) {
        // variables for each arm parts positions
        int armHeight = 0;
        int actuatorLength = 0;
        double rotatorLPosition = 0.0;
        double rotatorRPosition = 0.0;
        double hookLPosition = 88;
        double hookRPosition = 25;

        // switch with each of the positions on the board for dropping the pixels
        switch (level) {
            case FLOOR:
                actuatorLength = 0;
                armHeight = 0;
                rotatorLPosition = 0.78;
                rotatorRPosition = 0.24;
                break;
            /*
            case PICKUP:
                actuatorLength = 0;
                armHeight = -200;
                rotatorLPosition = 0.62;
                rotatorRPosition = 0.40;
                hookLPosition = 90;
                hookRPosition = 25;

             */
            case LEVEL1:
                armHeight = -978;
                actuatorLength = 2257;
                rotatorLPosition = 0.62;
                rotatorRPosition = 0.40;
                break;
            case LEVEL2AND3:
                armHeight = -1574;
                actuatorLength = 9515;
                rotatorLPosition = 0.67;
                rotatorRPosition = 0.36;
                break;
            case LEVEL4AND5:
                armHeight = -1781;
                actuatorLength = 12153;
                rotatorLPosition = 0.69;
                rotatorRPosition = 0.34;
                break;
            case HANG:
                armHeight = -3817;
                actuatorLength = 12153;
                rotatorLPosition = 0.84;
                rotatorRPosition = 0.16;
                break;
        }

        //setting the motor positions and powers for each level
        motorActuator.setTargetPosition(actuatorLength);
        motorActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorActuator.setPower(1);

        motorWormGear.setTargetPosition(armHeight);
        motorWormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorWormGear.setPower(1);

        //setting the servo rotator positions
        servoRotatorL.setPosition(rotatorLPosition);
        servoRotatorR.setPosition(rotatorRPosition);

        //setting hook positions
        servoHookL.setPosition(hookLPosition);
        servoHookR.setPosition(hookRPosition);


    }

    public enum boardLevels {
        FLOOR,
        PICKUP,
        LEVEL1,
        LEVEL2AND3,
        LEVEL4AND5,
        HANG,
        STACK5,
        STACK4,
        STACK3,
        STACK2
    }


}

