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
    private Servo servoPixelPusher = null;
    private Servo servoPlaneTrigger = null;
    private Servo servoAlignerL = null;
    private Servo servoAlignerR = null;
    private Servo servoTESTING = null;



    // called in CenterstageMainTeleOp to initialize servo and arm motors
    public void init(HardwareMap hwmap, boolean resetMotorEncoders) {

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
        servoPixelPusher = hwmap.get(Servo.class, "pixelPusher");
        servoPlaneTrigger = hwmap.get(Servo.class, "planeTrigger");
        servoTESTING = hwmap.get(Servo.class, "testServo");

        if (resetMotorEncoders) {
            // setup encoders on arm and worm gear motors
            motorActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorWormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorWormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




    }


    /*
     *     _____      _   _               __  __      _   _               _
     *    / ____|    | | | |             |  \/  |    | | | |             | |
     *    | |  __  ___| |_| |_ ___ _ __   | \  / | ___| |_| |__   ___   __| |___
     *    | | |_ |/ _ \ __| __/ _ \ '__|  | |\/| |/ _ \ __| '_ \ / _ \ / _` / __|
     *    | |__| |  __/ |_| ||  __/ |     | |  | |  __/ |_| | | | (_) | (_| \__ \
     *     \_____|\___|\__|\__\___|_|     |_|  |_|\___|\__|_| |_|\___/ \__,_|___/


     */
    public String getHookServoPosition() {
        return "L Hook Position: " + Double.toString(servoHookL.getPosition()) + ",  R Hook Position: " + Double.toString(servoHookR.getPosition());
    }

    //method for just finding out the position of plane launcher servo
    public String getPlaneTriggerPosition() {
        return "Plane Trigger Position: " + servoPlaneTrigger.getPosition();
    }

    public int getActuatorPosition(){
        return motorActuator.getCurrentPosition();
    }
    public int getArmPosition() {
        return motorWormGear.getCurrentPosition();
    }


    /*
     *     _____                     __  __      _   _               _
     *    |  __ \                   |  \/  |    | | | |             | |
     *    | |  | | ___   ___ _ __   | \  / | ___| |_| |__   ___   __| |___
     *    | |  | |/ _ \ / _ \ '__|  | |\/| |/ _ \ __| '_ \ / _ \ / _` / __|
     *    | |__| | (_) |  __/ |     | |  | |  __/ |_| | | | (_) | (_| \__ \
     *    |_____/ \___/ \___|_|     |_|  |_|\___|\__|_| |_|\___/ \__,_|___/
     */


    // methods for opening and closing each servo
    // smaller numbers on the right servo are closed, while
    // smaller numbers on the left servo are open
    public void openLeftClaw() {
        servoGrabberL.setPosition(0.29);
    }
    public void openRightClaw() {
        servoGrabberR.setPosition(0.79);
    }
    public void closeLeftClaw() {
        servoGrabberL.setPosition(0.16);
    }
    public void closeRightClaw() {
        servoGrabberR.setPosition(0.92);

    }


    // methods for moving arm back to starting position at the start of each match
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

    // methods for just testing hook positions, to avoid error
    public void restLeftHook() {
        servoHookL.setPosition(0.80);
    }
    public void hangLeftHook() {
        servoHookL.setPosition(0.31);
    }
    public void restRightHook() {
        servoHookR.setPosition(0.21);
    }
    public void hangRightHook() {
        servoHookR.setPosition(0.81);
    }


    // methods for moving a test servo
    public void moveServoPlusBig() {servoPixelPusher.setPosition(servoPixelPusher.getPosition() + 0.1);}
    public void moveServoMinusBig() {servoPixelPusher.setPosition(servoPixelPusher.getPosition() - 0.1);}
    public void moveServoPlusLittle() {servoPixelPusher.setPosition(servoPixelPusher.getPosition() + 0.01);}
    public void moveServoMinusLittle() {servoPixelPusher.setPosition(servoPixelPusher.getPosition() - 0.01);}





    public void liftPixelPusher() {
        // Alright, need plane to move from 0 degrees to 90 degrees, giving around
        // 0.00 - 0.33, adjusted for dead space is 0.15 - 0.48
        //                                     (lifted) - (lowered)

        // Take off hub, move servo, put hub on, test position.
        // 0.15 is lifted, 0.48 is lowered

        servoPixelPusher.setPosition(0.15);
    }
    public void lowerPixelPusher() {
        servoPixelPusher.setPosition(0.48);
    }

    public void launchPlane() {
        // Alright, 0 degrees to 45 degrees, 0.00 - 0.17,
        // adjusted to 0.15 - 0.32. 0.32 is down, and 0.15 is up
        //         (lifted) - (lowered)

        servoPlaneTrigger.setPosition(0.50);
    }
    public void lowerPlaneLauncher() {
        servoPlaneTrigger.setPosition(0.80);
    }


    public void moveToLevel(boardLevels level) {
        // variables for each arm parts positions
        int armHeight = 0;
        int actuatorLength = 0;
        double rotatorLPosition = 0.0;
        double rotatorRPosition = 0.0;
        double hookLPosition = 0.8;
        double hookRPosition = 0.21;

        // switch with each of the positions on the board for dropping the pixels
        switch (level) {
            case FLOOR:
                armHeight = 1340;
                actuatorLength = 0;
                rotatorLPosition = 0.78;
                rotatorRPosition = 0.24;
                hookLPosition = 0.80;
                hookRPosition = 0.21;
                break;
            case PICKUP:
                armHeight = 1100;
                actuatorLength = 0;
                rotatorLPosition = 0.62;
                rotatorRPosition = 0.40;
                hookLPosition = 0.80;
                hookRPosition = 0.21;
                break;
            case LEVEL1:
                armHeight = 322;
                actuatorLength = 2257;
                rotatorLPosition = 0.62;
                rotatorRPosition = 0.40;
                hookLPosition = 0.80;
                hookRPosition = 0.21;
                break;
            case ZEROPOSITION:
                armHeight = 0;
                actuatorLength = 0;
                rotatorLPosition = 0.62;
                rotatorRPosition = 0.40;
                hookLPosition = 0.80;
                hookRPosition = 0.21;
                break;
            case LEVEL2AND3:
                armHeight = -274;
                actuatorLength = 9515;
                rotatorLPosition = 0.67;
                rotatorRPosition = 0.36;
                hookLPosition = 0.80;
                hookRPosition = 0.21;
                break;
            case LEVEL4AND5:
                armHeight = -481;
                actuatorLength = 12153;
                rotatorLPosition = 0.69;
                rotatorRPosition = 0.34;
                hookLPosition = 0.80;
                hookRPosition = 0.21;
                break;
            case LIFTTOHANG:

                closeRightClaw();
                closeLeftClaw();
                lowerPlaneLauncher();

                armHeight = -2517;
                actuatorLength = 12153;
                rotatorLPosition = 0.36;
                rotatorRPosition = 0.66;
                hookLPosition = 0.31;
                hookRPosition = 0.81;
                break;
            case PULLUP:
                armHeight = -2517;
                actuatorLength = 9000;
                rotatorLPosition = 0.36;
                rotatorRPosition = 0.66;
                hookLPosition = 0.31;
                hookRPosition = 0.81;
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
        ZEROPOSITION,
        LEVEL2AND3,
        LEVEL4AND5,
        LIFTTOHANG,
        PULLUP,
        STACK4,
        STACK3,
        STACK2
    }


}

