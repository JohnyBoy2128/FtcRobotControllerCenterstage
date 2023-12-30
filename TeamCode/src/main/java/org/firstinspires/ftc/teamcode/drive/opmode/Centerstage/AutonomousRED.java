package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "SimpleAutonomous", group = "Autonomous")
public class SimpleAutonomous extends LinearOpMode {

    // Declare motors here
    DcMotor motorFR = null;
    DcMotor motorFL = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;



    // Constants for motor power and duration
    static final double FORWARD_POWER = 0.5;
    static final double BACKWARD_POWER = -0.5;
    static final long DRIVE_DURATION = 2000; // in milliseconds

    @Override
    public void runOpMode() {
        // Initialize motors
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");


        // Set motor directions
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        // Autonomous actions
        driveLeft();
        sleep(2700); // Pause for 1 secon

        // Stop all motors
        stopMotors();
    }

    private void driveForward() {
        motorFL.setPower(FORWARD_POWER);
        motorFR.setPower(FORWARD_POWER);
        motorBL.setPower(BACKWARD_POWER);
        motorBR.setPower(BACKWARD_POWER);
        sleep(100);
    }

    private void driveBackward() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(.5);
        motorBR.setPower(.5);
        sleep(100);
    }

    private void driveLeft() {
        motorFL.setPower(1);
        motorFR.setPower(1);
        motorBL.setPower(1);
        motorBR.setPower(1);
    }

    private void stopMotors() {
        motorFL.setPower(.5);
        motorFR.setPower(.5);
        motorBL.setPower(.5);
        motorBL.setPower(.5);
    }
}
