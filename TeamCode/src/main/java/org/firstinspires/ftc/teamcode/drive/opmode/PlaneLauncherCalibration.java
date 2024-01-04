package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;


/*
 *  This method uses two gamepads to calibrate the plane launcher angles and launch the plane
 *
 *  Gamepad 1 controls the launcher angles
 *  Gamepad 2 controls the plane trigger
 *
 *  The launcher down all the way is 0.67
 *  The launcher up all the way 0.40
 *
 */

@TeleOp(name="PlaneLauncherCalibration", group="Linear Opmode")
public class PlaneLauncherCalibration extends LinearOpMode {

    // declaring the two servos needed for this program
    private Servo servoLauncher = null;
    private Servo servoTrigger = null;


    // declare controller variables, which will be used to control setting the servo to
    // different positions
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    // decimal format to make telemetry easier to read
    DecimalFormat df = new DecimalFormat("#.##");



    // main method
    public void runOpMode() {

        // initialize the servo called "servo"
        servoLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        servoTrigger = hardwareMap.get(Servo.class, "planeTrigger");

        waitForStart();


        while (opModeIsActive()) { //---------------PRESSES PLAY---------------

            // method to update gamepads once per cycle
            try {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);
            } catch (Exception e) {
            }

            // simple ifs for buttons to move the plane launcher
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
                servoLauncher.setPosition(1);
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
                servoLauncher.setPosition(0);
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
                servoLauncher.setPosition(Math.max(servoLauncher.getPosition() - 0.1, 0));
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right)
                servoLauncher.setPosition(Math.min(servoLauncher.getPosition() + 0.1, 1));
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
                servoLauncher.setPosition(Math.max(servoLauncher.getPosition() - 0.01, 0));
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
                servoLauncher.setPosition(Math.min(servoLauncher.getPosition() + 0.01, 1));

            // telemetry just to see the position
            telemetry.addData("Launcher Position: ", df.format(servoLauncher.getPosition()));


            // simple ifs for buttons to move the plane trigger
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)
                servoTrigger.setPosition(1);
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)
                servoTrigger.setPosition(0);
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
                servoTrigger.setPosition(Math.max(servoTrigger.getPosition() - 0.1, 0));
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)
                servoTrigger.setPosition(Math.min(servoTrigger.getPosition() + 0.1, 1));
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper)
                servoTrigger.setPosition(Math.max(servoTrigger.getPosition() - 0.01, 0));
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
                servoTrigger.setPosition(Math.min(servoTrigger.getPosition() + 0.01, 1));

            // ifs just for setting two different positions for launched and set
            if (currentGamepad2.square && !previousGamepad2.square) {
                servoTrigger.setPosition( /* launch */);
            }
            if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                servoTrigger.setPosition( /* reset */);
            }

            // telemetry just to see the position
            telemetry.addData("Trigger Position: ", df.format(servoTrigger.getPosition()));
            telemetry.update();



        }
    }
}
