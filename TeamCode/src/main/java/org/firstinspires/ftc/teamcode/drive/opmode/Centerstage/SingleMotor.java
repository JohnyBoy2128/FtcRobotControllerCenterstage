package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//Just moves a motor and servo
@TeleOp
public class SingleMotor extends LinearOpMode {

    private DcMotor actuator;
    private Servo trigger;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();



    @Override
    public void runOpMode() {
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        trigger = hardwareMap.get(Servo.class, "trigger");

        trigger.getController().pwmEnable();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double lsy = -gamepad1.left_stick_y; // Remember, this is reversed!
            double rsy = gamepad1.right_stick_y;



            // Send calculated power to wheels

            telemetry.addData("Speed", lsy);
            telemetry.addData("Position", rsy);


            try {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
            }
            catch (Exception e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1 are being copied from valid Gamepads
            }

            // Rising edge detector for right bumper on gp2.
            if (currentGamepad1.dpad_down) {
                actuator.setPower(0);
            }
            if (currentGamepad1.dpad_left) {
                actuator.setPower(.5);
            }
            if (currentGamepad1.dpad_right) {
                actuator.setPower(.75);
            }
            if (currentGamepad1.dpad_up) {
                actuator.setPower(1);
            }
            if (lsy >= 0) {
                actuator.setPower(lsy);
            }
            if (currentGamepad1.right_bumper) {
                trigger.setPosition(1);
            }
            if (currentGamepad1.triangle) {
                trigger.setPosition(.5);
            }

            String controller = gamepad1.toString();
            telemetry.addData("Gamepad Status", controller);
            telemetry.update();


        }



    }

}

