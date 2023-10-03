package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Just moves a motor
@TeleOp
public class LinearActuator extends LinearOpMode {

    private DcMotor actuator;
    public static double actuatorPower = -1.0;



    @Override
    public void runOpMode() {
        actuator = hardwareMap.get(DcMotor.class, "actuator");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();



            actuator.setPower(actuatorPower);

        }
    }

}

