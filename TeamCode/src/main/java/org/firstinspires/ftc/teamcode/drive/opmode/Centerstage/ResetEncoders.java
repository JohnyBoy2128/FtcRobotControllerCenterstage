package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;

@TeleOp(name="Reset Encoders")
public class ResetEncoders extends LinearOpMode {

    private DistanceSensor distance;
    private ScoringMechanism mechanism;

    @Override
    public void runOpMode() {

        mechanism = new ScoringMechanism();

        // Get the distance sensor and motor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        mechanism.init(hardwareMap, true);


        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("distance", distance.getDistance(DistanceUnit.MM));
            telemetry.update();

            // If the distance in centimeters is less than 10, set the power to 0.3
            //if (distance.getDistance(DistanceUnit.CM) < 10)
        }
    }

}
