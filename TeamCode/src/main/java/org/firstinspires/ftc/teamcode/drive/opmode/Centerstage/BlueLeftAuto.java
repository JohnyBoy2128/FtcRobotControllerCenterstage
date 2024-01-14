package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueLeftAuto", group = "Centerstage")
public class BlueLeftAuto extends LinearOpMode {

    private ScoringMechanism mechanism = new ScoringMechanism();
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        mechanism.closeRightClaw();
        mechanism.closeLeftClaw();

        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.75, Math.toRadians(270.00)))
                .lineTo(new Vector2d(51.00, 62.75))
                .build();




        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(testTrack);
    }
/*
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.enableLiveView(true);
        builder.addProcessor(aprilTag); // Set and enable the processor.

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()

 */

}
