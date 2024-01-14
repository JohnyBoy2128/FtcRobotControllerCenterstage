package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueRightAuto", group = "Centerstage")
public class BlueRightAuto extends LinearOpMode {

    private ScoringMechanism mechanism = new ScoringMechanism();
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        mechanism.init(hardwareMap, true);

        mechanism.closeLeftClaw();
        mechanism.closeRightClaw();

        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 62.75, Math.toRadians(270.00)))
                .lineTo(new Vector2d(-36.04, 20.98))
                .lineTo(new Vector2d(-36.04, 29.08))
                .lineTo(new Vector2d(-49.16, 32.80))
                .lineTo(new Vector2d(-49.00, 13.20))
                .lineTo(new Vector2d(58.72, 13.04))
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
