package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "RedRightAuto", group = "Centerstage")
public class RedRightAuto extends LinearOpMode {

    // declaring basic vision stuff
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // declaring ScoringMechanism object
    protected ScoringMechanism mechanism = new ScoringMechanism();



    @Override
    public void runOpMode() {

        initAprilTag();


        // setting up basic stuff for start of match, where one pixel is loaded into the grabber, and one is on the pixel pusher
        mechanism.lowerPixelPusher();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(24, -48, Math.toRadians(90));


        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(24, -48.00, Math.toRadians(90)))
                .strafeRight(16)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(24, -48), Math.toRadians(90))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(testTrack);
    }

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

}
