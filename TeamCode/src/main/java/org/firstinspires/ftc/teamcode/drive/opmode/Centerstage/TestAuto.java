package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "CS: Test Auto", group = "Centerstage")
public class TestAuto extends LinearOpMode {

    protected ScoringMechanism mechanism = new ScoringMechanism();

    @Override
    public void runOpMode() {

        // setting up basic stuff for start of match, where one pixel is loaded into the grabber, and one is on the pixel pusher
        mechanism.lowerPixelPusher();
        mechanism.closeRightClaw();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(24, -48, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(24, -48.00, Math.toRadians(90)))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .strafeRight(16)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(24, -48), Math.toRadians(90))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(testTrack);
    }

}
