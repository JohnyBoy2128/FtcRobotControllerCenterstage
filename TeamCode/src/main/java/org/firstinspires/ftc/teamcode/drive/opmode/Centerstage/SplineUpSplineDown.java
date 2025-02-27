package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Spline Up, Spline Down", group = "Centerstage")
public class SplineUpSplineDown extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(12.00, -62.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(20.75, -29.00), Math.toRadians(90.00))
                .build();





        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(testTrack);
    }

}
