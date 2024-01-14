package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedLeftAuto", group = "Centerstage")
public class RedLeftAuto extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(90));


        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(22, -25), Math.toRadians(90))
                .strafeRight(10)
                .splineTo(new Vector2d(50, -40), Math.toRadians(270))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(testTrack);
    }
}
