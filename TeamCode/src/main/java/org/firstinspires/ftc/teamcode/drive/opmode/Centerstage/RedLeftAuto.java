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


        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(-36.53, -62.75, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.04, -20.98))
                .lineTo(new Vector2d(-36.04, -25.19))
                .lineTo(new Vector2d(-49.16, -32.80))
                .lineTo(new Vector2d(-49.00, -13.20))
                .lineTo(new Vector2d(58.72, -13.04))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(testTrack);
    }
}
