package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "CS: First Auto", group = "Centerstage")
public class FirstAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(24, -48, Math.toRadians(90));


        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(new Pose2d(24.00, -48.00, Math.toRadians(0)))
                .splineTo(new Vector2d(48.00, -0.00), Math.toRadians(0))
                .waitSeconds(1)
                .splineTo(new Vector2d(31.00, 48.00), Math.toRadians(90))
                .waitSeconds(1)
                .splineTo(new Vector2d(12.00, -0.00), Math.toRadians(-90.00))
                .waitSeconds(1)
                .splineTo(new Vector2d(36.00, -60.00), Math.toRadians(0.00))
                .waitSeconds(1)
                .build();




        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(testTrack);
    }

}
