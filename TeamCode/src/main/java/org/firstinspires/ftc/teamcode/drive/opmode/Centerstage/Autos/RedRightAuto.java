package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "CS: RedRightAuto", group = "Autos")
public class RedRightAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*
         *  Starting position for the RED side on the RIGHT is (12, -62.75)
         *  Pixel on the Right  line is center (23, -34) shifted down 8.875 and left 1
         *  Pixel on the Center line is center () shifted down and left
         *  Pixel on the Left   line is center () shifted down and left
         */



        Pose2d startPose = new Pose2d(12, -62.75, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence testTrack = drive.trajectorySequenceBuilder(startPose)
                // setting the constraints for movement
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                // move up to center pixel line
                .splineTo(new Vector2d(20.00, -29.75), Math.toRadians(90.00))
                .waitSeconds(0.5)

                // move right out of the way of the pixel, turn to board
                .lineTo(new Vector2d(30.00, -29.75))
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(50.00, -36.00))
                .build();



        waitForStart();
        if(isStopRequested()) return;
        drive.followTrajectorySequence(testTrack);
    }

}
