package org.firstinspires.ftc.teamcode.drive.opmode.Centerstage.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.drive.opmode.Centerstage.BaseAuto;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CenterstagePipeline;

@Config
@Autonomous(name="Red Left AUTO", group="Autos")
public class RedLeftAutoFINAL extends BaseAuto {

    // POSITIONS GOOD
    public static customPose2D boardRight = new customPose2D(50, -42.5, 0);
    public static customPose2D boardCenter = new customPose2D(50, -36, 0);
    public static customPose2D boardLeft = new customPose2D(50, -30, 0);
    public static customPose2D lineRight = new customPose2D(-36, -33, 0);
    public static customPose2D lineCenter = new customPose2D(-39, -24.75, 90);
    public static customPose2D lineLeft = new customPose2D(-47.25, -33, 90);

    public static customPose2D startPose = new customPose2D(-36, -62.75, 90);

    @Override
    public TrajectorySequence trajectorySequenceBuilder(CenterstagePipeline.detectionStates detectionState) {

        switch (detectionState) {
            case ONE:       // RIGHT SIDE
                return drive.trajectorySequenceBuilder(new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h)))
                        // setting the constraints for movement
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        // moving to the left spike mark
                        .splineTo(new Vector2d(lineRight.x, lineRight.y), Math.toRadians(lineRight.h))

                        // lower arm to the ground, open the left grabber, and move the arm back up
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // moving across the middle
                        .lineTo(new Vector2d(-36.00, -12.00))
                        .lineTo(new Vector2d(30.00, -12.00))

                        // moving arm up to board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // moving to board
                        .splineTo(new Vector2d(boardRight.x, boardRight.y), Math.toRadians(boardRight.h))

                        // opening claw
                        .addTemporalMarker(() -> mechanism.openRightClaw())

                        // remove after auto testing, this is just for not havingt to reset the arm each time
                        .waitSeconds(8)
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        .build();

            case TWO:       // CENTER SIDE
                return drive.trajectorySequenceBuilder(new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h)))
                        // setting the constraints for movement
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        // moving to center spike mark
                        .splineTo(new Vector2d(lineCenter.x, lineCenter.y), Math.toRadians(lineCenter.h))

                        // lower arm to the ground, open the left grabber, and move the arm back up
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // moving back from line
                        .lineToSplineHeading(new Pose2d(-54.00, -36.00, Math.toRadians(90.00)))

                        // moving across middle
                        .lineTo(new Vector2d(-54.00, -12.00))
                        .lineToConstantHeading(new Vector2d(24.00, -12.00))

                        // moving arm up to board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // moving to board
                        .lineToLinearHeading(new Pose2d(boardCenter.x, boardCenter.y, Math.toRadians(boardCenter.h)))

                        // opening claw
                        .addTemporalMarker(() -> mechanism.openRightClaw())

                        // remove after auto testing, this is just for not havingt to reset the arm each time
                        .waitSeconds(8)
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        .build();

            case THREE:     // LEFT SIDE
                return drive.trajectorySequenceBuilder(new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h)))
                        // setting the constraints for movement
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        // moving up to the left spike mark
                        .splineTo(new Vector2d(lineLeft.x, lineLeft.y), Math.toRadians(lineLeft.h))

                        // lower arm to the ground, open the left grabber, and move the arm back up
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // moving away from pixel
                        .lineTo(new Vector2d(-57.42, -43.65))

                        // moving to middle, moving right
                        .lineTo(new Vector2d(-57.10, -12.00))
                        .lineTo(new Vector2d(24, -12.00))

                        // lifting arm for board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // moving to board
                        .lineToLinearHeading(new Pose2d(boardLeft.x, boardLeft.y, Math.toRadians(boardLeft.h)))

                        // opening claw
                        .addTemporalMarker(() -> mechanism.openRightClaw())

                        // remove after auto testing, this is just for not havingt to reset the arm each time
                        .waitSeconds(8)
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        .build();

        }

        // won't be reached
        return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .build();
    }

    @Override
    public Pose2d startPoseSetter() {
        return new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h));
    }
}
