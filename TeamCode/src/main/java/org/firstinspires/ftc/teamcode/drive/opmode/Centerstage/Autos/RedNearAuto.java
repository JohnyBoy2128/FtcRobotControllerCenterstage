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
@Autonomous(name="Red Near AUTO", group="Autos")
public class RedNearAuto extends BaseAuto {

    // POSITIONS GOOD
    public static customPose2D boardRight = new customPose2D(51, -42.5, 0);
    public static customPose2D boardCenter = new customPose2D(51, -35.5, 0);
    public static customPose2D boardLeft = new customPose2D(51, -28, 0);
    public static customPose2D lineRight = new customPose2D(25.25, -46.8, 90);
    public static customPose2D lineCenter = new customPose2D(18.5, -39, 90);
    public static customPose2D lineLeft = new customPose2D(14.375, -34.5, 180);

    public static customPose2D startPose = new customPose2D(12, -62.75, 90);

    @Override
    public TrajectorySequence trajectorySequenceBuilder(CenterstagePipeline.detectionStates detectionState) {

        switch (detectionState) {
            case ONE:       // RIGHT SIDE
                return drive.trajectorySequenceBuilder(new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h)))
                        // setting the constraints for movement
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        // moving toward the right line with the left pixel grabber
                        .splineTo(new Vector2d(lineRight.x, lineRight.y), Math.toRadians(lineRight.h))
                        .forward(8)
                        .back(9)

                        // lower arm, open left grabber
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // line back to drop off pixel
                        .lineTo(new Vector2d(25.51, -59.37))

                        // pick up arm for board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // moving towards the right side of the board
                        .lineTo(new Vector2d(27.62, -55.32))
                        .splineToSplineHeading(new Pose2d(boardRight.x, boardRight.y), Math.toRadians(boardRight.h))

                        // open right grabber to drop pixel
                        .addTemporalMarker(() -> mechanism.openRightClaw())
                        .waitSeconds(0.3)

                        // back up away from the board, and move toward the center of the field in the backstage area
                        .splineToSplineHeading(new Pose2d(boardRight.x - 5, boardRight.y), Math.toRadians(boardRight.h))
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .strafeRight(15)

                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))


                        .build();

            case TWO:       // CENTER SIDE
                return drive.trajectorySequenceBuilder(new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h)))
                        // setting the constraints for movement
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        // moving toward the center line with the left pixel grabber
                        .splineTo(new Vector2d(lineCenter.x, lineCenter.y), Math.toRadians(lineCenter.h))
                        .forward(8)
                        .back(10)

                        // lower arm, open left grabber
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // move right out of the way of the pixel, turn to board
                        .lineTo(new Vector2d(30.00, -50))

                        // pick up arm for board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // move to center of board
                        .lineToLinearHeading(new Pose2d(boardCenter.x, boardCenter.y, Math.toRadians(boardCenter.h)))
                        // open right grabber to drop pixel
                        .addTemporalMarker(() -> mechanism.openRightClaw())
                        .waitSeconds(0.3)

                        // back up away from the board, and move toward the center of the field in the backstage area
                        .splineToSplineHeading(new Pose2d(boardCenter.x - 5, boardCenter.y), Math.toRadians(boardCenter.h))
                        .turn(Math.toRadians(90))
                        .forward(24)
                        .strafeRight(15)

                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))


                        .build();

            case THREE:     // LEFT SIDE
                return drive.trajectorySequenceBuilder(new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h)))
                        // setting the constraints for movement
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        // moving toward the left line with the left pixel grabber
                        .splineTo(new Vector2d(lineLeft.x, lineLeft.y), Math.toRadians(lineLeft.h))
                        .forward(9)
                        .back(11)

                        // lower arm, open left grabber
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // move right out of the way of the pixel, turn to board
                        .lineTo(new Vector2d(lineLeft.x + 12, lineLeft.y))

                        // pick up arm for board, and close grabber to reach farther left
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))
                        .addTemporalMarker(() -> mechanism.closeLeftClaw())

                        // move to left side of board
                        .lineTo(new Vector2d(lineLeft.x + 20, lineLeft.y))
                        .splineToSplineHeading(new Pose2d(boardLeft.x, boardLeft.y, Math.toRadians(boardLeft.h)), Math.toRadians(boardLeft.h))

                        // open right grabber to drop pixel
                        .addTemporalMarker(() -> mechanism.openRightClaw())
                        .waitSeconds(0.3)

                        // back up away from the board, and move toward the center of the field in the backstage area
                        .splineToSplineHeading(new Pose2d(boardLeft.x - 5, boardLeft.y), Math.toRadians(boardLeft.h))
                        .turn(Math.toRadians(90))
                        .forward(18)
                        .strafeRight(15)

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
