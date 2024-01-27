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
@Autonomous(name="Blue Left AUTO", group="Autos")
public class BlueLeftAutoFINAL extends BaseAuto {

    public static customPose2D boardRight = new customPose2D(50, 30, 0);
    public static customPose2D boardCenter = new customPose2D(50, 36, 0);
    public static customPose2D boardLeft = new customPose2D(50, 42.5, 0);
    public static customPose2D lineRight = new customPose2D(0.75, 33, 180);
    public static customPose2D lineCenter = new customPose2D(15, 24.5, 270);
    public static customPose2D lineLeft = new customPose2D(23.5, 33, 270);

    public static customPose2D startPose = new customPose2D(12, 62.75, 270);

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

                        // lower arm, open left grabber
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // moving away from pixel moving to board
                        .lineToSplineHeading(new Pose2d(24, 48, Math.toRadians(270)))

                        // pick up arm for board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // move to right side of the board
                        .lineToSplineHeading(new Pose2d(50.00, 30.00, Math.toRadians(0.00)))
                        .build();


            case TWO:       // OLD AUTO FOR TESTING
                return drive.trajectorySequenceBuilder(new Pose2d(startPose.x, startPose.y, Math.toRadians(startPose.h)))
                        // setting the constraints for movement
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        // moving toward the center line with the left pixel grabber
                        .splineTo(new Vector2d(lineCenter.x, lineCenter.y), Math.toRadians(lineCenter.h))

                        // lower arm, open left grabber
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.FLOOR))
                        .waitSeconds(1) // waiting so long jic, can be adjusted later
                        .addTemporalMarker(() -> mechanism.openLeftClaw())
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.ZEROPOSITION))

                        // move right out of the way of the pixel, turn to board
                        .lineTo(new Vector2d(30.00, 29.75))

                        // pick up arm for board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // move to center of board
                        .lineToLinearHeading(new Pose2d(boardCenter.x, boardCenter.y, Math.toRadians(boardCenter.h)))

                        // open right grabber to drop pixel
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

                        // moving away from pixel, turning to 90
                        .lineTo(new Vector2d(34.00, 42.87))

                        // moving arm up to board
                        .addTemporalMarker(() -> mechanism.moveToLevel(ScoringMechanism.boardLevels.LEVEL1))

                        // moving to the board
                        .splineToSplineHeading(new Pose2d(boardLeft.x, boardLeft.y, Math.toRadians(boardLeft.h)), Math.toRadians(boardLeft.h))

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
