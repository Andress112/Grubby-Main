package com.stellar.meepmeeptravx;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTravX {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18.1).setDimensions(18.1,17.5)
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(9, -58.4, Math.toRadians(270)))
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35.6, -64.48, Math.toRadians(0)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(69, -69, Math.toRadians(0)))
//                           Right side

                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-51.7, -58.9, Math.toRadians(45.67)), Math.toRadians(220))

////                        // First Specimen
////
//                        .strafeTo(new Vector2d(9, -27.5))
//                        .waitSeconds(1)
//
//                        // Move all samples
//                        .setTangent(200)
//                        .splineToLinearHeading(new Pose2d(47.6, -50, Math.toRadians(90)), Math.toRadians(0))
//
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(90)), Math.toRadians(0))
//
//                        .setTangent(90)
//                        .splineToLinearHeading(new Pose2d(50.17, -39.25, Math.toRadians(50.39)), Math.toRadians(200))
////
//                        .waitSeconds(1)
//
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(50, -39, Math.toRadians(-60)), Math.toRadians(0))
//
//                        .waitSeconds(1)
//
////                        // Second Specimen
////
//                        .setTangent(60)
//                        .splineToLinearHeading(new Pose2d(19.9681, -44.1309, Math.toRadians(-23.9854)), Math.toRadians(200))
//
//                        .waitSeconds(1)
//
//                        .setTangent(60)
//                        .splineToLinearHeading(new Pose2d(6, -27.5, Math.toRadians(270)), Math.toRadians(90))
//
//                        .waitSeconds(1)
//
//                        // Thirth Specimen
//
//                        .setTangent(180)
//                        .splineToLinearHeading(new Pose2d(19.9681, -44.1309, Math.toRadians(-23.9854)), Math.toRadians(0))
//
//                        .waitSeconds(1)
//
//                        .setTangent(60)
//                        .splineToLinearHeading(new Pose2d(3, -27.5, Math.toRadians(270)), Math.toRadians(90))
//
//                        .waitSeconds(1)
//
//                        // Thirth Specimen
//
//                        .setTangent(180)
//                        .splineToLinearHeading(new Pose2d(19.9681, -44.1309, Math.toRadians(-23.9854)), Math.toRadians(0))
//
//                        .waitSeconds(1)
//
////                        // Parking
//
//                        .setTangent(200)
//                        .splineToLinearHeading(new Pose2d(47.4, -56.8, Math.toRadians(270)), Math.toRadians(0))

                           // Left Side Testing

                        // -52 -54.4 45.67
                        // First sample preload
//                        .setTangent(Math.toRadians(120))
//                        .splineToLinearHeading(new Pose2d(-51.6, -54.4, Math.toRadians(45.67)), Math.toRadians(180))
//                        // Collect first sample
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(-47.8, -46, Math.toRadians(92.5)), Math.toRadians(180))
//                        // Place first sample
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(-51.6, -54.4, Math.toRadians(45.67)), Math.toRadians(180))
//                        // Collect second sample
//                        .splineToLinearHeading(new Pose2d(-58.5, -44.7, Math.toRadians(94.5)), Math.toRadians(180))
//                        // Place second sample
//                        .setTangent(Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-51.6, -54.4, Math.toRadians(45.67)), Math.toRadians(270))
//                        // Collect third sample
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(-60, -44.31, Math.toRadians(112)), Math.toRadians(180))
//                        // Place third sample
//                        .setTangent(Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-51.6, -54.4, Math.toRadians(45.67)), Math.toRadians(0))
//                        // Park
//                        .setTangent(45)
//                        .splineToLinearHeading(new Pose2d(-22, -5, Math.toRadians(180)), Math.toRadians(0))

                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
