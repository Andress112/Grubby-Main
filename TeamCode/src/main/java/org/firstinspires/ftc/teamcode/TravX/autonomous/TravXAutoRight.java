package org.firstinspires.ftc.teamcode.TravX.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TravX.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.TravX.utilities.SampleDetector;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "TravX Auto Right", group = "Autonomous", preselectTeleOp = "TravX Main")
public class TravXAutoRight extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    FtcDashboard ftcDashboard;
    OpenCvWebcam camera;

    // Auto Collector variables
    private double previousError = 0;
    private long previousTime = 0;
    private double integral = 0;
    public boolean targetReached = false;

    // timeout's
    long Looptimeout = 1000; // 1 second in milliseconds
    long LoopstartTime = System.currentTimeMillis();

    // Getting the hardware initialized
    private final TravXHardware Robot = new TravXHardware(this);
    private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(Robot);
    private final SampleDetector SampleDetectorPipeline = new SampleDetector();

    private enum AutoState {
        START,
        INITIAL_SPECIMEN_SCORING,
        MOVE_SAMPLES,
        COLLECT_FIRST_SPECIMEN,
        PLACE_FIRST_SPECIMEN,
        COLLECT_SECOND_SPECIMEN,
        PLACE_SECOND_SPECIMEN,
        FINAL_PARK_SIDE,
        FINISH
    }

    private AutoState currentState = AutoState.START;

    private void threadSleep(long milliseconds) {
        try {
            for (int i = 0; i < milliseconds; i++) {
                if (isStopRequested()) {
                    return;
                } else {
                    Thread.sleep(1);
                }
            }
        } catch (InterruptedException e) {
            return;
        }
    }

    public void moveToObject(MecanumDrive drive, int relDx, int relDy, double speed) {
        // Reset the function variables when it's called again after being used
        if (targetReached) {
            previousTime = 0;
            previousError = 0;
            integral = 0;
            targetReached = false;
        }

        // Normalize the distance and direction
        double distance = Math.sqrt(relDx * relDx + relDy * relDy);

        if (distance < 5) { // Adjust threshold as needed
            drive.setDrivePowers( new PoseVelocity2d(new Vector2d(0, 0), 0));
            targetReached = true;
            return;
        }

        // PD Controller constants (tuned)
        double kP = 0.0019;   // Proportional gain
        double kI = 0.000085;   // Integral gain
        double kD = 0.0004;  // Derivative gain

        // Track error changes for derivative calculation
        double error = distance;

        long currentTime = System.currentTimeMillis();

        double deltaTime = (currentTime - previousTime) / 1000.0; // Convert ms to seconds

        if (deltaTime > 0) {
            integral += error * deltaTime;
            double maxIntegral = 1000; // Anti-windup cap
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        }

        double derivative = (deltaTime > 0) ? (error - previousError) / deltaTime : 0;

        // PID Control output
        double controlSignal = (kP * error) + (kI * integral) + (kD * derivative);

        // Non-linear gain scaling (exponential for smoother control)
        double minGain = 0.55;
        double maxGain = 1;
        double gain = minGain + (maxGain - minGain) * (1 - Math.exp(-distance / 30.0));

        // Axial (forward/backward) and lateral (sideways) adjustments
        double axial = -relDy / distance * 1.03;  // Move forward/backward to center
        double lateral = relDx / distance * 1.65; // Move sideways to center

        // Apply gain, speed, and control signal
        axial *= speed * gain * controlSignal;
        lateral *= speed * gain * controlSignal * 1.5;

        // Clamp values to ensure valid motor inputs
        axial = Math.max(-1, Math.min(1, axial));
        lateral = Math.max(-1, Math.min(1, lateral));

        double slowdownThreshold = 20.0;
        if (distance < slowdownThreshold) {
            double slowdownFactor = distance / slowdownThreshold;
            axial *= slowdownFactor;
        }

        // Create a PoseVelocity2d to pass to setDrivePowers
        PoseVelocity2d powers = new PoseVelocity2d(new Vector2d(axial / Robot.voltageSensor.getVoltage() * 12, -lateral / Robot.voltageSensor.getVoltage() * 12), 0);  // No rotation (ω) needed

        // Use setDrivePowers to control the robot movement
        drive.setDrivePowers(powers);

        // Update error tracking
        previousError = error;
        previousTime = currentTime;
    }

    public boolean MoveUsingCamera(MecanumDrive drive, boolean DoTransfer) {
        if (collectorSubsystem.CollectorClawClosed) {
            collectorSubsystem.ToggleCollectorClaw();
        }
        if (!collectorSubsystem.CollectorClawDown) {
            collectorSubsystem.CollectorClawPitchToggle();
        }
        threadSleep(100);

        Robot.autoPickupRelativeXOffset = SampleDetectorPipeline.getRelDx();
        Robot.autoPickupRelativeYOffset = SampleDetectorPipeline.getRelDy();
        Robot.autoPickupRelativeAngleOffset = SampleDetectorPipeline.getChosenAngle();

        if (Robot.autoPickupRelativeXOffset == 0 && Robot.autoPickupRelativeYOffset == 0) {
            return false;
        } else {
            // Move Robot to that position
            if (Math.abs(Robot.autoPickupRelativeXOffset) > 45 || Math.abs(Robot.autoPickupRelativeYOffset) > 50) {
                // Move the robot to the detected object
                moveToObject(
                        drive,
                        Robot.autoPickupRelativeXOffset,
                        Robot.autoPickupRelativeYOffset,
                        0.34 // Speed
                );
            } else {
                drive.setDrivePowers( new PoseVelocity2d(new Vector2d(0, 0), 0));
                targetReached = true;
                collectorSubsystem.ToggleCollectorClaw();
                threadSleep(350);
                if (DoTransfer) {
                    collectorSubsystem.CollectorMoveToClaw(false, false);
                }
                return true;
            }
        }
        return false;
    }

    private Action CollectUsingCollector(MecanumDrive drive, boolean AutoCollect, boolean AtAngle, boolean DoTransfer, boolean WaitBeforeClosingClaw, boolean ClawCollectionPointUp) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    collectorSubsystem.ToggleCollector();
                    threadSleep(350);
                    if (collectorSubsystem.CollectorClawClosed) {
                        Robot.collectorClawServo.setPosition(Robot.CollectorClawOpenPosition);
                        collectorSubsystem.CollectorClawClosed = false;
                    }
                    if (!collectorSubsystem.CollectorClawDown) {
                        collectorSubsystem.CollectorClawDown = true;
                        if (!AtAngle) {
                            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
                        }
                        if (ClawCollectionPointUp) {
                            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition + 0.05);
                            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition + 0.05);
                        } else {
                            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition);
                            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition);
                        }
                    }
                    if (AtAngle) {
                        Robot.collectorRotationServo.setPosition(0.5);
                    }
                    Looptimeout = 3000; // 1 second in milliseconds
                    LoopstartTime = System.currentTimeMillis();
                    while (!collectorSubsystem.CollectorPositionReached() && !isStopRequested()) {
                        if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                            break;
                        }
                        threadSleep(3);
                    }


                    SampleDetectorPipeline.setDetectBothRedAndBlue(true);
                    targetReached = true;
                    if (AutoCollect) {
                        boolean SampleCollected = false;
                        long startTime = System.currentTimeMillis();  // Get the current system time (milliseconds)
                        long timeout = 900;  // Timeout duration in milliseconds (2 seconds)
                        while (!SampleCollected && !isStopRequested()) {
                            if (System.currentTimeMillis() - startTime > timeout) {
                                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                                break;
                            }
                            SampleCollected = MoveUsingCamera(drive, DoTransfer);
                        }
                        if (System.currentTimeMillis() - startTime > timeout && !collectorSubsystem.CollectorClawClosed) {
                            if (WaitBeforeClosingClaw) {
                                threadSleep(300);
                            }
                            Robot.collectorClawServo.setPosition(Robot.CollectorClawClosePosition);
                            collectorSubsystem.CollectorClawClosed = true;
                            threadSleep(350);
                            if (DoTransfer) {
                                collectorSubsystem.CollectorMoveToClaw(false, false
                                );
                            }
                        }
                    } else {
                        if (ClawCollectionPointUp) {
                            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition);
                            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition);
                            threadSleep(50);
                        }
                        if (WaitBeforeClosingClaw) {
                            threadSleep(200);
                        }
                        Robot.collectorClawServo.setPosition(Robot.CollectorClawClosePosition);
                        collectorSubsystem.CollectorClawClosed = true;
                        threadSleep(350);
                        if (DoTransfer) {
                            collectorSubsystem.CollectorMoveToClaw(false, false);
                        }
                    }
                    SampleDetectorPipeline.setDetectBothRedAndBlue(false);
                    Looptimeout = 3000; // 1 second in milliseconds
                    LoopstartTime = System.currentTimeMillis();
                    while (collectorSubsystem.TransferActionLock && !isStopRequested()) {
                        if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                            break;
                        }
                        threadSleep(3);
                    }
                    threadSleep(300);
                    initialized = true;
                }
                return false;
            }
        };
    }

    private Action MoveAndReleaseSpecimen() {
        return new Action() {
            private boolean initialized  = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Robot.armFlipServoRight.setPosition(0.6);
                    Robot.armFlipServoLeft.setPosition(0.6);
                    Looptimeout = 3000; // 1 second in milliseconds
                    LoopstartTime = System.currentTimeMillis();
                    while (!collectorSubsystem.ArmFlipServoPositionReached() && !isStopRequested()) {
                        if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                            break;
                        }
                        threadSleep(1);
                    }
                    Robot.clawFlipServo.setPosition(0.8);
                    collectorSubsystem.ArmClawPositionIndex = 2;
                    threadSleep(350);
                    Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);
                    collectorSubsystem.ArmClawClosed = false;
                    initialized = true;
                }
                return false;
            }
        };
    }

    /** @noinspection RedundantThrows*/
    @Override
    public void runOpMode() throws InterruptedException {
        boolean initSuccess = Robot.init(true, true); // Adjust parameters as needed
        if (!initSuccess) {
            telemetry.addData("Status", "Initialization Failed!");
            telemetry.update();
            sleep(5000);
            return;
        } else {
            telemetry.addData("Status", "Ready!");
            telemetry.update();
        }

        Robot.ClawFlipServoDownPosition = 0.55;
        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);

        // Defining the starting pose
        Pose2d startingPose = new Pose2d(9, -58.4, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        // Keep track of the latest robot pose
        Pose2d currentPose = startingPose;

        // Getting the FTC Dashboard Instance
        ftcDashboard = FtcDashboard.getInstance();

        // Move to the initial scoring position
        TrajectoryActionBuilder goToSubmersible = drive.actionBuilder(currentPose)
                .strafeTo(new Vector2d(9, -27));

        Action clipFirstSpecimen = new SequentialAction(
            new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        collectorSubsystem.MoveMainArmToPosition(collectorSubsystem.ExtenderArmPosition_1);
                        collectorSubsystem.ExtenderArmPositionIndex = 1;
                        threadSleep(100);
                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoHighRungPosition);
                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoHighRungPosition);
                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoHighRungPosition);
                        collectorSubsystem.ArmClawPositionIndex = 3;
                        threadSleep(300);
                        initialized = true;
                    }
                    return false;
                }
            },
            goToSubmersible.build(),
            new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);
                        collectorSubsystem.ArmClawClosed = false;
                        threadSleep(150);
                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                        collectorSubsystem.ArmClawPositionIndex = 0;
                        threadSleep(200);
                        collectorSubsystem.MoveMainArmToPosition(collectorSubsystem.ExtenderArmPosition_0);
                        collectorSubsystem.ExtenderArmPositionIndex = 0;
                        initialized = true;
                    }
                    return false;
                }
            }
        );

        collectorSubsystem.ArmClawClosed = true;

        // Start the PIDF control loop
        collectorSubsystem.setOpModeActive(true);

        // Camera Setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(SampleDetectorPipeline);
        camera.setMillisecondsPermissionTimeout(2500);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                ftcDashboard.startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error:", "Camera failed to open with error code: %d", errorCode);
                telemetry.update();
            }
        });
        SampleDetectorPipeline.setDetectBothRedAndBlue(true);

        waitForStart();
        runtime.reset();

        Robot.ClawFlipServoDownPosition = 0.42;
        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);

        collectorSubsystem.startControlLoop();

        while (opModeIsActive() && !isStopRequested()) {
            if (isStopRequested()) {
                collectorSubsystem.setOpModeActive(false);
                collectorSubsystem.stopControlLoop();

                Thread.sleep(10);
                return;
            }

            drive.updatePoseEstimate();
            currentPose = drive.pose;

            TelemetryPacket packet = new TelemetryPacket();

            switch (currentState) {
                case START:
                    currentState = AutoState.INITIAL_SPECIMEN_SCORING;
                    break;

                case INITIAL_SPECIMEN_SCORING:
                    Actions.runBlocking(clipFirstSpecimen);
                    currentState = AutoState.MOVE_SAMPLES;
                    break;

                case MOVE_SAMPLES:
                    // First
                    TrajectoryActionBuilder GoToFirstSample = drive.actionBuilder(currentPose)
                        .setTangent(200)
                        .splineToLinearHeading(new Pose2d(46, -47, Math.toRadians(89.4)), Math.toRadians(0));
                    Action MoveFirstSpecimensAction = new SequentialAction(
                            GoToFirstSample.build(),
                            CollectUsingCollector(drive, true, false, true, false, false)
                    );
                    Actions.runBlocking(MoveFirstSpecimensAction);

                    // Second
                    drive.updatePoseEstimate();
                    TrajectoryActionBuilder GoToSecondSample = drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(57, -47, Math.toRadians(89.8)), Math.toRadians(0));
                    Action MoveSecondSpecimensAction = new SequentialAction(
                            GoToSecondSample.build(),
                            new ParallelAction(
                                MoveAndReleaseSpecimen(),
                                CollectUsingCollector(drive, true, false, true, false, false)
                            ),
                            MoveAndReleaseSpecimen()
                    );
                    Actions.runBlocking(MoveSecondSpecimensAction);

                    // Third
//                    drive.updatePoseEstimate();
//                    TrajectoryActionBuilder GoToThirdSample = drive.actionBuilder(drive.pose)
//                            .setTangent(90)
//                            .splineToLinearHeading(new Pose2d(50, -40, Math.toRadians(50.41)), Math.toRadians(200));
//                    Action MoveThirdSpecimensAction = new SequentialAction(
//                            GoToThirdSample.build(),
//                            CollectUsingCollector(drive, true, true, false, false, false),
//                            new Action() {
//                                private boolean initialized = false;
//
//                                @Override
//                                public boolean run(@NonNull TelemetryPacket packet) {
//                                    if (!initialized) {
//                                        collectorSubsystem.setDesiredCollectorPosition((double) Robot.CollectorMaxExtensionTics /2);
//                                        threadSleep(300);
//                                        initialized = true;
//                                    }
//                                    return false;
//                                }
//                            }
//                    );
//                    Actions.runBlocking(MoveThirdSpecimensAction);
//
//                    // Third Part 2
//                    drive.updatePoseEstimate();
//                    TrajectoryActionBuilder DropThirdSample = drive.actionBuilder(drive.pose)
//                            .setTangent(0)
//                            .splineToLinearHeading(new Pose2d(50, -39, Math.toRadians(-60)), Math.toRadians(0));
//                    Action DropThirdSpecimensAction = new SequentialAction(
//                            DropThirdSample.build(),
//                            new Action() {
//                                private boolean initialized = false;
//
//                                @Override
//                                public boolean run(@NonNull TelemetryPacket packet) {
//                                    if (!initialized) {
////                                        collectorSubsystem.setDesiredCollectorPosition(285);;
////                                        while (!collectorSubsystem.CollectorPositionReached()) {
////                                            threadSleep(3);
////                                        }
//                                        collectorSubsystem.CollectorClawDown = true;
//                                        Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
//                                        Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition);
//                                        Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition);
//                                        threadSleep(200);
//                                        collectorSubsystem.CollectorClawClosed = false;
//                                        Robot.collectorClawServo.setPosition(Robot.CollectorClawOpenPosition);
//                                        threadSleep(100);
//                                        collectorSubsystem.ToggleCollector();
//                                        while (!collectorSubsystem.CollectorPositionReached()) {
//                                            threadSleep(3);
//                                        }
//                                        initialized = true;
//                                    }
//                                    return false;
//                                }
//                            }
//                    );
//                    Actions.runBlocking(DropThirdSpecimensAction);

                    currentState = AutoState.COLLECT_FIRST_SPECIMEN;
                    break;

                case COLLECT_FIRST_SPECIMEN:
                    TrajectoryActionBuilder CollectFirstSpecimen = drive.actionBuilder(currentPose)
                        .setTangent(60)
                        .splineToLinearHeading(new Pose2d(19.9681, -44.1309, Math.toRadians(-23.9854)), Math.toRadians(200));

                    Action CollectFirstSpecimenAction = new SequentialAction(
                            CollectFirstSpecimen.build(),
                            CollectUsingCollector(drive, false, false, true, true, true)
                            );
                    Actions.runBlocking(CollectFirstSpecimenAction);
                    currentState = AutoState.PLACE_FIRST_SPECIMEN;
                    break;

                case PLACE_FIRST_SPECIMEN:
                    TrajectoryActionBuilder PlaceFirstSpecimen = drive.actionBuilder(currentPose)
                            .setTangent(60)
                            .splineToLinearHeading(new Pose2d(6, -27.5, Math.toRadians(270)), Math.toRadians(90));
                    Action PlaceFirstSpecimenAction = new SequentialAction(
                            new ParallelAction(
                                    PlaceFirstSpecimen.build(),
                                    new Action() {
                                        private boolean initialized = false;

                                        @Override
                                        public boolean run(@NonNull TelemetryPacket packet) {
                                            if (!initialized) {
                                                collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_1);
                                                collectorSubsystem.ExtenderArmPositionIndex = 1;
                                                threadSleep(150);
                                                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoHighRungPosition);
                                                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoHighRungPosition);
                                                Looptimeout = 1500; // 1 second in milliseconds
                                                LoopstartTime = System.currentTimeMillis();
                                                while (!collectorSubsystem.ArmFlipServoPositionReached() && !isStopRequested()) {
                                                    if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                        break;
                                                    }
                                                    threadSleep(1);
                                                }
                                                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoHighRungPosition);
                                                collectorSubsystem.ArmClawPositionIndex = 3;
                                                initialized = true;
                                            }
                                            return false;
                                        }
                                    }
                            ),
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);
                                        collectorSubsystem.ArmClawClosed = false;
                                        threadSleep(150);
                                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                                        Looptimeout = 1500; // 1 second in milliseconds
                                        LoopstartTime = System.currentTimeMillis();
                                        while (!collectorSubsystem.ArmFlipServoPositionReached() && !isStopRequested()) {
                                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                break;
                                            }
                                            threadSleep(1);
                                        }
                                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                                        collectorSubsystem.ArmClawPositionIndex = 0;
                                        threadSleep(200);
                                        collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_0);
                                        collectorSubsystem.ExtenderArmPositionIndex = 0;
                                        initialized = true;
                                    }
                                    return false;
                                }
                            }
                    );
                    Actions.runBlocking(PlaceFirstSpecimenAction);
                    currentState = AutoState.COLLECT_SECOND_SPECIMEN;
                    break;

                case COLLECT_SECOND_SPECIMEN:
                    TrajectoryActionBuilder CollectSecondSpecimen = drive.actionBuilder(currentPose)
                            .setTangent(180)
                            .splineToLinearHeading(new Pose2d(19.9681, -44.1309, Math.toRadians(-25.9854)), Math.toRadians(200));

                    Action CollectSecondSpecimenAction = new SequentialAction(
                            CollectSecondSpecimen.build(),
                            CollectUsingCollector(drive, false, false, true, true, true)
                    );
                    Actions.runBlocking(CollectSecondSpecimenAction);
                    currentState = AutoState.PLACE_SECOND_SPECIMEN;
                    break;

                case PLACE_SECOND_SPECIMEN:
                    TrajectoryActionBuilder PlaceSecondSpecimen = drive.actionBuilder(currentPose)
                            .setTangent(60)
                            .splineToLinearHeading(new Pose2d(3, -27.5, Math.toRadians(270)), Math.toRadians(90));
                    Action PlaceSecondSpecimenAction = new SequentialAction(
                            new ParallelAction(
                                PlaceSecondSpecimen.build(),
                                new Action() {
                                    private boolean initialized = false;

                                    @Override
                                    public boolean run(@NonNull TelemetryPacket packet) {
                                        if (!initialized) {
                                            collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_1);
                                            collectorSubsystem.ExtenderArmPositionIndex = 1;
                                            threadSleep(150);
                                            Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoHighRungPosition);
                                            Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoHighRungPosition);
                                            Looptimeout = 1500; // 1 second in milliseconds
                                            LoopstartTime = System.currentTimeMillis();
                                            while (!collectorSubsystem.ArmFlipServoPositionReached() && !isStopRequested()) {
                                                if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                    break;
                                                }
                                                threadSleep(1);
                                            }
                                            Robot.clawFlipServo.setPosition(Robot.ClawFlipServoHighRungPosition);
                                            collectorSubsystem.ArmClawPositionIndex = 3;
                                            initialized = true;
                                        }
                                        return false;
                                    }
                                }
                            ),
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);
                                        collectorSubsystem.ArmClawClosed = false;
                                        threadSleep(150);
                                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                                        Looptimeout = 1500; // 1 second in milliseconds
                                        LoopstartTime = System.currentTimeMillis();
                                        while (!collectorSubsystem.ArmFlipServoPositionReached() && !isStopRequested()) {
                                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                break;
                                            }
                                            threadSleep(1);
                                        }
                                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                                        collectorSubsystem.ArmClawPositionIndex = 0;
                                        threadSleep(200);
                                        collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_0);
                                        collectorSubsystem.ExtenderArmPositionIndex = 0;
                                        initialized = true;
                                    }
                                    return false;
                                }
                            }
                    );
                    Actions.runBlocking(PlaceSecondSpecimenAction);
                    currentState = AutoState.FINAL_PARK_SIDE;
                    break;

                case FINAL_PARK_SIDE:
                    //  Final parking position
                    TrajectoryActionBuilder finalParkSide = drive.actionBuilder(currentPose)
                            .setTangent(200)
                            .splineToLinearHeading(new Pose2d(47.4, -54, Math.toRadians(270)), Math.toRadians(0));
                    Action finalParkSideAction = new SequentialAction(
                            finalParkSide.build()
                    );
                    Actions.runBlocking(finalParkSideAction);

                    currentState = AutoState.FINISH;
                    break;

                case FINISH:
                    collectorSubsystem.setOpModeActive(false);
                    collectorSubsystem.stopControlLoop();

                    break;
            }
            ftcDashboard.sendTelemetryPacket(packet);
            if (currentState == AutoState.FINISH) {
                stop();
                break;
            }
        }
    }
}
