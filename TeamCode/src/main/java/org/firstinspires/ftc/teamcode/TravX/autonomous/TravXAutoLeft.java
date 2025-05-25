package org.firstinspires.ftc.teamcode.TravX.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
@Autonomous(name = "TravX Auto Left", group = "Autonomous", preselectTeleOp = "TravX Main")
public class TravXAutoLeft extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam camera;

    FtcDashboard ftcDashboard;

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
        INITIAL_SAMPLE_SCORING,
        PICK_FIRST_SAMPLE,
        PLACE_FIRST_SAMPLE,
        PICK_SECOND_SAMPLE,
        PLACE_SECOND_SAMPLE,
        PICK_THIRD_SAMPLE,
        PLACE_THIRD_SAMPLE,
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
    public void timerSleep(double milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.milliseconds() < milliseconds && !isStopRequested()) {
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

    public boolean MoveUsingCamera(MecanumDrive drive) {
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
                collectorSubsystem.CollectorMoveToClaw(false, true);
                Looptimeout = 1000; // 1 second in milliseconds
                LoopstartTime = System.currentTimeMillis();
                while (collectorSubsystem.TransferActionLock && !isStopRequested()) {
                    if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                        break;
                    }
                    threadSleep(1);
                }
                return true;
            }
        }
        return false;
    }

    private Action CollectUsingCollector(MecanumDrive drive, boolean HalfExtend, boolean AtAngle, long timeout, boolean extendWithColectorDown) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    if (HalfExtend) {
                        collectorSubsystem.CollectorActionInProgress = true;
                        if (!extendWithColectorDown) {
                            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchUpPosition);
                            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchUpPosition);
                            collectorSubsystem.CollectorClawDown = false;
                        } else {
                            if (AtAngle) {
                                Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition);
                                Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition);
                            } else {
                                Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition + 0.007);
                                Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition + 0.007);
                            }
                        }

                        collectorSubsystem.setDesiredCollectorPosition(185);

                        Looptimeout = 2000; // 1 second in milliseconds
                        LoopstartTime = System.currentTimeMillis();
                        while (!collectorSubsystem.CollectorPositionReached() && collectorSubsystem.CollectorActionInProgress && !isStopRequested()) {
                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                break;
                            }
                            threadSleep(10);
                        }
                        if (AtAngle) {
                            Robot.collectorRotationServo.setPosition(0.16);
                        }

                        collectorSubsystem.CollectorActionInProgress = false;
                        collectorSubsystem.CollectorExtended = true;
                    } else {
                        collectorSubsystem.ToggleCollector();
                    }
                    threadSleep(300);
                    if (collectorSubsystem.CollectorClawClosed) {
                        collectorSubsystem.ToggleCollectorClaw();
                    }
                    if (!collectorSubsystem.CollectorClawDown) {
                        collectorSubsystem.CollectorClawDown = true;
                        if (!extendWithColectorDown) {
                            if (AtAngle) {
                                Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition);
                                Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition);
                            } else {
                                Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition + 0.007);
                                Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition + 0.007);
                            }
                        }
                    }
                    Looptimeout = 2000; // 1 second in milliseconds
                    LoopstartTime = System.currentTimeMillis();
                    while (!collectorSubsystem.CollectorPositionReached() && !isStopRequested()) {
                        if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                            break;
                        }
                        threadSleep(3);
                    }
                    if (AtAngle) {
                        Robot.collectorRotationServo.setPosition(0.16);
                    }

                    SampleDetectorPipeline.setSamplePickupColorIndex(0);
                    targetReached = true;
                    boolean SampleCollected = false;
                    long startTime = System.currentTimeMillis();  // Get the current system time (milliseconds)
                    while (!SampleCollected && !isStopRequested()) {
                        if (System.currentTimeMillis() - startTime > timeout) {
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                            break;
                        }
                        SampleCollected = MoveUsingCamera(drive);
                    }
                    if (!SampleCollected && (System.currentTimeMillis() - startTime > timeout && !collectorSubsystem.CollectorClawClosed)) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        Robot.collectorClawServo.setPosition(Robot.CollectorClawClosePosition);
                        collectorSubsystem.CollectorClawClosed = true;
                        threadSleep(350);
                        collectorSubsystem.CollectorMoveToClaw(false, true);
                        Looptimeout = 1000; // 1 second in milliseconds
                        LoopstartTime = System.currentTimeMillis();
                        while (collectorSubsystem.TransferActionLock && !isStopRequested()) {
                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                break;
                            }
                            threadSleep(1);
                        }
                        Robot.clawJawsServo.setPosition(Robot.ClawJawsClosePosition);
                        threadSleep(350);
                    }
                    threadSleep(300);
                    Looptimeout = 1500; // 1 second in milliseconds
                    LoopstartTime = System.currentTimeMillis();
                    while (!collectorSubsystem.CollectorPositionReached() && !isStopRequested()) {
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
        Pose2d startingPose = new Pose2d(-35, -64, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        // Keep track of the latest robot pose
        Pose2d currentPose = startingPose;

        // Getting the FTC Dashboard Instance
        ftcDashboard = FtcDashboard.getInstance();

        collectorSubsystem.ArmClawClosed = true;

        // Start the PIDF control loop
        collectorSubsystem.setOpModeActive(true);
        SampleDetectorPipeline.setSamplePickupColorIndex(0);

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

        waitForStart();
        runtime.reset();

        Robot.ClawFlipServoDownPosition = 0.42;

        collectorSubsystem.startControlLoop();

        while (opModeIsActive() && !isStopRequested()) {
            if (isStopRequested()) {
                return;
            }

            drive.updatePoseEstimate();
            currentPose = drive.pose;

            switch (currentState) {
                case START:
                    currentState = AutoState.INITIAL_SAMPLE_SCORING;
                    break;

                case INITIAL_SAMPLE_SCORING:
                    TrajectoryActionBuilder goToBasketZone1 = drive.actionBuilder(currentPose)
                            .setTangent(Math.toRadians(120))
                            .splineToLinearHeading(new Pose2d(-51.7, -58.9, Math.toRadians(45.67)), Math.toRadians(180), new TranslationalVelConstraint(30.0), new ProfileAccelConstraint(-20.0, 30.0));
                    Action  dropSampleInHighBasketAction1 = new SequentialAction(
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_3);
                                        collectorSubsystem.ExtenderArmPositionIndex = 3;
                                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                                        collectorSubsystem.ArmClawPositionIndex = 1;
                                        Robot.clawFlipServo.setPosition(1);
                                        Looptimeout = 1500; // 1 second in milliseconds
                                        LoopstartTime = System.currentTimeMillis();
                                        while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                break;
                                            }
                                            threadSleep(3);
                                        }
                                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            },
                            goToBasketZone1.build(),
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.ReleaseClawObj();
                                        threadSleep(300);
                                        Robot.clawFlipServo.setPosition(1);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            }

                    );
                    Actions.runBlocking(dropSampleInHighBasketAction1);
                    currentState = AutoState.PICK_FIRST_SAMPLE;
                    break;

                case PICK_FIRST_SAMPLE:
                    TrajectoryActionBuilder goToSampleOne = drive.actionBuilder(currentPose)
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-45.8, -55, Math.toRadians(92.5)), Math.toRadians(180));
                    Action collectSampleOne = new SequentialAction(
                            goToSampleOne.build(),
                            new ParallelAction(
                                new Action() {
                                    private boolean initialized = false;

                                    @Override
                                    public boolean run(@NonNull TelemetryPacket packet) {
                                        if (!initialized) {
                                            collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_0);
                                            collectorSubsystem.ExtenderArmPositionIndex = 0;
                                            Looptimeout = 2500; // 1 second in milliseconds
                                            LoopstartTime = System.currentTimeMillis();
                                            while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                                if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                    break;
                                                }
                                                threadSleep(3);
                                            }
                                            Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                                            Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                                            Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                                            collectorSubsystem.ArmClawPositionIndex = 0;

                                            initialized = true;
                                        }
                                        return false;
                                    }
                                },
                                CollectUsingCollector(drive, false, false, 900, false)
                            )
                    );
                    Actions.runBlocking(collectSampleOne);

                    currentState = AutoState.PLACE_FIRST_SAMPLE;
                    break;

                case PLACE_FIRST_SAMPLE:
                    TrajectoryActionBuilder goToBasketZone2 = drive.actionBuilder(currentPose)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-51.7, -58.9, Math.toRadians(45.67)), Math.toRadians(180));
                    Action  dropSampleInHighBasketAction2 = new SequentialAction(
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_3);
                                        collectorSubsystem.ExtenderArmPositionIndex = 3;
                                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                                        collectorSubsystem.ArmClawPositionIndex = 1;
                                        Robot.clawFlipServo.setPosition(1);
                                        threadSleep(300);
                                        Looptimeout = 2500; // 1 second in milliseconds
                                        LoopstartTime = System.currentTimeMillis();
                                        while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                break;
                                            }
                                            threadSleep(3);
                                        }
                                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                                        threadSleep(200);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            },
                            goToBasketZone2.build(),
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.ReleaseClawObj();
                                        threadSleep(150);
                                        Robot.clawFlipServo.setPosition(1);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            }

                    );
                    Actions.runBlocking(dropSampleInHighBasketAction2);

                    currentState = AutoState.PICK_SECOND_SAMPLE;
                    break;

                case PICK_SECOND_SAMPLE:
                    TrajectoryActionBuilder goToSampleTwo = drive.actionBuilder(currentPose)
                            .splineToLinearHeading(new Pose2d(-52, -47.5, Math.toRadians(94.5)), Math.toRadians(180));
                    Action collectSampleTwo = new SequentialAction(
                            goToSampleTwo.build(),
                            new ParallelAction(
                                    new Action() {
                                        private boolean initialized = false;

                                        @Override
                                        public boolean run(@NonNull TelemetryPacket packet) {
                                            if (!initialized) {
                                                collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_0);
                                                collectorSubsystem.ExtenderArmPositionIndex = 0;
                                                Looptimeout = 2500; // 1 second in milliseconds
                                                LoopstartTime = System.currentTimeMillis();
                                                while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                                    if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                        break;
                                                    }
                                                    threadSleep(3);
                                                }
                                                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                                                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                                                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                                                collectorSubsystem.ArmClawPositionIndex = 0;

                                                initialized = true;
                                            }
                                            return false;
                                        }
                                    },
                                    CollectUsingCollector(drive, true, false, 600, false)
                            )
                    );
                    Actions.runBlocking(collectSampleTwo);

                    currentState = AutoState.PLACE_SECOND_SAMPLE;
                    break;

                case PLACE_SECOND_SAMPLE:
                    TrajectoryActionBuilder goToBasketZone3 = drive.actionBuilder(currentPose)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(new Pose2d(-51.7, -58.9, Math.toRadians(45.67)), Math.toRadians(270));
                    Action  dropSampleInHighBasketAction3 = new SequentialAction(
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_3);
                                        collectorSubsystem.ExtenderArmPositionIndex = 3;
                                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                                        collectorSubsystem.ArmClawPositionIndex = 1;
                                        Robot.clawFlipServo.setPosition(1);
                                        threadSleep(300);
                                        Looptimeout = 2500; // 1 second in milliseconds
                                        LoopstartTime = System.currentTimeMillis();
                                        while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                break;
                                            }
                                            threadSleep(3);
                                        }
                                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                                        threadSleep(200);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            },
                            goToBasketZone3.build(),
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.ReleaseClawObj();
                                        threadSleep(150);
                                        Robot.clawFlipServo.setPosition(1);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            }

                    );
                    Actions.runBlocking(dropSampleInHighBasketAction3);
                    currentState = AutoState.PICK_THIRD_SAMPLE;

                    break;

                case PICK_THIRD_SAMPLE:
                    TrajectoryActionBuilder goToSampleThree = drive.actionBuilder(currentPose)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-49, -37.7, Math.toRadians(150)), Math.toRadians(180));
                    Action collectSampleThree = new SequentialAction(
                            goToSampleThree.build(),
                            new ParallelAction(
                                    new Action() {
                                        private boolean initialized = false;

                                        @Override
                                        public boolean run(@NonNull TelemetryPacket packet) {
                                            if (!initialized) {
                                                collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_0);
                                                collectorSubsystem.ExtenderArmPositionIndex = 0;
                                                Looptimeout = 2500; // 1 second in milliseconds
                                                LoopstartTime = System.currentTimeMillis();
                                                collectorSubsystem.ArmClawPositionIndex = 0;
                                                while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                                    if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                        break;
                                                    }
                                                    threadSleep(3);
                                                }
                                                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                                                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                                                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                                                initialized = true;
                                            }
                                            return false;
                                        }
                                    },
                                    CollectUsingCollector(drive, true, true, 1300, true)
                            )
                    );
                    Actions.runBlocking(collectSampleThree);

                    currentState = AutoState.PLACE_THIRD_SAMPLE;
                    break;

                case PLACE_THIRD_SAMPLE:
                    TrajectoryActionBuilder goToBasketZone4 = drive.actionBuilder(currentPose)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(new Pose2d(-51.7, -58.9, Math.toRadians(45.67)), Math.toRadians(0));
                    Action  dropSampleInHighBasketAction4 = new SequentialAction(
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_3);
                                        collectorSubsystem.ExtenderArmPositionIndex = 3;
                                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                                        collectorSubsystem.ArmClawPositionIndex = 1;
                                        Robot.clawFlipServo.setPosition(1);
                                        threadSleep(300);
                                        Looptimeout = 3000; // 1 second in milliseconds
                                        LoopstartTime = System.currentTimeMillis();
                                        while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                break;
                                            }
                                            threadSleep(3);
                                        }
                                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                                        threadSleep(200);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            },
                            goToBasketZone4.build(),
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        collectorSubsystem.ReleaseClawObj();
                                        threadSleep(150);
                                        Robot.clawFlipServo.setPosition(1);
                                        initialized = true;
                                    }
                                    return false;
                                }
                            },
                            new Action() {
                                private boolean initialized = false;

                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    if (!initialized) {
                                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                                        collectorSubsystem.ArmClawPositionIndex = 1;
                                        Robot.clawFlipServo.setPosition(1);
                                        threadSleep(800);
                                        Robot.clawJawsServo.setPosition(Robot.ClawJawsClosePosition);
                                        collectorSubsystem.setDesiredArmPosition(collectorSubsystem.ExtenderArmPosition_0);
                                        collectorSubsystem.ExtenderArmPositionIndex = 0;
                                        Looptimeout = 2000; // 1 second in milliseconds
                                        LoopstartTime = System.currentTimeMillis();
                                        while (!collectorSubsystem.MainArmPositionReached() && !isStopRequested()) {
                                            if (System.currentTimeMillis() - LoopstartTime >= Looptimeout) {
                                                break;
                                            }
                                            threadSleep(3);
                                        }
                                        initialized = true;
                                    }
                                    return false;
                                }
                            }

                    );
                    Actions.runBlocking(dropSampleInHighBasketAction4);
                    currentState = AutoState.FINISH;

                    break;

                case FINISH:
                    collectorSubsystem.setOpModeActive(false);
                    collectorSubsystem.stopControlLoop();

                    camera.stopStreaming();
                    camera.closeCameraDevice();
                    ftcDashboard.stopCameraStream();

                    stop();
                    break;
            }
            if (currentState == AutoState.FINISH) {
                break;
            }
        }
    }
}