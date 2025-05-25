package org.firstinspires.ftc.teamcode.TravX.teleop;

import static java.lang.Math.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TravX.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.TravX.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.TravX.utilities.SampleDetector;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

/** @noinspection RedundantThrows*/
@TeleOp(name = "TravX Main", group = "TravXMain")
public class TravXTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime GameRuntime = new ElapsedTime();

    FtcDashboard ftcDashboard;
    Telemetry dashboardTelemetry;
    OpenCvWebcam camera;

    // Getting the hardware initialized
    private final TravXHardware Robot = new TravXHardware(this);
    private long LastAction = 0;
    private long LastHangAction = 0;

    // Initializing the subsystems
    private final Drivetrain drivetrain = new Drivetrain(Robot);
    private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(Robot);
    private final SampleDetector SampleDetectorPipeline = new SampleDetector();

    private static boolean IsAutoPickUpOn = false;
    private static boolean UpdateServoPosition = false;
    public static boolean IsAutoWheelControlEnabled = false;
    private static boolean LastIsAutoPickUpOn = false;

    private boolean IsBlueAlliance = true;
    private int autonomousCollectColorIndex = 0; // 0 - Yellow, 1 - Blue, 2 - Red
    private int lastAutonomousCollectColorIndex = -1;
    private long autonomousCollectionStartTime = 0;
    private long autonomousCollectionStartTimeCamera = 0;

    // Rumble
    private boolean GameStarted = false;
    private boolean halfTimeRumble =  false;
    private boolean endGameRumble =  false;

    /** @noinspection BusyWait*/
    @Override
    public void runOpMode() throws InterruptedException {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        // Getting the FTC Dashboard Instance
        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        // camera Rumble
        Gamepad.RumbleEffect cameraRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 150)
                .build();

        Gamepad.RumbleEffect cameraStopRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 150)
                .addStep(0, 0, 250)
                .addStep(1, 1, 150)
                .build();

        // Half time Rumble
        Gamepad.RumbleEffect halfTimeRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 500)
                .addStep(0, 0, 250)
                .addStep(1, 1, 500)
                .build();

        // Endgame Rumble
        Gamepad.RumbleEffect endGameRubleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 400)
                .addStep(0, 0, 250)
                .addStep(1, 1, 400)
                .addStep(0, 0, 350)
                .addStep(1, 0, 300)
                .addStep(0, 0, 200)
                .addStep(0, 1, 400)
                .build();

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

        // Waiting for the match to start
        waitForStart();
        runtime.reset();

        // Initializing the Robot
        boolean initSuccess = Robot.init(true, false);
        if (!initSuccess) {
            sleep(10000);
            return;
        }

        // Start the PIDF control loop
        collectorSubsystem.setOpModeActive(true);
        collectorSubsystem.startControlLoop();

        // Starting the drivetrain thread!
        drivetrain.startWheelControl(this);

        Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            // Rumble Timing
            if (GameRuntime.seconds() <= 150) {
                if ((!GameStarted) && drivetrain.hasRobotMoved()) {
                    GameStarted = true;
                    GameRuntime.reset();
                }
                if (GameStarted) {
                    double HALF_TIME = 60.0;
                    if (((GameRuntime.seconds()-1) >= HALF_TIME) && (!halfTimeRumble)) {
                        halfTimeRumble = true;
                        gamepad1.runRumbleEffect(halfTimeRumbleEffect);
                        gamepad2.runRumbleEffect(halfTimeRumbleEffect);
                    }
                    double END_GAME = 135.0;
                    if (((GameRuntime.seconds()-1) >= END_GAME) && (!endGameRumble)) {
                        endGameRumble = true;
                        gamepad1.runRumbleEffect(endGameRubleEffect);
                        gamepad2.runRumbleEffect(endGameRubleEffect);
                    }
                }
            }

            // Detect user input

            // Gamepad 1 / Main Driver

            if (gamepad1.left_stick_button) {
                drivetrain.ToggleMotorPower();
            }
            if (gamepad1.right_stick_button) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    LastAction = System.currentTimeMillis();

                    IsBlueAlliance = !IsBlueAlliance;
                    if (autonomousCollectColorIndex == 1) {
                        autonomousCollectColorIndex = 2;
                    } else if (autonomousCollectColorIndex == 2) {
                        autonomousCollectColorIndex = 1;
                    }
                }
            }
            if (gamepad1.options) {
                drivetrain.ToggleDriveTrain();
            }
            if (gamepad1.dpad_right) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    LastAction = System.currentTimeMillis();
                    autonomousCollectionStartTime = System.currentTimeMillis();
                    autonomousCollectionStartTimeCamera = System.currentTimeMillis();
                    IsAutoWheelControlEnabled = true;
                    Robot.isAutoMovementActive = true;
                    IsAutoPickUpOn = !IsAutoPickUpOn;
                    UpdateServoPosition = true;
                    if (IsAutoPickUpOn) {
                        gamepad1.runRumbleEffect(cameraRumble);
                    }
                }
            }
            if (gamepad1.left_bumper) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    autonomousCollectColorIndex = 0;
                    LastAction = System.currentTimeMillis();
                    autonomousCollectionStartTime = System.currentTimeMillis();
                    autonomousCollectionStartTimeCamera = System.currentTimeMillis();
                    IsAutoWheelControlEnabled = false;
                    IsAutoPickUpOn = !IsAutoPickUpOn;
                    UpdateServoPosition = true;
                    if (IsAutoPickUpOn) {
                        gamepad1.runRumbleEffect(cameraRumble);
                    }
                }
            }
            if (gamepad1.right_bumper) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    if (IsBlueAlliance) {
                        autonomousCollectColorIndex = 1;
                    } else {
                        autonomousCollectColorIndex = 2;
                    }
                    LastAction = System.currentTimeMillis();
                    autonomousCollectionStartTime = System.currentTimeMillis();
                    autonomousCollectionStartTimeCamera = System.currentTimeMillis();
                    IsAutoWheelControlEnabled = false;
                    IsAutoPickUpOn = !IsAutoPickUpOn;
                    UpdateServoPosition = true;
                    if (IsAutoPickUpOn) {
                        gamepad1.runRumbleEffect(cameraRumble);
                    }
                }
            }
            if (gamepad1.cross || gamepad1.a) {
                collectorSubsystem.ToggleCollector();
            }
            if (gamepad1.square || gamepad1.x) {
                collectorSubsystem.ToggleCollectorClaw();
            }
            if (gamepad1.circle || gamepad1.b) {
                if (IsAutoPickUpOn) {
                    IsAutoPickUpOn = false;
                    UpdateServoPosition = false;
                }
                collectorSubsystem.CollectorMoveToClaw();
            }
            if (gamepad1.triangle || gamepad1.y) {
                collectorSubsystem.CollectorClawPitchToggle();
            }
            if (gamepad1.left_trigger > 0.01) {
                collectorSubsystem.retractCollectorRecursively(gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger > 0.01) {
                collectorSubsystem.extendCollectorRecursively(gamepad1.right_trigger);
            }
            if (gamepad1.dpad_up) {
                collectorSubsystem.ResetCollectorSubsystem();
            }
            if (gamepad1.dpad_left) {
                if ((System.currentTimeMillis() - LastHangAction) > Robot.DebounceTime) {
                    LastHangAction = System.currentTimeMillis();
                    drivetrain.ReverseControls(!drivetrain.getReverseControls());
                }
            }
            if (gamepad1.dpad_down) {
                collectorSubsystem.ThrowUsingCollector();
            }
            if (gamepad1.touchpad) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    LastAction = System.currentTimeMillis();

                    if (IsBlueAlliance) {
                        if (autonomousCollectColorIndex == 0) {
                            autonomousCollectColorIndex = 1;
                        } else {
                            autonomousCollectColorIndex = 0;
                        }
                    } else {
                        if (autonomousCollectColorIndex == 0) {
                            autonomousCollectColorIndex = 2;
                        } else {
                            autonomousCollectColorIndex = 0;
                        }
                    }
                }
            }

            // Gamepad 2 / Second Driver

            if (gamepad2.circle || gamepad2.b) {
                collectorSubsystem.ChangeMainArmPosition();
            }
            if (gamepad2.left_bumper) {
                collectorSubsystem.RetractMainArm();
            }
            if (gamepad2.right_bumper) {
                collectorSubsystem.ExtendMainArm();
            }
            if (gamepad2.dpad_down) {
                collectorSubsystem.RetractMainArmAllTheWay();
            }
            if (gamepad2.dpad_up) {
                collectorSubsystem.ExtendMainArmAllTheWay();
            }
            if (gamepad2.cross || gamepad2.a) {
                collectorSubsystem.ReleaseClawObj();
            }
            if (gamepad2.triangle || gamepad2.y) {
                collectorSubsystem.ChangeMainArmCollectionPosition();
            }
            if (gamepad2.touchpad) {
                if ((System.currentTimeMillis() - LastAction) > 2500) {
                    LastAction = System.currentTimeMillis();
                    collectorSubsystem.ToggleHangAction();
                }
            }

            // Change gamepad color for the selected autonomous pickup color
            if (autonomousCollectColorIndex != lastAutonomousCollectColorIndex || IsAutoPickUpOn != LastIsAutoPickUpOn) {
                lastAutonomousCollectColorIndex = autonomousCollectColorIndex;
                LastIsAutoPickUpOn = IsAutoPickUpOn;
                if (autonomousCollectColorIndex == 0) {
                    SampleDetectorPipeline.setSamplePickupColorIndex(0);
                    if (IsAutoPickUpOn) {
                        gamepad1.setLedColor(0, 225, 0, 9999999); // Yellow
                    } else {
                        gamepad1.setLedColor(255, 225, 0, 9999999);
                    }
                } else if (autonomousCollectColorIndex == 1) {
                    SampleDetectorPipeline.setSamplePickupColorIndex(1);
                    if (IsAutoPickUpOn) {
                        gamepad1.setLedColor(0, 255, 255, 9999999);  // Blue
                    } else {
                        gamepad1.setLedColor(0, 0, 255, 9999999);
                    }
                } else {
                    SampleDetectorPipeline.setSamplePickupColorIndex(2);
                    if (IsAutoPickUpOn) {
                        gamepad1.setLedColor(255, 0, 85, 9999999); // Red
                    } else {
                        gamepad1.setLedColor(255, 0, 0, 9999999);
                    }
                }
            }

            // Auto Collector
            if (IsAutoPickUpOn) {
                if (collectorSubsystem.CollectorClawClosed) {
                    collectorSubsystem.CollectorClawClosed = false;
                    Robot.collectorClawServo.setPosition(Robot.CollectorClawOpenPosition);
                    Robot.IndicatorLedRed.off();
                    Robot.IndicatorLedGreen.on();
                }
                if (!collectorSubsystem.CollectorClawDown) {
                    collectorSubsystem.CollectorClawPitchToggle();
                }

                Robot.autoPickupRelativeXOffset = SampleDetectorPipeline.getRelDx();
                Robot.autoPickupRelativeYOffset = SampleDetectorPipeline.getRelDy();
                Robot.autoPickupRelativeAngleOffset = SampleDetectorPipeline.getChosenAngle();

                if (IsAutoPickUpOn && UpdateServoPosition && (System.currentTimeMillis() - autonomousCollectionStartTimeCamera > 150)) {
                    if (!IsAutoWheelControlEnabled) {
                        IsAutoPickUpOn = false;
                        UpdateServoPosition = false;
                    } else {
                        UpdateServoPosition = false;
                    }
                }

                if (Robot.autoPickupRelativeXOffset == 0 && Robot.autoPickupRelativeYOffset == 0) {
                    telemetry.addLine("No object detected");
                    telemetry.update();
                    dashboardTelemetry.addLine("No object detected");
                    dashboardTelemetry.update();
                    if (IsAutoWheelControlEnabled && (System.currentTimeMillis() - autonomousCollectionStartTime > Robot.autoPickupTimeBeforeReturningControlToDriver)) {
                        IsAutoWheelControlEnabled = false;
                        drivetrain.targetReached = true;
                        Robot.isAutoMovementActive = false;
                        IsAutoPickUpOn = false;
                        drivetrain.stopAllMotors();
                        gamepad1.runRumbleEffect(cameraStopRumble);
                    }
                } else {
                    telemetry.addLine("object detected!");
                    telemetry.update();
                    dashboardTelemetry.addLine("object detected!");
                    dashboardTelemetry.addData("Object X Offset", Robot.autoPickupRelativeXOffset);
                    dashboardTelemetry.addData("Object Y Offset", Robot.autoPickupRelativeYOffset);
                    dashboardTelemetry.addData("Object Angle", Robot.autoPickupRelativeAngleOffset);
                    dashboardTelemetry.update();
                    autonomousCollectionStartTime = System.currentTimeMillis();

                    if (!collectorSubsystem.CollectorClawClosed && collectorSubsystem.CollectorClawDown && UpdateServoPosition) {
                        // Adjust the angle if needed (it should be between 0 and 180 degrees)
                        double adjustedAngle = Math.max(0, Math.min(180, Robot.autoPickupRelativeAngleOffset));

                        // Check if the change in angle exceeds the threshold
                        if (Robot.prevAdjustedAngle >= 170 && adjustedAngle <= 15) {
                            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateRightPosition);
                        } else if (Robot.prevAdjustedAngle <= 15 && adjustedAngle >= 170) {
                            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateLeftPosition);
                        } else {
                            // Map the angle to the servo position range [Robot.CollectorClawRotateRightPosition, Robot.CollectorClawRotateLeftPosition]
                            double servoPosition = Robot.CollectorClawRotateRightPosition -
                                    (adjustedAngle / 180.0) *
                                            (Robot.CollectorClawRotateRightPosition - Robot.CollectorClawRotateLeftPosition);

                            // Update the servo position
                            Robot.collectorRotationServo.setPosition(servoPosition);
                        }
                        // Save the current angle as the previous adjusted angle
                        Robot.prevAdjustedAngle = adjustedAngle;
                    }

                    // Move Robot to that position
                    if (IsAutoWheelControlEnabled) {
                        if (Math.abs(Robot.autoPickupRelativeXOffset) > 35 || Math.abs(Robot.autoPickupRelativeYOffset) > 45) {
                            // Move the robot to the detected object
                            drivetrain.moveToObject(
                                    Robot.autoPickupRelativeXOffset,
                                    Robot.autoPickupRelativeYOffset,
                                    Robot.autoPickupMotorPower // Speed
                            );
                        } else {
                            drivetrain.targetReached = true;
                            IsAutoPickUpOn = false;
                            collectorSubsystem.ToggleCollectorClaw();
                            Thread.sleep(320);
                            collectorSubsystem.CollectorMoveToClaw();
                            Robot.isAutoMovementActive = false;
                            drivetrain.stopAllMotors();

                        }
                    } else if (Robot.isAutoMovementActive) {
                        drivetrain.targetReached = true;
                        Robot.isAutoMovementActive = false;
                        drivetrain.stopAllMotors();
                    }
                }
            } else {
                if (Robot.isAutoMovementActive) {
                    drivetrain.targetReached = true;
                    Robot.isAutoMovementActive = false;
                    drivetrain.stopAllMotors();
                }
            }

            // Telemetry
            telemetry.addData("Run Time:", runtime.toString());
            telemetry.addData("Drive Train Active", Robot.DriveTrainSwitch);
            telemetry.addData("Drive Train Power", round(Robot.PowerPercentageValue * 100) + "%");
            telemetry.addData("Main Arm position index", collectorSubsystem.ExtenderArmPositionIndex);
//            telemetry.addData("Collector encoder Position", Robot.collectorMotor.getCurrentPosition());
//            telemetry.addData("Desired collector position", collectorSubsystem.getDesiredCollectorPosition());
//            telemetry.addData("Desired arm position", collectorSubsystem.getDesiredArmPosition());
//            telemetry.addData("Main arm power", Robot.armExtendMotorRight.getPower());
//            telemetry.addData("Collector power", Robot.collectorMotor.getPower());
//            telemetry.addData("Relative Y Offset", Robot.autoPickupRelativeXOffset);
//            telemetry.addData("Relative X Offset", Robot.autoPickupRelativeYOffset);
//            telemetry.addData("Angle", Robot.autoPickupRelativeAngleOffset);
//            telemetry.addData("Auto Pick Up", IsAutoPickUpOn);
            telemetry.addData("CollecotrEncoderReseted", collectorSubsystem.CollecotrEncoderReseted);
            telemetry.addData("ArmEncoderReseted", collectorSubsystem.ArmEncoderReseted);
            telemetry.addData("collectorMotorOffset", Robot.collectorMotorOffset);
            telemetry.addData("armExtendMotorOffset", Robot.armExtendMotorOffset);
            telemetry.update();

//            dashboardTelemetry.addData("Run Time:", runtime.toString());
//            dashboardTelemetry.addData("Drive Train Active", Robot.DriveTrainSwitch);
//            dashboardTelemetry.addData("Drive Train Power", round(Robot.PowerPercentageValue * 100) + "%");
//            dashboardTelemetry.addData("Main Arm position index", collectorSubsystem.ExtenderArmPositionIndex);
//            dashboardTelemetry.addData("Main arm power", Robot.armExtendMotor.getPower());

//            dashboardTelemetry.addData("Collector encoder Position", Robot.collectorMotor.getCurrentPosition());
//            dashboardTelemetry.addData("Desired collector position", collectorSubsystem.getDesiredCollectorPosition());
//            dashboardTelemetry.addData("Desired main arm position", collectorSubsystem.getDesiredArmPosition());
//            dashboardTelemetry.addData("Main arm right encoder Position", Robot.armExtendMotorRight.getCurrentPosition());
//            dashboardTelemetry.addData("Main arm left encoder Position", Robot.armExtendMotorLeft.getCurrentPosition());

//            dashboardTelemetry.addData("Collector power", Robot.collectorMotor.getPower());
//            dashboardTelemetry.addData("Relative Y Offset", Robot.autoPickupRelativeXOffset);
//            dashboardTelemetry.addData("Relative X Offset", Robot.autoPickupRelativeYOffset);
//            dashboardTelemetry.addData("Angle", Robot.autoPickupRelativeAngleOffset);
//            dashboardTelemetry.addData("Auto Pick Up", IsAutoPickUpOn);
//            dashboardTelemetry.addData("TransferActionLock", collectorSubsystem.TransferActionLock);
//            dashboardTelemetry.update();
        }

        // Stop the wheel control when the op mode is no longer active
        drivetrain.stopWheelControl();
        collectorSubsystem.setOpModeActive(false);
        collectorSubsystem.stopControlLoop();

        camera.stopStreaming();
        camera.closeCameraDevice();
        ftcDashboard.stopCameraStream();
    }
}
