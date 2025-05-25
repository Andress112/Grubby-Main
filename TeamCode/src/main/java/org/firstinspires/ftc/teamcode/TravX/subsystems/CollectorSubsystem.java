package org.firstinspires.ftc.teamcode.TravX.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;

import java.util.HashMap;
import java.util.Map;

/** @noinspection ALL */
@Config
public class CollectorSubsystem {

    // Variables

    public final int ExtenderArmPosition_0 = 0; // Bottom position
    public final int ExtenderArmPosition_1 = 290; // high rung
    public final int ExtenderArmPosition_2 = 310; // Low basket
    public final int ExtenderArmPosition_3 = 850; // high basket
    public final int ExtenderArmPosition_4 = 850; // Hang Position

    public final int ExtenderArmPosition_Clipping_offset = 600; // When Clipping a piece on the rung it needs to go down a bit, this i the offset
    public final int MotorsPidsStatusTolerance = 5;

    // Don't touch!
    private Map<String, Long> lastActionMap = new HashMap<>();

    public volatile boolean CollectorExtended = false;
    public volatile boolean CollectorClawClosed = false;
    public volatile boolean CollectorClawDown = false;
    public volatile boolean ArmClawClosed = false;
    public volatile boolean ArmHasObject = false;
    public volatile boolean HangEngaged = false;
    public volatile boolean PowerSaversEngaged = true;

    public volatile boolean CollectorActionInProgress = false;
    public volatile boolean TransferActionLock = false;
    private volatile boolean MainActionLock = false;
    public volatile boolean HangActionLock = false;
    public volatile int ArmClawPositionIndex = 0;
    public volatile int ArmClawPositionLastIndex = 0;
    public volatile int ExtenderArmPositionIndex = 0;


    private TravXHardware Robot;

    // PIDF Controllers
    private PIDFController collectorMotorPID;
    private PIDFController armExtendMotorRightPID;
    private PIDFController armExtendMotorLeftPID;

    // PIDF Constants
    public static double COLLECTOR_KP = 0.007;
    public static double COLLECTOR_KI = 0.0015;
    public static double COLLECTOR_KD = 0.00048;
    public static double COLLECTOR_KF = 0;

    public static double ARM_KP = 0.01;
    public static double ARM_KI = 0.0000015;
    public static double ARM_KD = 0.00055;
    public static double ARM_KF = 0.0001;

    public static boolean coefficientsUpdated = false;
    public boolean ArmEncoderReseted = true;
    public boolean CollecotrEncoderReseted = true;

    // PIDF Toggles

    public static boolean useArmMotorPIDF  = true;
    public static boolean useCollectorMotorPIDF  = true;

    // Desired Setpoints
    private double desiredCollectorPosition = 0.0;
    private double desiredArmPosition = 0.0;

    // Control thread
    private Thread controlThread;
    private volatile boolean isControlThreadRunning = false;

    // Reference to OpMode active state
    private volatile boolean isOpModeActive = false;

    // timeout's
    long timeout = 1000; // 1 second in milliseconds
    long startTime = System.currentTimeMillis();

    public CollectorSubsystem(TravXHardware robotHardware) {
        this.Robot = robotHardware;

        // Initialize PIDF Controllers
        collectorMotorPID = new PIDFController(COLLECTOR_KP, COLLECTOR_KI, COLLECTOR_KD, COLLECTOR_KF);
        armExtendMotorRightPID = new PIDFController(ARM_KP, ARM_KI, ARM_KD, ARM_KF);
        armExtendMotorLeftPID = new PIDFController(ARM_KP, ARM_KI, ARM_KD, ARM_KF);

        // Set initial setpoints
        collectorMotorPID.setSetPoint(desiredCollectorPosition);
        armExtendMotorRightPID.setSetPoint(desiredArmPosition);
        armExtendMotorLeftPID.setSetPoint(desiredArmPosition);
    }
    // Public Functions

    public void startControlLoop() {
        if (isControlThreadRunning) return; // Prevent multiple threads

        useCollectorMotorPIDF = true;
        useArmMotorPIDF = true;
        isControlThreadRunning = true;
        controlThread = new Thread(new PIDFControlRunnable());
        controlThread.start();
    }

    // Stop the control thread
    public void stopControlLoop() {
        useCollectorMotorPIDF = false;
        useArmMotorPIDF = false;
        isControlThreadRunning = false;
        if (controlThread != null) {
            try {
                controlThread.join();
            } catch (InterruptedException e) {
                Robot.CurrentOpMode.telemetry.addData("Error", "Control thread interrupted!");
                Robot.CurrentOpMode.telemetry.update();
            }
        }
    }

    // Method to set the OpMode active state
    public void setOpModeActive(boolean active) {
        this.isOpModeActive = active;
    }
    // Public Methods to set desired positions
    public void setDesiredCollectorPosition(double position) {
        this.desiredCollectorPosition = position;
        if (position >= Robot.PIDFResetTolerance) {
            threadSleep(250);
            CollecotrEncoderReseted = false;
        }
        collectorMotorPID.setSetPoint(position);
    }
    public void setDesiredArmPosition(double position) {
        this.desiredArmPosition = position;
        if (position >= Robot.PIDFResetTolerance) {
            threadSleep(250);
            ArmEncoderReseted = false;
        }
        armExtendMotorRightPID.setSetPoint(position);
        armExtendMotorLeftPID.setSetPoint(position);
    }
    // Public Methods to get desired positions
    public double getDesiredCollectorPosition() {
        return this.desiredCollectorPosition;
    }
    public double getDesiredArmPosition() {
        return this.desiredArmPosition;
    }
    // Public Methods to get positions status
    public boolean CollectorPositionReached() {
        return ((Math.abs(this.desiredCollectorPosition) - Math.abs(Robot.collectorMotor.getCurrentPosition())) <= MotorsPidsStatusTolerance);
    }
    public boolean MainArmPositionReached() {
        return ((Math.abs(this.desiredArmPosition) - Math.abs(Robot.armExtendMotorRight.getCurrentPosition())) <= MotorsPidsStatusTolerance);
    }
    public boolean ArmFlipServoPositionReached() {
        return Math.abs((311 + (Robot.armFlipServoRight.getPosition() * (53.5 - 311))) - ((Robot.ArmFlipServoPosition.getVoltage() / 3.3) * 360)) <= 5;
    }
    public Thread ToggleCollectorClaw() {
        String actionKey = "ToggleCollectorClaw"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (currentTime - lastActionMap.getOrDefault(actionKey, 0L)) > Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread CollectorServoToggleThread = new Thread(new CollectorClawToggleRunnable());
            CollectorServoToggleThread.start();
            return CollectorServoToggleThread;
        }
        return null;
    }
    public Thread CollectorClawPitchToggle() {
        String actionKey = "CollectorClawPitchToggle"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread CollectorClawPitchToggleThread = new Thread(new CollectorClawPitchToggleRunnable());
            CollectorClawPitchToggleThread.start();
            return CollectorClawPitchToggleThread;
        }
        return null;
    }
    public Thread ToggleCollector() {
        String actionKey = "ToggleCollector"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            CollectorActionInProgress = false;
            if (CollectorExtended) {
                Thread RetractCollectorThread = new Thread(new RetractCollectorRunnable());
                RetractCollectorThread.start();
                return RetractCollectorThread;
            } else {
                Thread ExtendCollectorThread = new Thread(new ExtendCollectorRunnable());
                ExtendCollectorThread.start();
                return ExtendCollectorThread;
            }
        }
        return null;
    }
    public Thread ExtendCollectorHalfWayRunnable() {
        String actionKey = "ExtendCollectorHalfWayRunnable"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            CollectorActionInProgress = false;
            Thread ExtendCollectorHalfWayThread = new Thread(new ExtendCollectorHalfWayRunnable());
            ExtendCollectorHalfWayThread.start();
            return ExtendCollectorHalfWayThread;
        }
        return null;
    }
    public Thread CollectorMoveToClaw() {
        return CollectorMoveToClaw(true, false);
    }
    public Thread CollectorMoveToClaw(boolean moveClawBackAfterAction, boolean WaitMoreBeforeTranfer) {
        String actionKey = "CollectorMoveToClaw"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && CollectorClawClosed && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            TransferActionLock = true;
            lastActionMap.put(actionKey, currentTime);
            Thread CollectorMoveToClawThread = new Thread(new CollectorMoveToArmClawRunnable(moveClawBackAfterAction, WaitMoreBeforeTranfer));
            CollectorMoveToClawThread.start();
            return CollectorMoveToClawThread;
        }
        return null;
    }
    public Thread ReleaseClawObj() {
        String actionKey = "ReleaseClawObj"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread ReleaseClawObjectThread = new Thread(new ReleaseArmClawObjectRunnable());
            ReleaseClawObjectThread.start();
            return ReleaseClawObjectThread;
        }
        return null;
    }
    public Thread ChangeMainArmPosition() {
        String actionKey = "ChangeMainArmPosition"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread MoveArmToTheBackThread = new Thread(new MoveArmToTheBackRunnable());
            MoveArmToTheBackThread.start();
            return MoveArmToTheBackThread;
        }
        return null;
    }
    public Thread ChangeMainArmCollectionPosition() {
        String actionKey = "ChangeMainArmCollectionPosition"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread MoveArmToCollectionPositionThread = new Thread(new MoveArmToCollectionPositionToggle());
            MoveArmToCollectionPositionThread.start();
            return MoveArmToCollectionPositionThread;
        }
        return null;
    }
    public Thread ExtendMainArm() {
        String actionKey = "ExtendMainArm"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L)) > Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread ExtendMainArmThread = new Thread(new ExtendMainArmRunnable());
            ExtendMainArmThread.start();
            return ExtendMainArmThread;
        }
        return null;
    }
    public Thread RetractMainArm() {
        String actionKey = "RetractMainArm"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread RetractMainArmThread = new Thread(new RetractMainArmRunnable());
            RetractMainArmThread.start();
            return RetractMainArmThread;
        }
        return null;
    }
    public Thread ThrowUsingCollector() {
        String actionKey = "ThrowUsingCollector";
        long currentTime = System.currentTimeMillis();
        if (CollectorClawClosed && !MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread ThrowUsingCollectorThread = new Thread(new ThrowUsingCollectorRunnable());
            ThrowUsingCollectorThread.start();
            return ThrowUsingCollectorThread;
        }
        return null;
    }
    public Thread RetractMainArmAllTheWay() {
        String actionKey = "RetractMainArmAllTheWay"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread RetractMainArmAllTheWayThread = new Thread(new RetractMainArmAllTheWayRunnable());
            RetractMainArmAllTheWayThread.start();
            return RetractMainArmAllTheWayThread;
        }
        return null;
    }
    public Thread ExtendMainArmAllTheWay() {
        String actionKey = "ExtendMainArmAllTheWay"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread ExtendMainArmAllTheWayThread = new Thread(new ExtendMainArmAllTheWayRunnable());
            ExtendMainArmAllTheWayThread.start();
            return ExtendMainArmAllTheWayThread;
        }
        return null;
    }
    public Thread ToggleHangAction() {
        String actionKey = "ToggleHangAction"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            HangActionLock = true;
            Thread ToggleHangActioThread = new Thread(new ToggleHangActioRunnable());
            ToggleHangActioThread.start();
            return ToggleHangActioThread;
        }
        return null;
    }
    public Thread ResetCollectorSubsystem() {
        String actionKey = "ResetCollectorSubsystem"; // Unique key for this function
        long currentTime = System.currentTimeMillis();
        if (!MainActionLock && !TransferActionLock && !HangActionLock && (System.currentTimeMillis() - lastActionMap.getOrDefault(actionKey, 0L))> Robot.DebounceTime) {
            lastActionMap.put(actionKey, currentTime);
            Thread ResetCollectorSubsystemThread = new Thread(new ResetCollectorSubsystemRunnable());
            ResetCollectorSubsystemThread.start();
            return ResetCollectorSubsystemThread;
        }
        return null;
    }
    public void extendCollectorRecursively(double triggerValue) {
        if (!MainActionLock && !HangActionLock && !TransferActionLock) {
            // Calculate the increment based on trigger intensity
            double increment = triggerValue * Robot.COLLECTOR_EXTENSION_SPEED;

            // Update desiredCollectorPosition, ensuring it does not exceed the maximum limit
            desiredCollectorPosition = Range.clip(desiredCollectorPosition + increment, 0, Robot.CollectorMaxExtensionTics);

            if (ArmClawPositionIndex == 2 && desiredCollectorPosition >= 50) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                ArmClawPositionIndex = 1;
            }

            // Update the PIDF controller's setpoint
            collectorMotorPID.setSetPoint(desiredCollectorPosition);
            if (desiredCollectorPosition > 3) {
                CollectorExtended = true;
            }
        }
    }
    public void retractCollectorRecursively(double triggerValue) {
        if (!MainActionLock && !HangActionLock && !TransferActionLock) {
            // Calculate the decrement based on trigger intensity and loop delay
            double decrement = triggerValue * Robot.COLLECTOR_RETRACTION_SPEED;

            // Update desiredCollectorPosition, ensuring it does not go below the minimum limit
            desiredCollectorPosition = Range.clip(desiredCollectorPosition - decrement, 0, Robot.CollectorMaxExtensionTics);

            // Update the PIDF controller's setpoint
            collectorMotorPID.setSetPoint(desiredCollectorPosition);
            if (desiredCollectorPosition <= 2) {
                CollectorExtended = false;
            }
        }
    }
    // Private Functions

    private void threadSleep(long milliseconds) {
        try {
            for (int i = 0; i < milliseconds; i++) {
                if (Robot.CurrentOpMode.isStopRequested()) {
                    return;
                } else {
                    Thread.sleep(1);
                }
            }
        } catch (InterruptedException e) {
            return;
        }
    }
    private class PIDFControlRunnable implements Runnable {
        @Override
        public void  run() {
            // Initialize PIDF Controllers (in case constants have changed)
            collectorMotorPID.setPIDF(COLLECTOR_KP, COLLECTOR_KI, COLLECTOR_KD, COLLECTOR_KF);
            armExtendMotorRightPID.setPIDF(ARM_KP, ARM_KI, ARM_KD, ARM_KF);
            armExtendMotorLeftPID.setPIDF(ARM_KP, ARM_KI, ARM_KD, ARM_KF);

            collectorMotorPID.setTolerance(Robot.PIDFTolerance);
            armExtendMotorRightPID.setTolerance(Robot.PIDFTolerance);
            armExtendMotorLeftPID.setTolerance(Robot.PIDFTolerance);

            ElapsedTime runtime = new ElapsedTime();

            try {
                while (isControlThreadRunning && isOpModeActive && !Thread.currentThread().isInterrupted() && !Robot.CurrentOpMode.isStopRequested()) {
                    // Update PIDF coefficients in case they have been tuned via dashboard
                    if (coefficientsUpdated) {
                        collectorMotorPID.setPIDF(COLLECTOR_KP, COLLECTOR_KI, COLLECTOR_KD, COLLECTOR_KF);
                        armExtendMotorRightPID.setPIDF(ARM_KP, ARM_KI, ARM_KD, ARM_KF);
                        armExtendMotorLeftPID.setPIDF(ARM_KP, ARM_KI, ARM_KD, ARM_KF);

                        collectorMotorPID.setTolerance(Robot.PIDFTolerance);
                        armExtendMotorRightPID.setTolerance(Robot.PIDFTolerance);
                        armExtendMotorLeftPID.setTolerance(Robot.PIDFTolerance);

//                        coefficientsUpdated = false;
                    }

                    // Control lift Motor
                    if (Robot.LiftMotor.isBusy()) {
                        Robot.LiftMotor.setPower(1);
                    } else {
                        Robot.LiftMotor.setPower(0);
                    }

                    // Collector Motor Control
                    if (useCollectorMotorPIDF) {
                        double currentCollectorPos = Robot.collectorMotor.getCurrentPosition();
                        boolean CollectorMagneticSensorState = Robot.CollectorMagneticSensor.getState();
                        if (!CollectorMagneticSensorState && !CollecotrEncoderReseted) {
                            Robot.collectorMotorOffset = Math.abs(currentCollectorPos);
                            CollecotrEncoderReseted = true;
                            if (Robot.collectorMotorOffset < 0) {
                                Robot.collectorMotorOffset = 0;
                            }
                        }
                        double CollectorPos = currentCollectorPos - Robot.collectorMotorOffset;
                        if (Math.abs(CollectorPos) <= 3 && CollectorMagneticSensorState) {
                            Robot.collectorMotorOffset = 0;
                        }
                        double collectorPower = collectorMotorPID.calculate(CollectorPos);
                        if (Math.abs(collectorPower) <= 0.05) {
                            collectorPower = 0;
                        }
                        collectorPower = Range.clip(collectorPower, -1.0, 1.0);
                        if (!CollectorMagneticSensorState && (Math.abs(desiredCollectorPosition) - Math.abs(currentCollectorPos) <= MotorsPidsStatusTolerance) && !CollectorExtended && PowerSaversEngaged) {
                            Robot.collectorMotor.setPower(0);
                        } else {
                            Robot.collectorMotor.setPower(collectorPower * Robot.collectorMotorPower);
                        }
                    }

                    // Arm Extend Motor Control
                    if (useArmMotorPIDF) {
                        double currentArmPosRight = Robot.armExtendMotorRight.getCurrentPosition();
                        double currentArmPosLeft = Robot.armExtendMotorRight.getCurrentPosition();
                        boolean ArmMagneticSensorState = Robot.ArmMagneticSensor.getState();
                        if (!ArmMagneticSensorState && !ArmEncoderReseted) {
                            ArmEncoderReseted = true;
                            Robot.armExtendMotorOffset = Math.abs(currentArmPosLeft);
                            if (Robot.armExtendMotorOffset < 0) {
                                Robot.armExtendMotorOffset = 0;
                            }
                        }
                        currentArmPosRight = currentArmPosRight - Robot.armExtendMotorOffset;
                        currentArmPosLeft = currentArmPosLeft - Robot.armExtendMotorOffset;
                        if ((Math.abs(currentArmPosRight) <= 5 || Math.abs(currentArmPosLeft) <= 5) && ArmMagneticSensorState) {
                            Robot.armExtendMotorOffset = 0;
                        }
                        double armPowerRight = armExtendMotorRightPID.calculate(currentArmPosRight);
                        double armPowerLeft = armExtendMotorLeftPID.calculate(currentArmPosLeft);
                        armPowerRight = Range.clip(armPowerRight, -1.0, 1.0);
                        armPowerLeft = Range.clip(armPowerLeft, -1.0, 1.0);
                        if (ExtenderArmPositionIndex == 0) {
                            if (Math.abs(armPowerRight) <= 0.1) {
                                armPowerRight = 0;
                            }
                            if (Math.abs(armPowerLeft) <= 0.1) {
                                armPowerLeft = 0;
                            }
                        }
                        if (!ArmMagneticSensorState && ExtenderArmPositionIndex == 0 && PowerSaversEngaged) {
                            Robot.armExtendMotorRight.setPower(0); // Stop useless power draw
                            Robot.armExtendMotorLeft.setPower(0); // Stop useless power draw
                        } else {
                            if (Robot.armExtendMotorRight.getPower() < 0) {
                                Robot.armExtendMotorRight.setPower(armPowerRight * Robot.armExtendMotorPowerDown); // Descent Power
                                Robot.armExtendMotorLeft.setPower(armPowerLeft * Robot.armExtendMotorPowerDown); // Descent Power
                            } else {
                                Robot.armExtendMotorRight.setPower(armPowerRight * Robot.armExtendMotorPowerUp); // Rise Power
                                Robot.armExtendMotorLeft.setPower(armPowerLeft * Robot.armExtendMotorPowerUp); // Rise Power
                            }
                        }
                    }


                    // Sleep for loop delay
                    threadSleep(Robot.PID_LOOP_DELAY);
                }
            } catch (Exception e) {
                // Currently do nothing
            } finally {
                // Stop motors when control loop ends
                collectorMotorPID = null;
                armExtendMotorLeftPID = null;
                armExtendMotorRightPID = null;

                Robot.collectorMotor.setPower(0);
                Robot.armExtendMotorRight.setPower(0);
                Robot.armExtendMotorLeft.setPower(0);
                useCollectorMotorPIDF = false;
                useArmMotorPIDF = false;
            }
        }
    }
    public void MoveMainArmToPosition(int position) {
        setDesiredArmPosition(position);
    }
    private class CollectorClawToggleRunnable implements Runnable {
        @Override
        public void run() {
            if (CollectorClawClosed) {
                CollectorClawClosed = false;
                Robot.collectorClawServo.setPosition(Robot.CollectorClawOpenPosition);
                Robot.IndicatorLedRed.off();
                Robot.IndicatorLedGreen.on();
            } else {
                CollectorClawClosed = true;
                Robot.collectorClawServo.setPosition(Robot.CollectorClawClosePosition);
                Robot.IndicatorLedRed.on();
                Robot.IndicatorLedGreen.off();
            }
        }
    }
    private class CollectorClawPitchToggleRunnable implements Runnable {
        @Override
        public void run() {
            if (CollectorClawDown) {
                CollectorClawDown = false;
                Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
                Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchMidPointPosition);
                Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchMidPointPosition);
            } else {
                CollectorClawDown = true;
                Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
                Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition);
                Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition);
            }
        }
    }
    private class ExtendCollectorRunnable implements Runnable {
        @Override
        public void run() {
            CollectorActionInProgress = true;
            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchMidPointPosition);
            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchMidPointPosition);
            CollectorClawDown = false;

            setDesiredCollectorPosition(Robot.CollectorMaxExtensionTics);

            if (ArmClawPositionIndex != 0) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                ArmClawPositionIndex = 0;
            }

            timeout = 1500; // 1 second in milliseconds
            startTime = System.currentTimeMillis();
            while (!CollectorPositionReached() && CollectorActionInProgress && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(5);
            }

            CollectorActionInProgress = false;
            CollectorExtended = true;
        }
    }
    private class ExtendCollectorHalfWayRunnable implements Runnable {
        @Override
        public void run() {
            CollectorActionInProgress = true;
            setDesiredCollectorPosition(Robot.CollectorMaxExtensionTics);

            if (ArmClawPositionIndex != 0) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                ArmClawPositionIndex = 0;
            }

            CollectorClawDown = false;
            CollectorActionInProgress = false;
            CollectorExtended = true;
        }
    }
    private class RetractCollectorRunnable implements Runnable {
        @Override
        public void run() {
            CollectorActionInProgress = true;
            CollectorClawDown = false;
            setDesiredCollectorPosition(0);
            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchMidPointPosition);
            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchMidPointPosition);

            timeout = 1500; // 1 second in milliseconds
            startTime = System.currentTimeMillis();
            while (!CollectorPositionReached() && CollectorActionInProgress && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(5);
            }

            CollectorActionInProgress = false;
            CollectorExtended = false;
        }
    }
    private class CollectorMoveToArmClawRunnable implements Runnable {
        private final boolean moveClawBackAfterAction;
        private final boolean WaitMoreBeforeTranfer;

        // Constructor to accept the parameter
        public CollectorMoveToArmClawRunnable(boolean moveClawBackAfterAction, boolean WaitMoreBeforeTranfer) {
            this.moveClawBackAfterAction = moveClawBackAfterAction;
            this.WaitMoreBeforeTranfer = WaitMoreBeforeTranfer;
        }

        @Override
        public void run() {
            Robot.IndicatorLedGreen.on();
            Robot.IndicatorLedRed.on();
            PowerSaversEngaged = false;
            // Make sure the collector is in a know position
            Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);
            Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoTransitionPosition);
            Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoTransitionPosition);

            if (CollectorExtended) {
                setDesiredCollectorPosition(0);
            }
            if (ExtenderArmPositionIndex != 0) {
                ExtenderArmPositionIndex = 0;
                MoveMainArmToPosition(ExtenderArmPosition_0); // Make sure the main arm is all the way down
            }

            if (CollectorClawDown) {
                Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateLeftPosition);
            }
            threadSleep(50);
            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchTransitionPosition);
            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchTransitionPosition);

            threadSleep(30);
            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);

            timeout = 1000; // 1 second in milliseconds
            startTime = System.currentTimeMillis();
            while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(1);
            }
            threadSleep(100);

            Robot.clawFlipServo.setPosition(Robot.ClawFlipServoTransitionPosition);

            timeout = 2500; // 1 second in milliseconds
            startTime = System.currentTimeMillis();
            while (!CollectorPositionReached() || !MainArmPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(20);
            }
            ArmHasObject = false;

            threadSleep(100);
            if (WaitMoreBeforeTranfer) {
//                threadSleep(350);
            }

            if (ArmClawPositionIndex == 0) {
                threadSleep(250);
            } else {
                threadSleep(450);
            }

            if (CollectorExtended) {
                CollectorExtended = false;
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!CollectorPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(10);
                }
            }

            Robot.clawJawsServo.setPosition(Robot.ClawJawsClosePosition);
            threadSleep(50); // de verificat
            Robot.collectorClawServo.setPosition(Robot.CollectorClawOpenPosition);

            if (WaitMoreBeforeTranfer) {
                threadSleep(150); // de verificat
            }
            threadSleep(100); // de verificat

            if (moveClawBackAfterAction) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
            }
            TransferActionLock = false;

            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchUpPosition);
            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchUpPosition);

            Robot.IndicatorLedGreen.on();
            Robot.IndicatorLedRed.off();

            ArmClawPositionIndex = 1;
            PowerSaversEngaged = true;
            CollectorClawClosed = false;
            ArmHasObject = true;
            ArmClawClosed = true;
            CollectorClawDown = false;
        }
    }
    private class ExtendMainArmRunnable implements Runnable {
        @Override
        public void run() {
            int NewPosition = -1;
            switch (ExtenderArmPositionIndex) {
                case 0:
                    NewPosition = ExtenderArmPosition_1;

                    if (ArmClawClosed && ArmClawPositionIndex != 2) {
                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoHighRungPosition);
                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoHighRungPosition);
                        timeout = 1000; // 1 second in milliseconds
                        startTime = System.currentTimeMillis();
                        while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                            if (System.currentTimeMillis() - startTime >= timeout) {
                                break;
                            }
                            threadSleep(1);
                        }
                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoHighRungPosition);
                        ArmClawPositionIndex = 3;
                    }

                    ExtenderArmPositionIndex = 1;
                    break;
                case 1:
                    NewPosition = ExtenderArmPosition_2;
                    if (ArmClawClosed) {
                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                        timeout = 1000; // 1 second in milliseconds
                        startTime = System.currentTimeMillis();
                        while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                            if (System.currentTimeMillis() - startTime >= timeout) {
                                break;
                            }
                            threadSleep(1);
                        }
                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                        ArmClawPositionIndex = 1;
                    }
                    ExtenderArmPositionIndex = 2;
                    break;
                case 2:
                    NewPosition = ExtenderArmPosition_3;
                    if (ArmClawClosed) {
                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                        timeout = 1000; // 1 second in milliseconds
                        startTime = System.currentTimeMillis();
                        while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                            if (System.currentTimeMillis() - startTime >= timeout) {
                                break;
                            }
                            threadSleep(1);
                        }
                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                        ArmClawPositionIndex = 1;
                    }
                    ExtenderArmPositionIndex = 3;
                    break;
            }
            if (NewPosition == -1) {
                return;
            }

            MoveMainArmToPosition(NewPosition);
        }
    }

    private class RetractMainArmRunnable implements Runnable {
        @Override
        public void run() {
            int NewPosition = -100;
            switch (ExtenderArmPositionIndex) {
                case 1:
                    NewPosition = ExtenderArmPosition_0;

                    Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                    Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                    timeout = 1000; // 1 second in milliseconds
                    startTime = System.currentTimeMillis();
                    while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                        if (System.currentTimeMillis() - startTime >= timeout) {
                            break;
                        }
                        threadSleep(1);
                    }
                    Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                    ArmClawPositionIndex = 0;

                    ExtenderArmPositionIndex = 0;
                    break;
                case 2:
                    NewPosition = ExtenderArmPosition_1;

                    if (ArmClawClosed) {
                        Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoHighRungPosition);
                        Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoHighRungPosition);
                        timeout = 1000; // 1 second in milliseconds
                        startTime = System.currentTimeMillis();
                        while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                            if (System.currentTimeMillis() - startTime >= timeout) {
                                break;
                            }
                            threadSleep(1);
                        }
                        Robot.clawFlipServo.setPosition(Robot.ClawFlipServoHighRungPosition);
                        ArmClawPositionIndex = 3;
                    }

                    ExtenderArmPositionIndex = 1;
                    break;
                case 3:
                    NewPosition = ExtenderArmPosition_2;
                    ExtenderArmPositionIndex = 2;
                    break;
            }
            if (NewPosition == -100) {
                return;
            }
            MoveMainArmToPosition(NewPosition);
        }
    }
    private class ThrowUsingCollectorRunnable implements Runnable {
        @Override
        public void run() {
            CollectorActionInProgress = true;
            CollectorClawDown = false;
            if (CollectorExtended) {
                setDesiredCollectorPosition(0);
                timeout = 2000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!CollectorPositionReached() && CollectorActionInProgress && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(5);
                }
            }
            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchMidPointPosition);
            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchMidPointPosition);
            setDesiredCollectorPosition(Robot.CollectorMaxExtensionTics);

            if (ArmClawPositionIndex != 0) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                timeout = 500; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                ArmClawPositionIndex = 0;
            }

            timeout = 2000;
            startTime = System.currentTimeMillis();
            while (!((Math.abs(desiredCollectorPosition - 30) - Math.abs(Robot.collectorMotor.getCurrentPosition())) <= MotorsPidsStatusTolerance) && CollectorActionInProgress && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(5);
            }
            Robot.collectorClawServo.setPosition(Robot.CollectorClawOpenPosition);
            CollectorClawClosed = false;

            threadSleep(300);

            setDesiredCollectorPosition(0);
            timeout = 2000;
            startTime = System.currentTimeMillis();
            while (!CollectorPositionReached() && CollectorActionInProgress && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(5);
            }

            CollectorActionInProgress = false;
            CollectorExtended = false;
        }
    }
    private class ReleaseArmClawObjectRunnable implements Runnable {
        @Override
        public void run() {
            if (ArmClawClosed) {
                Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);
                ArmClawClosed = false;
            } else {
                Robot.clawJawsServo.setPosition(Robot.ClawJawsClosePosition);
                ArmClawClosed = true;
            }
        }
    }
    private class MoveArmToTheBackRunnable implements Runnable {
        @Override
        public void run() {
            if (ArmClawPositionIndex != 0) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                ArmClawPositionIndex = 0;
            } else {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoBackPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoBackPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoBackPosition);
                ArmClawPositionIndex = 2;
            }
        }
    }
    private class MoveArmToCollectionPositionToggle implements Runnable {
        @Override
        public void run() {
            if (ArmClawPositionIndex != 3) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoHighRungPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoHighRungPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoHighRungPosition);
                ArmClawPositionIndex = 3;
            } else {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                ArmClawPositionIndex = 1;
            }
        }
    }
    private class RetractMainArmAllTheWayRunnable implements Runnable {
        @Override
        public void run() {
            setDesiredArmPosition(ExtenderArmPosition_0);

            Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
            Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
            timeout = 1000; // 1 second in milliseconds
            startTime = System.currentTimeMillis();
            while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(1);
            }
            Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
            ArmClawPositionIndex = 0;

            ExtenderArmPositionIndex = 0;
        }
    }
    private class ExtendMainArmAllTheWayRunnable implements Runnable {
        @Override
        public void run() {
            setDesiredArmPosition(ExtenderArmPosition_3);

            if (ArmClawClosed && ArmClawPositionIndex != 2) {
                Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoMidPosition);
                Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoMidPosition);
                timeout = 1000; // 1 second in milliseconds
                startTime = System.currentTimeMillis();
                while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        break;
                    }
                    threadSleep(1);
                }
                Robot.clawFlipServo.setPosition(Robot.ClawFlipServoMidPosition);
                ArmClawPositionIndex = 1;
            }

            ExtenderArmPositionIndex = 3;
        }
    }
    private class ToggleHangActioRunnable implements Runnable {
        @Override
        public void run() {
            HangActionLock = true;
            if (Math.abs(Robot.HangMotorDownPosition - Robot.LiftMotor.getCurrentPosition()) <= 100 && !HangEngaged) {
                Robot.CurrentOpMode.gamepad2.setLedColor(0, 0, 255, 9999999);  // Blue
                Robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.LiftMotor.setTargetPosition(0);
                Robot.LiftMotor.setTargetPositionTolerance(5);
                Robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Robot.LiftMotor.setPower(1);
            } else {
                if (!HangEngaged) {
                    Robot.CurrentOpMode.gamepad2.setLedColor(0, 255, 0, 9999999);  // Green
                    // Set Servos positions
                    Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                    Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                    Robot.clawJawsServo.setPosition(Robot.ClawJawsClosePosition);
                    timeout = 1000; // 1 second in milliseconds
                    startTime = System.currentTimeMillis();
                    while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                        if (System.currentTimeMillis() - startTime >= timeout) {
                            break;
                        }
                        threadSleep(1);
                    }
                    Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                    ArmClawPositionIndex = 1;
                    // Put collector up
                    Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
                    Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchUpPosition);
                    Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchUpPosition);
                    CollectorClawDown = false;
                    setDesiredArmPosition(-100);
                    threadSleep(100);
                    timeout = 3000; // 1 second in milliseconds
                    startTime = System.currentTimeMillis();
                    while (Robot.ArmMagneticSensor.getState() && !Robot.CurrentOpMode.isStopRequested()) {
                        if (System.currentTimeMillis() - startTime >= timeout) {
                            break;
                        }
                        threadSleep(5);
                    }
                    // Engage Hang servo
                    Robot.HangServo.setPosition(Robot.HangServoClosePosition);
                    threadSleep(300);
                    setDesiredArmPosition(ExtenderArmPosition_0);
                    // Raise Arm to the max position and the coil to the max
                    Robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Robot.LiftMotor.setTargetPosition(0);
                    Robot.LiftMotor.setTargetPositionTolerance(5);
                    Robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.LiftMotor.setPower(1);

                    setDesiredArmPosition(ExtenderArmPosition_4);
                    ExtenderArmPositionIndex = 4;
                    timeout = 3000; // 1 second in milliseconds
                    startTime = System.currentTimeMillis();
                    while (Robot.LiftMotor.isBusy() || !MainArmPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                        if (System.currentTimeMillis() - startTime >= timeout) {
                            break;
                        }
                        threadSleep(5);
                    }
                    HangEngaged = true;
                } else {
                    Robot.CurrentOpMode.gamepad2.setLedColor(255, 0, 0, 9999999);  // Red
                    // Set Servos positions
                    Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
                    Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);
                    Robot.clawJawsServo.setPosition(Robot.ClawJawsClosePosition);
                    timeout = 1000; // 1 second in milliseconds
                    startTime = System.currentTimeMillis();
                    while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                        if (System.currentTimeMillis() - startTime >= timeout) {
                            break;
                        }
                        threadSleep(1);
                    }
                    Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
                    ArmClawPositionIndex = 1;
                    // Put collector up
                    Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
                    Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchUpPosition);
                    Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchUpPosition);
                    CollectorClawDown = false;
                    // Reset Hang servo
                    Robot.HangServo.setPosition(Robot.HangServoOpenPosition);
                    setDesiredArmPosition(ExtenderArmPosition_0);
                    ExtenderArmPositionIndex = 0;
                    threadSleep(100);
                    // Engage Hang Motor
                    Robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Robot.LiftMotor.setTargetPosition(Robot.HangMotorDownPosition);
                    Robot.LiftMotor.setTargetPositionTolerance(5);
                    Robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Robot.LiftMotor.setPower(1);
                    timeout = 3000; // 1 second in milliseconds
                    startTime = System.currentTimeMillis();
                    while (Robot.LiftMotor.isBusy() && !Robot.CurrentOpMode.isStopRequested()) {
                        if (System.currentTimeMillis() - startTime >= timeout) {
                            break;
                        }
                        threadSleep(5);
                    }
                    HangEngaged = false;
                    HangActionLock = false;
                }
            }
        }
    }
    private class ResetCollectorSubsystemRunnable implements Runnable {
        @Override
        public void run() {
            MainActionLock = true;
            // Reset all the variables to default
            CollectorExtended = false;
            CollectorClawClosed = false;
            CollectorClawDown = true;
            ArmClawClosed = false;
            ArmHasObject = false;

            CollectorActionInProgress = false;
            TransferActionLock = false;
            ArmClawPositionIndex = 0;
            ArmClawPositionLastIndex = 0;
            ExtenderArmPositionIndex = 0;

            ArmEncoderReseted = false;
            CollecotrEncoderReseted = false;

            Robot.collectorMotorOffset = 0;
            Robot.armExtendMotorOffset = 0;

            // arm
            Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);
            Robot.clawFlipServo.setPosition(Robot.ClawFlipServoDownPosition);
            Robot.armFlipServoRight.setPosition(Robot.ArmFlipServoDownPosition);
            timeout = 1000; // 1 second in milliseconds
            startTime = System.currentTimeMillis();
            while (!ArmFlipServoPositionReached() && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                threadSleep(1);
            }
            Robot.armFlipServoLeft.setPosition(Robot.ArmFlipServoDownPosition);

            // collector
            Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
            Robot.collectorClawServo.setPosition(Robot.CollectorClawOpenPosition);
            Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchDownPosition);
            Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchDownPosition);

            useCollectorMotorPIDF = false;
            useArmMotorPIDF = false;

            // Finished flags
            boolean armFinished = false;
            boolean collectorFinished = false;

            if (!Robot.ArmMagneticSensor.getState()) {
                armFinished = true;
                Robot.armExtendMotorRight.setPower(0);
                Robot.armExtendMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                Robot.armExtendMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                Robot.armExtendMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                Robot.armExtendMotorLeft.setPower(0);
                Robot.armExtendMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                Robot.armExtendMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                Robot.armExtendMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                Robot.armExtendMotorRight.setPower(-1);
                Robot.armExtendMotorLeft.setPower(-1);
            }

            if (!Robot.CollectorMagneticSensor.getState()) {
                collectorFinished = true;
                Robot.collectorMotor.setPower(0);
                Robot.collectorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                Robot.collectorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                Robot.collectorMotor.setPower(-1);
            }

            timeout = 5000; // 5 second in milliseconds
            startTime = System.currentTimeMillis();
            while (!collectorFinished || !armFinished && !Robot.CurrentOpMode.isStopRequested()) {
                if (System.currentTimeMillis() - startTime >= timeout) {
                    break;
                }
                // Gradually increase motor power for arm motor if target is not reached
                if (!armFinished && Robot.ArmMagneticSensor.getState()) {
                    Robot.armExtendMotorRight.setPower(-1);
                }

                // Gradually increase motor power for collector motor if target is not reached
                if (!collectorFinished && Robot.CollectorMagneticSensor.getState()) {
                    Robot.collectorMotor.setPower(-1);
                }

                // Check the arm touch sensor
                if (!armFinished) { // Corrected condition
                    if (!Robot.ArmMagneticSensor.getState()) {
                        Robot.armExtendMotorRight.setPower(0);
                        Robot.armExtendMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        Robot.armExtendMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        Robot.armExtendMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        Robot.armExtendMotorLeft.setPower(0);
                        Robot.armExtendMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        Robot.armExtendMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        Robot.armExtendMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        if (!Robot.ArmMagneticSensor.getState()) {
                            Robot.armExtendMotorOffset = Robot.armExtendMotorLeft.getCurrentPosition();
                        }
                        armFinished = true; // Set the flag to true
                    }
                }

                // Check the collector touch sensor
                if (!collectorFinished) { // Corrected condition
                    if (!Robot.CollectorMagneticSensor.getState()) {
                        Robot.collectorMotor.setPower(0);
                        Robot.collectorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        Robot.collectorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                        if (!Robot.CollectorMagneticSensor.getState()) {
                            Robot.collectorMotorOffset = Robot.collectorMotor.getCurrentPosition();
                        }
                        collectorFinished = true; // Set the flag to true
                    }
                }
                // Exit loop when both actions are complete
                if (collectorFinished && armFinished) {
                    break;
                }
            }

            setDesiredCollectorPosition(0);
            setDesiredArmPosition(ExtenderArmPosition_0);

            useCollectorMotorPIDF = true;
            useArmMotorPIDF = true;

            MainActionLock = false;
        }
    }
}