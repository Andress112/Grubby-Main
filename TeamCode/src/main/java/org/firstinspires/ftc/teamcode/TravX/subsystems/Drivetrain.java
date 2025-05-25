package org.firstinspires.ftc.teamcode.TravX.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;

/** @noinspection unused*/
public class Drivetrain {
    private final TravXHardware robot;
    private WheelControl wheelControlRunnable;
    private Thread wheelControlThread;
    private volatile boolean isWheelControlActive = false;

    private long lastTime_DriveTrainSwitched = 0;
    private long lastTime_DriveTrainPowerSwitched = 0;

    private volatile boolean Switch_Drivetrain = true;
    private volatile boolean Switch_PowerMode = true;

    // Auto Collector variables
    private double previousError = 0;
    private long previousTime = 0;
    private double integral = 0;
    public boolean targetReached = false;

    public Drivetrain(TravXHardware robot) {
        this.robot = robot;
    }

    public void startWheelControl(LinearOpMode opMode) {
        if (!isWheelControlActive) {
            isWheelControlActive = true;
            wheelControlRunnable = new WheelControl(robot, opMode);
            wheelControlThread = new Thread(wheelControlRunnable);
            wheelControlThread.start();
        }
    }

    public void stopWheelControl() {
        if (isWheelControlActive) {
            isWheelControlActive = false;
            if (wheelControlThread != null) {
                wheelControlRunnable.stop();
                try {
                    wheelControlThread.join(); // Wait for the thread to finish
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    public boolean hasRobotMoved() {
        if (wheelControlRunnable != null) {
            return wheelControlRunnable.hasRobotMoved();
        }
        return false;
    }

    public void resetRobotMovedFlag() {
        if (wheelControlRunnable != null) {
            wheelControlRunnable.RobotHasBeenMovedUsingTheWheels = false;
        }
    }

    public void ReverseControls(boolean reversed) {
        wheelControlRunnable.ReverseControls(reversed);
    }

    public boolean getReverseControls() {
        return wheelControlRunnable.RobotControlsBackwards;
    }

    public void ToggleDriveTrain() {
        if (Switch_Drivetrain) {
            if (System.currentTimeMillis() - lastTime_DriveTrainSwitched > robot.DebounceTime) {
                Switch_Drivetrain = false; // Set the flag to prevent rapid button presses

                Thread DriveTrainSwitchThread = new Thread(new ToggleDrivetrain());
                DriveTrainSwitchThread.start();

                lastTime_DriveTrainSwitched = System.currentTimeMillis(); // Update the last press time
            }
        }
    }
    public void ToggleMotorPower() {
        if (Switch_PowerMode) {
            if (System.currentTimeMillis() - lastTime_DriveTrainPowerSwitched > robot.DebounceTime) {
                Switch_PowerMode = false; // Set the flag to prevent rapid button presses

                Thread MotorPowerSwitchThread = new Thread(new ToggleMotorPower());
                MotorPowerSwitchThread.start();

                lastTime_DriveTrainPowerSwitched = System.currentTimeMillis(); // Update the last press time
            }
        }
    }


    private class ToggleDrivetrain implements Runnable {
        @Override
        public void run() {
            robot.DriveTrainSwitch = !robot.DriveTrainSwitch;
            Switch_Drivetrain = true;
        }
    }
    private class ToggleMotorPower implements Runnable {
        @Override
        public void run() {
            if (robot.PowerPercentageValue == robot.MotorFastPower) {
                robot.PowerPercentageValue = robot.MotorCrawlPower;
                robot.YawnGain = 0.8;
            } else {
                robot.PowerPercentageValue = robot.MotorFastPower;
                robot.YawnGain = 1;
            }
            Switch_PowerMode = true;
        }
    }
    public void stopAllMotors() {
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }
    /** @noinspection UnnecessaryLocalVariable*/
    public void moveToObject(int relDx, int relDy, double speed) {
        // Reset the function variables when its called again after it has been used
        if (targetReached) {
            previousTime = 0;
            previousError = 0;
            integral = 0;
            targetReached = false;
        }

        if (!robot.isAutoMovementActive) {
            robot.isAutoMovementActive = true;
        }

        // Normalize the distance and direction
        double distance = Math.sqrt(relDx * relDx + relDy * relDy);

        if (distance < 5) { // Adjust threshold as needed
            stopAllMotors();
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
        double axial = -relDy / distance * 1.025;  // Move forward/backward to center
        double lateral = relDx / distance * 1.6; // Move sideways to center

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

        // Mecanum wheel formulas (no yaw, only axial and lateral movement)
        double leftFrontPower = axial + lateral;
        double rightFrontPower = axial - lateral;
        double leftBackPower = axial - lateral;
        double rightBackPower = axial + lateral;

        // Normalize motor powers
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Slew rate limiting (smooth transitions)
//        double maxPowerChange = 0.3; // Max change per cycle
//
//        leftFrontPower = clampPower(robot.leftFrontDrive.getPower(), leftFrontPower, maxPowerChange);
//        rightFrontPower = clampPower(robot.rightFrontDrive.getPower(), rightFrontPower, maxPowerChange);
//        leftBackPower = clampPower(robot.leftBackDrive.getPower(), leftBackPower, maxPowerChange);
//        rightBackPower = clampPower(robot.rightBackDrive.getPower(), rightBackPower, maxPowerChange);

        // Set the motor powers
        robot.leftFrontDrive.setPower(leftFrontPower / robot.voltageSensor.getVoltage() * 12);
        robot.rightFrontDrive.setPower(rightFrontPower  / robot.voltageSensor.getVoltage() * 12);
        robot.leftBackDrive.setPower(leftBackPower  / robot.voltageSensor.getVoltage() * 12);
        robot.rightBackDrive.setPower(rightBackPower  / robot.voltageSensor.getVoltage() * 12);

        // Update error tracking
        previousError = error;
        previousTime = currentTime;
    }

    // Helper method for slew rate limiting
    private double clampPower(double currentPower, double targetPower, double maxChange) {
        if (targetPower > currentPower + maxChange) {
            return currentPower + maxChange;
        } else return Math.max(targetPower, currentPower - maxChange);
    }
}

