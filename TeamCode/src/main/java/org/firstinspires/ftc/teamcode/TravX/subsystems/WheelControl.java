package org.firstinspires.ftc.teamcode.TravX.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;

/** @noinspection BusyWait*/
public class WheelControl implements Runnable {
    private final TravXHardware robot;
    private final LinearOpMode opMode;
    private volatile boolean isActive = true; // Control the running state
    public boolean RobotHasBeenMovedUsingTheWheels = false;
    public boolean RobotControlsBackwards = false;

    public final double deadzone = 0.02;

    public WheelControl(TravXHardware robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while (isActive && !Thread.currentThread().isInterrupted() && !opMode.isStopRequested()) {
            if (!robot.isAutoMovementActive) {

                double axial = (Math.abs(opMode.gamepad1.left_stick_y) > deadzone) ? -opMode.gamepad1.left_stick_y * robot.axialGain : 0;
                double lateral = (Math.abs(opMode.gamepad1.left_stick_x) > deadzone) ? opMode.gamepad1.left_stick_x * robot.lateralGain : 0;
                double yaw = (Math.abs(opMode.gamepad1.right_stick_x) > deadzone) ? opMode.gamepad1.right_stick_x * robot.YawnGain : 0;


                // Speed scaling based on distance from center
                double speedFactor = Math.sqrt(axial * axial + lateral * lateral + yaw * yaw);
                double scale = Math.min(1.0, speedFactor);

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                double leftFrontPower;
                double rightFrontPower;
                double leftBackPower;
                double rightBackPower;

                if (!RobotControlsBackwards) {
                    // Normal controls
                    leftFrontPower = (axial + lateral + yaw) * robot.PowerPercentageValue * scale;
                    rightFrontPower = (axial - lateral - yaw) * robot.PowerPercentageValue * scale;
                    leftBackPower = (axial - lateral + yaw) * robot.PowerPercentageValue * scale;
                    rightBackPower = (axial + lateral - yaw) * robot.PowerPercentageValue * scale;
                } else {
                    // Controls adjusted for reversed orientation
                    leftFrontPower = (-axial - lateral + yaw) * robot.PowerPercentageValue * scale;
                    rightFrontPower = (-axial + lateral - yaw) * robot.PowerPercentageValue * scale;
                    leftBackPower = (-axial + lateral + yaw) * robot.PowerPercentageValue * scale;
                    rightBackPower = (-axial - lateral - yaw) * robot.PowerPercentageValue * scale;
                }

                // Normalize the values so no wheel power exceeds 100%
                double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                        Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

                // To see if the robot started moving
                if ((maxPower > 0) && !RobotHasBeenMovedUsingTheWheels) {
                    RobotHasBeenMovedUsingTheWheels = true;
                }

                if (maxPower > 1.0) {
                    leftFrontPower /= maxPower;
                    rightFrontPower /= maxPower;
                    leftBackPower /= maxPower;
                    rightBackPower /= maxPower;
                }

                if (!robot.RobotHasBeenMovedUsingTheWheels) {
                    if (maxPower > 0.05) {
                        robot.RobotHasBeenMovedUsingTheWheels = true;
                    }
                }
                // Send calculated power to wheels
                if (robot.DriveTrainSwitch) {
                    robot.leftFrontDrive.setPower(leftFrontPower);
                    robot.rightFrontDrive.setPower(rightFrontPower);
                    robot.leftBackDrive.setPower(leftBackPower);
                    robot.rightBackDrive.setPower(rightBackPower);
                } else {
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftBackDrive.setPower(0);
                    robot.rightBackDrive.setPower(0);
                }

                // Sleep for a short duration to avoid busy waiting
                try {
                    Thread.sleep(5); // Sleep for 5 milliseconds
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            } else {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    public void stop() {
        isActive = false;
    }
    public void ReverseControls(boolean val) {
        RobotControlsBackwards = val;
    }
    // Getter method
    public boolean hasRobotMoved() {
        return RobotHasBeenMovedUsingTheWheels;
    }
}
