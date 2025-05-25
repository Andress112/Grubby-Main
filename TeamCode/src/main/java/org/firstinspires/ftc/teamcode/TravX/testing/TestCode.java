package org.firstinspires.ftc.teamcode.TravX.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TravX.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.TravX.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;

@Disabled
@Config
@TeleOp(name="Testing GroundTestCode", group="TravXTesting")
public class TestCode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    FtcDashboard ftcDashboard;
    Telemetry dashboardTelemetry;

    // Getting the hardware initialized
    private final TravXHardware Robot = new TravXHardware(this);
    private long LastAction = 0;

    // Initializing the subsystems
    private final Drivetrain drivetrain = new Drivetrain(Robot);
    private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(Robot);

    @Override
    public void runOpMode() {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        // Getting the FTC Dashboard Instance
        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        // Initializing the Robot
        boolean initSuccess = Robot.init(true, false);
        if (!initSuccess) {
            sleep(10000);
            return;
        }

        // Start the PIDF control loop
        collectorSubsystem.startControlLoop();
        collectorSubsystem.setOpModeActive(true);

        waitForStart();
        runtime.reset();

        Robot.collectorPitchServoRight.setPosition(Robot.CollectorPitchUpPosition);
        Robot.collectorPitchServoLeft.setPosition(Robot.CollectorPitchUpPosition);
        Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            if (gamepad1.dpad_left) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    LastAction = System.currentTimeMillis();
                    Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateLeftPosition);
                }
            }
            if (gamepad1.dpad_up) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    LastAction = System.currentTimeMillis();
                    Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateMidPosition);
                }
            }
            if (gamepad1.dpad_right) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    LastAction = System.currentTimeMillis();
                    Robot.collectorRotationServo.setPosition(Robot.CollectorClawRotateRightPosition);
                }
            }
        }
        // Stop the wheel control when the op mode is no longer active
        drivetrain.stopWheelControl();
        collectorSubsystem.stopControlLoop();
        collectorSubsystem.setOpModeActive(false);
    }
}
