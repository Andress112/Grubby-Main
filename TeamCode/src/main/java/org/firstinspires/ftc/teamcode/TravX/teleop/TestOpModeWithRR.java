package org.firstinspires.ftc.teamcode.TravX.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TravX.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;

@Disabled
@TeleOp(name = "TravX Test Auto RR", group = "TravXMain")
public class TestOpModeWithRR extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    FtcDashboard ftcDashboard;
    Telemetry dashboardTelemetry;
    Thread trajectoryThread = null;

    private final TravXHardware Robot = new TravXHardware(this);
    private long LastAction = 0;

    // Initializing the subsystems
    private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(Robot);

    /** @noinspection BusyWait*/
    @Override
    public void runOpMode() throws InterruptedException {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        // Getting the FTC Dashboard Instance
        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        // Defining the starting pose
        Pose2d startingPose = new Pose2d(-35, -64, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        // Waiting for the match to start
        waitForStart();
        runtime.reset();

        // Initializing the Robot
        boolean initSuccess = Robot.init(true, true);
        if (!initSuccess) {
            sleep(10000);
            return;
        }

        // Start the PIDF control loop
        collectorSubsystem.setOpModeActive(true);
        collectorSubsystem.startControlLoop();

        Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            drive.updatePoseEstimate();

            // Manual gamepad driving (only if no trajectory is running)
            if (trajectoryThread == null || !trajectoryThread.isAlive()) {
                double forward = -gamepad1.left_stick_y;
                double strafe = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), turn));
            }

            // Start a trajectory when pressing right stick button
            if (gamepad1.cross && (trajectoryThread == null || !trajectoryThread.isAlive())) {
                if ((System.currentTimeMillis() - LastAction) > Robot.DebounceTime) {
                    LastAction = System.currentTimeMillis();

                    TrajectoryActionBuilder goToBasket = drive.actionBuilder(drive.pose)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-47.7, -54, Math.toRadians(45.67)), Math.toRadians(220));
                    Action goToBasketAction = new SequentialAction(goToBasket.build());

                    // Run trajectory in a new thread
                    trajectoryThread = new Thread(() -> Actions.runBlocking(goToBasketAction));
                    trajectoryThread.start();
                }
            }

            // Stop trajectory if button "B" is pressed
            if (gamepad1.circle && trajectoryThread != null && trajectoryThread.isAlive()) {
                trajectoryThread.interrupt(); // Stop trajectory thread
            }

            if (gamepad1.square) {
                drive.pose = new Pose2d(69, -69, Math.toRadians(0));
            }
        }

        collectorSubsystem.setOpModeActive(false);
        collectorSubsystem.stopControlLoop();
    }

}
