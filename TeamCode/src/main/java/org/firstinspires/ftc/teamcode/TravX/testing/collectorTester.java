package org.firstinspires.ftc.teamcode.TravX.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.TravX.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;


@TeleOp(name = "Collector testing", group = "TravXTesting")
public class collectorTester extends LinearOpMode {

    private final TravXHardware Robot = new TravXHardware(this);
    private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(Robot);


    @Override
    public void runOpMode() throws InterruptedException {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();

        // Initializing the Robot
        boolean initSuccess = Robot.init(true, false);
        if (!initSuccess) {
            sleep(10000);
            return;
        }

        // Start the PIDF control loop
        collectorSubsystem.setOpModeActive(true);
        collectorSubsystem.startControlLoop();

        Robot.clawJawsServo.setPosition(Robot.ClawJawsOpenPosition);

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            if (gamepad1.cross || gamepad1.a) {
                collectorSubsystem.ToggleCollector();
            }

            if (gamepad1.left_trigger > 0.01) {
                collectorSubsystem.retractCollectorRecursively(gamepad1.left_trigger * 0.3);
            }
            if (gamepad1.right_trigger > 0.01) {
                collectorSubsystem.extendCollectorRecursively(gamepad1.right_trigger * 0.3);
            }

            telemetry.addData("Collector Power", Robot.collectorMotor.getPower());
            telemetry.addData("Collector Encoder Position", Robot.collectorMotor.getCurrentPosition());
            telemetry.addData("Collector offset", Robot.collectorMotorOffset);
            telemetry.addData("Collector known position", Robot.collectorMotor.getCurrentPosition() - Robot.collectorMotorOffset);
            telemetry.addData("Collector desired position", collectorSubsystem.getDesiredCollectorPosition());
            telemetry.addData("Collector Power Draw", Robot.collectorMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Collector Power Draw Alert", Robot.collectorMotor.getCurrentAlert(CurrentUnit.AMPS));
            telemetry.addData("Collector Velocitiy", Robot.collectorMotor.getVelocity());
            telemetry.addData("Collector sensor", !Robot.CollectorMagneticSensor.getState());
            telemetry.update();
        }
    }
}
