package org.firstinspires.ftc.teamcode.TravX.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.TravX.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;


@TeleOp(name = "Main arm testing", group = "TravXTesting")
public class MainArmTester extends LinearOpMode {

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

            if (gamepad1.left_bumper) {
                collectorSubsystem.RetractMainArm();
            }
            if (gamepad1.right_bumper) {
                collectorSubsystem.ExtendMainArm();
            }
            if (gamepad1.dpad_down) {
                collectorSubsystem.RetractMainArmAllTheWay();
            }
            if (gamepad1.dpad_up) {
                collectorSubsystem.ExtendMainArmAllTheWay();
            }

            telemetry.addData("Main arm Power Left", Robot.armExtendMotorLeft.getPower());
            telemetry.addData("Main arm Power Right", Robot.armExtendMotorRight.getPower());
            telemetry.addLine("");
            telemetry.addData("Main arm Encoder Position Left", Robot.armExtendMotorLeft.getCurrentPosition());
            telemetry.addData("Main arm Encoder Position Right", Robot.armExtendMotorRight.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addData("Main arm offset", Robot.armExtendMotorOffset);
            telemetry.addLine("");
            telemetry.addData("Main arm known position Left", Robot.armExtendMotorLeft.getCurrentPosition() - Robot.armExtendMotorOffset);
            telemetry.addData("Main arm known position Right", Robot.armExtendMotorRight.getCurrentPosition() - Robot.armExtendMotorOffset);
            telemetry.addLine("");
            telemetry.addData("Main arm desired position", collectorSubsystem.getDesiredArmPosition());
            telemetry.addLine("");
            telemetry.addData("Main arm Power Draw Left", Robot.armExtendMotorLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Main arm Power Draw Right", Robot.armExtendMotorRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("");
            telemetry.addData("Main arm Velocitiy Left", Robot.armExtendMotorLeft.getVelocity());
            telemetry.addData("Main arm Velocitiy Right", Robot.armExtendMotorRight.getVelocity());
            telemetry.addData("Main arm sensor", !Robot.ArmMagneticSensor.getState());
            telemetry.update();
        }
    }
}
