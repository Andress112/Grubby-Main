package org.firstinspires.ftc.teamcode.TravX.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Hang System Status", group = "TravXUtilities")
public class HangSystemStatus extends LinearOpMode {

    private boolean manualMode = false;
    private boolean ArmMotorsEngaged = false;
    private final TravXHardware Robot = new TravXHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        // Initializing the Robot
        boolean initSuccess = Robot.init(false, false);
        if (!initSuccess) {
            sleep(10000);
            return;
        }

        Robot.HangServo.setPosition(Robot.HangServoOpenPosition);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            if (gamepad1.cross || gamepad1.a) {
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.circle || gamepad1.b) {
                if (Robot.HangServo.getPosition() == Robot.HangServoOpenPosition) {
                    Robot.HangServo.setPosition(Robot.HangServoClosePosition);
                } else {
                    Robot.HangServo.setPosition(Robot.HangServoOpenPosition);
                }
            }
            if (gamepad1.dpad_up) {
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                Robot.LiftMotor.setTargetPosition(0);
                Robot.LiftMotor.setTargetPositionTolerance(5);
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                Robot.LiftMotor.setPower(1);
            }
            if (gamepad1.dpad_down) {
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                Robot.LiftMotor.setTargetPosition(Robot.HangMotorDownPosition);
                Robot.LiftMotor.setTargetPositionTolerance(5);
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                Robot.LiftMotor.setPower(1);
            }
            if (gamepad1.triangle) {
                if (ArmMotorsEngaged) {
                    ArmMotorsEngaged = false;
                    Robot.armExtendMotorLeft.setPower(0);
                    Robot.armExtendMotorRight.setPower(0);
                } else {
                    ArmMotorsEngaged = true;
                    Robot.armExtendMotorLeft.setPower(1);
                    Robot.armExtendMotorRight.setPower(1);
                }
            }


            // Manual Mode
            if (gamepad1.left_trigger > 0.05) {
                manualMode = true;
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                Robot.LiftMotor.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.05) {
                manualMode = true;
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                Robot.LiftMotor.setPower(gamepad1.right_trigger);
            } else if (manualMode) {
                Robot.LiftMotor.setPower(0);
                manualMode = false;
                Robot.LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addLine("Press Dpad Up to go Up");
            telemetry.addLine("Press Dpad Down to go down");
            telemetry.addLine("Press O/B to toggle hang Servo");
            telemetry.addLine("Use the triggers to move the motor manually!");
            telemetry.addLine("");
            telemetry.addLine("Press X/A to reset Encoder");
            telemetry.addLine("Reset it at the top!");
            telemetry.addLine("");
            telemetry.addData("Hang Motor Position", Robot.LiftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}