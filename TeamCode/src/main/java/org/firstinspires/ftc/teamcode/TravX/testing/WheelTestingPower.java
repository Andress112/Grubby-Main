package org.firstinspires.ftc.teamcode.TravX.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;

//@Disabled
@Config
@TeleOp(name="Wheel Testing Power", group="TravXTesting")
public class WheelTestingPower extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();


    // Getting the hardware initialized
    private final TravXHardware Robot = new TravXHardware(this);

    @Override
    public void runOpMode() {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        // Initializing the Robot
        boolean initSuccess = Robot.init(false, false);
        if (!initSuccess) {
            sleep(10000);
            return;
        }

        Robot.leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Robot.rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Robot.leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Robot.rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            if (gamepad1.right_trigger > 0.1) {
                Robot.leftBackDrive.setPower(gamepad1.right_trigger);
                Robot.rightBackDrive.setPower(gamepad1.right_trigger);
                Robot.leftFrontDrive.setPower(gamepad1.right_trigger);
                Robot.rightFrontDrive.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                Robot.leftBackDrive.setPower(-gamepad1.left_trigger);
                Robot.rightBackDrive.setPower(-gamepad1.left_trigger);
                Robot.leftFrontDrive.setPower(-gamepad1.left_trigger);
                Robot.rightFrontDrive.setPower(-gamepad1.left_trigger);
            } else {
                Robot.leftBackDrive.setPower(0);
                Robot.rightBackDrive.setPower(0);
                Robot.leftFrontDrive.setPower(0);
                Robot.rightFrontDrive.setPower(0);
            }
        }
    }
}
