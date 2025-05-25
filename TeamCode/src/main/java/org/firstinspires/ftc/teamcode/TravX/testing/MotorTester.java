package org.firstinspires.ftc.teamcode.TravX.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Config
@TeleOp(name="Motor Testing", group="TravXTesting")
public class MotorTester extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    FtcDashboard ftcDashboard;
    Telemetry dashboardTelemetry;

    public DcMotorEx LiftMotor = null;

    @Override
    public void runOpMode() {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        // Getting the FTC Dashboard Instance
        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        LiftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            if (gamepad1.left_trigger > 0.01) {
                LiftMotor.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.01) {
                LiftMotor.setPower(gamepad1.right_trigger);
            } else {
                LiftMotor.setPower(0);
            }
        }
    }
}
