package org.firstinspires.ftc.teamcode.TravX.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;


@TeleOp(name="Test Sensors", group="TravXTesting")
public class SenzorValues extends LinearOpMode {

    TravXHardware Robot = new TravXHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        boolean InitSucces = Robot.init(false, false);
        if (!InitSucces) {
            return;
        }

        waitForStart();

        while (opModeIsActive()) {

            // Get the proximity value (Robot.collectorDistanceSensor in cm)

            telemetry.addLine("Motor Encoder Values:");
            telemetry.addData("Fron Right Motor", Robot.rightFrontDrive.getCurrentPosition());
            telemetry.addData("Fron Left Motor", Robot.leftFrontDrive.getCurrentPosition());
            telemetry.addData("Back Right Motor", Robot.rightBackDrive.getCurrentPosition());
            telemetry.addData("Back Left Motor", Robot.leftBackDrive.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addData("Colector Motor", Robot.collectorMotor.getCurrentPosition());
            telemetry.addData("Arm Extend Motor Right", Robot.armExtendMotorRight.getCurrentPosition());
            telemetry.addData("Arm Extend Motor Left", Robot.armExtendMotorLeft.getCurrentPosition());
            telemetry.addData("Hang Motor", Robot.LiftMotor.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addLine("Servos:");
            telemetry.addData("Collector Pitch Right servo", Robot.collectorPitchServoRight.getPosition());
            telemetry.addData("Collector Pitch Left servo", Robot.collectorPitchServoLeft.getPosition());
            telemetry.addData("Collector Rotation servo", Robot.collectorRotationServo.getPosition());
            telemetry.addData("Arm Flip servo", Robot.armFlipServoRight.getPosition());
            telemetry.addData("Claw Flip servo", Robot.clawFlipServo.getPosition());
            telemetry.addLine("");
            telemetry.addLine("Sensor's Values:");
            telemetry.addData("Collector Magnetic Button Pressed", !Robot.CollectorMagneticSensor.getState());
            telemetry.addData("Arm Magnetic Button Pressed", !Robot.ArmMagneticSensor.getState());

            telemetry.update();

        }
    }
}
