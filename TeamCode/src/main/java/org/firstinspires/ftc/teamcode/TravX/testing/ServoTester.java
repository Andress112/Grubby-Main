package org.firstinspires.ftc.teamcode.TravX.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TravX.utilities.TravXHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

//@Disabled
@Config
@TeleOp(name="Servo Testing", group="TravXTesting")
public class ServoTester extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    FtcDashboard ftcDashboard;
    Telemetry dashboardTelemetry;

    // Getting the hardware initialized
    private final TravXHardware Robot = new TravXHardware(this);

    // Map of single servos
    private final Map<String, Servo> singleServos = new HashMap<>();

    // Map of paired servos (acting as one)
    private final Map<String, List<Servo>> pairedServos = new HashMap<>();

    // List of servo names (including paired ones)
    private static List<String> servoNames = new ArrayList<>();

    // Index for servo selection
    public static int selectedServoIndex = 0;

    // Position to set the selected servo
    public static double servoPosition = 0.0;

    public static boolean Active = false;

    @Override
    public void runOpMode() {
        // Updating the Telemetry
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        // Getting the FTC Dashboard Instance
        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        // Initializing the Robot
        boolean initSuccess = Robot.init(false, false);
        if (!initSuccess) {
            sleep(10000);
            return;
        }

        // Add single servos
        singleServos.put("collectorClawServo", Robot.collectorClawServo);
        singleServos.put("collectorRotationServo", Robot.collectorRotationServo);
        singleServos.put("clawFlipServo", Robot.clawFlipServo);
        singleServos.put("clawJawsServo", Robot.clawJawsServo);
        singleServos.put("HangServo", Robot.HangServo);

        // Add paired servos (acting as one)
        pairedServos.put("collectorPitchServos", Arrays.asList(Robot.collectorPitchServoLeft, Robot.collectorPitchServoRight));
        pairedServos.put("armFlipServos", Arrays.asList(Robot.armFlipServoLeft, Robot.armFlipServoRight));

        // Populate servo names list
        servoNames.addAll(singleServos.keySet());    // Add single servo names
        servoNames.addAll(pairedServos.keySet());    // Add paired servo names

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            if (Active) {
                if (selectedServoIndex >= 0 && selectedServoIndex < servoNames.size()) {
                    String selectedServoName = servoNames.get(selectedServoIndex);

                    // If it's a single servo
                    if (singleServos.containsKey(selectedServoName)) {
                        Servo selectedServo = singleServos.get(selectedServoName);
                        if (selectedServo != null) {
                            selectedServo.setPosition(servoPosition);
                        }
                    }

                    // If it's a paired servo
                    else if (pairedServos.containsKey(selectedServoName)) {
                        List<Servo> servos = pairedServos.get(selectedServoName);
                        if (servos != null) {
                            for (Servo servo : servos) {
                                servo.setPosition(servoPosition);
                            }
                        }
                    }
                }
            }
            telemetry.addData("Selected Servo", selectedServoIndex < servoNames.size() ? servoNames.get(selectedServoIndex) : "None");
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addLine();
            telemetry.addLine("0 - clawJawsServo");
            telemetry.addLine("1 - collectorClawServo");
            telemetry.addLine("2 - clawFlipServo");
            telemetry.addLine("3 - collectorRotationServo");
            telemetry.addLine("4 - HangServo");
            telemetry.addLine("5 - armFlipServos");
            telemetry.addLine("6 - collectorPitchServo");
            telemetry.update();

            dashboardTelemetry.addData("Selected Servo", selectedServoIndex < servoNames.size() ? servoNames.get(selectedServoIndex) : "None");
            dashboardTelemetry.addData("Servo Position", servoPosition);
            dashboardTelemetry.addLine();
            dashboardTelemetry.addLine("0 - clawJawsServo");
            dashboardTelemetry.addLine("1 - collectorClawServo");
            dashboardTelemetry.addLine("2 - clawFlipServo");
            dashboardTelemetry.addLine("3 - collectorRotationServo");
            dashboardTelemetry.addLine("4 - HangServo");
            dashboardTelemetry.addLine("4 - armFlipServos");
            dashboardTelemetry.addLine("5 - collectorPitchServo");
            dashboardTelemetry.update();
        }
    }
}