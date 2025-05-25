package org.firstinspires.ftc.teamcode.TravX.utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/** @noinspection UnusedAssignment*/
@Config
public class TravXHardware {

    /* Declare OpMode members. */
    public LinearOpMode CurrentOpMode = null;   // gain access to methods in the calling OpMode.

    // Motors

    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightBackDrive = null;

    public DcMotorEx collectorMotor = null;
    public DcMotorEx armExtendMotorRight = null;
    public DcMotorEx armExtendMotorLeft = null;
    public DcMotorEx LiftMotor = null;

    // Servos

    public Servo collectorClawServo = null;
    public Servo collectorPitchServoLeft = null;
    public Servo collectorPitchServoRight = null;
    public Servo collectorRotationServo = null;
    public Servo armFlipServoRight = null;
    public Servo armFlipServoLeft = null;
    public Servo clawFlipServo = null;
    public Servo clawJawsServo = null;
    public Servo HangServo = null;

    //  Sensors

    public DigitalChannel ArmMagneticSensor = null;
    public DigitalChannel CollectorMagneticSensor = null;

    public LED IndicatorLedGreen = null;
    public LED IndicatorLedRed = null;
    public VoltageSensor voltageSensor = null;

    public AnalogInput ArmFlipServoPosition = null;
    // Variable and positions

    public final int DebounceTime = 300; // In ms

    public volatile boolean DriveTrainSwitch = true;
    public volatile boolean RobotHasBeenMovedUsingTheWheels = false;
    public volatile double PowerPercentageValue = 1;

    // Auto Collector Variables
    public volatile int autoPickupRelativeXOffset = 0;
    public volatile int autoPickupRelativeYOffset = 0;
    public volatile double autoPickupRelativeAngleOffset = 0;
    public volatile double prevAdjustedAngle = 0;
    public final double autoPickupMotorPower = 0.4;
    public final int autoPickupTimeBeforeReturningControlToDriver = 1000;
    public volatile boolean isAutoMovementActive = false;

    // Motor Powers
    public final double MotorCrawlPower = 0.45;
    public final double MotorFastPower = 1;

    public volatile double axialGain = 1;
    public volatile double lateralGain = 1;
    public volatile double YawnGain = 1;

    // Motor Offsets
    public volatile double armExtendMotorOffset = 0;
    public volatile double collectorMotorOffset = 0;
    public double collectorMotorPower = 1;
    public double armExtendMotorPowerDown = 0.6;
    public double armExtendMotorPowerUp = 1;
    public final double PIDFTolerance = 15;
    public final double PIDFResetTolerance = 50;

    // Arm Flip Positions
    public final double ArmFlipServoDownPosition = 0.2;
    public final double ArmFlipServoMidPosition = 0.95;
    public final double ArmFlipServoBackPosition = 0.8;
    public final double ArmFlipServoHighRungPosition = 0.85;
    public final double ArmFlipServoTransitionPosition = 0.29;

    // Claw Flip Positions
    public double ClawFlipServoDownPosition = 0.42;
    public final double ClawFlipServoMidPosition = 0.7;
    public final double ClawFlipServoBackPosition = 0.55;
    public final double ClawFlipServoHighRungPosition = 0.71;
    public final double ClawFlipServoTransitionPosition = 0.58;

    // Claw Flip Positions

    // Arm Claw Jaws
    public final double ClawJawsOpenPosition = 0.45;
    public final double ClawJawsClosePosition = .6;

    // Collector Pitch
    public final double CollectorPitchDownPosition = 0.078;
    public final double CollectorPitchMidPointPosition = 0.41;
    public final double CollectorPitchUpPosition = 0.67;
    public final double CollectorPitchTransitionPosition = 0.73;

    // Collector Claw
    public final double CollectorClawOpenPosition = 0.4;
    public final double CollectorClawClosePosition = 0.14;

    // Collector Variables
    public final int CollectorMaxExtensionTics = 350;
    public static double COLLECTOR_EXTENSION_SPEED = 0.19; // Ticks per second
    public static double COLLECTOR_RETRACTION_SPEED = 0.2;
    public final int PID_LOOP_DELAY = 2;

    // Collector Claw Rotate Servo
    public final double CollectorClawRotateLeftPosition = 0; // Ticks per second
    public final double CollectorClawRotateMidPosition = 0.35; // Ticks per second
    public final double CollectorClawRotateRightPosition = 0.66; // Ticks per second

    // Hang Servo Positions
    public final double HangServoOpenPosition = 0.4;
    public final double HangServoClosePosition = 0.51;
    // Hang Motor Positions
    public final int HangMotorDownPosition = 10000; // Hang Position (One Full rotation)

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public TravXHardware (LinearOpMode opmode) {
        CurrentOpMode = opmode;
    }

    public boolean init(boolean SetInitialPositions, boolean ForAutonomy) {
        try {
            //          Assign the hardware
            if (!ForAutonomy) {
                leftFrontDrive = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "leftFrontPower");
                leftBackDrive = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "leftBackPower");
                rightFrontDrive = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "rightFrontPower");
                rightBackDrive = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "rightBackPower");
            }

            collectorMotor = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "collectorMotor");
            armExtendMotorRight = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "armExtendMotorRight");
            armExtendMotorLeft = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "armExtendMotorLeft");
            LiftMotor = CurrentOpMode.hardwareMap.get(DcMotorEx.class, "LiftMotor");

            //          Set Motor Directions

            if (!ForAutonomy) {
                leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
            }

//            armExtendMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
            armExtendMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
            collectorMotor.setDirection(DcMotorEx.Direction.REVERSE);
            LiftMotor.setDirection(DcMotorEx.Direction.REVERSE);

            //          Motor Braking

            if (!ForAutonomy) {
                leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }

            //          Assign the servos

            // Collector
            collectorClawServo = CurrentOpMode.hardwareMap.get(Servo.class, "collectorClawServo");
            collectorPitchServoRight = CurrentOpMode.hardwareMap.get(Servo.class, "collectorPitchServoRight");
            collectorPitchServoLeft = CurrentOpMode.hardwareMap.get(Servo.class, "collectorPitchServoLeft");
            collectorRotationServo = CurrentOpMode.hardwareMap.get(Servo.class, "collectorRotationServo");
            // Arm
            armFlipServoRight = CurrentOpMode.hardwareMap.get(Servo.class, "armFlipServoRight");
            armFlipServoLeft = CurrentOpMode.hardwareMap.get(Servo.class, "armFlipServoLeft");
            clawFlipServo = CurrentOpMode.hardwareMap.get(Servo.class, "clawFlipServo");
            clawJawsServo = CurrentOpMode.hardwareMap.get(Servo.class, "clawJawsServo");
            // Hang
            HangServo = CurrentOpMode.hardwareMap.get(Servo.class, "HangServo");

            //          Set Servo Directions

            collectorPitchServoLeft.setDirection(Servo.Direction.REVERSE);

            //          Assign the sensors

            ArmMagneticSensor = CurrentOpMode.hardwareMap.get(DigitalChannel.class, "armMagneticSensor");
            CollectorMagneticSensor = CurrentOpMode.hardwareMap.get(DigitalChannel.class, "collectorMagneticSensor");

            IndicatorLedGreen = CurrentOpMode.hardwareMap.get(LED.class, "indicatorLedGreen");
            IndicatorLedRed = CurrentOpMode.hardwareMap.get(LED.class, "indicatorLedRed");
            voltageSensor = CurrentOpMode.hardwareMap.voltageSensor.iterator().next();

            ArmFlipServoPosition = CurrentOpMode.hardwareMap.get(AnalogInput.class, "armFlipServoPosition");

            IndicatorLedGreen.on();
            IndicatorLedRed.off();

            ArmMagneticSensor.setMode(DigitalChannel.Mode.INPUT);
            CollectorMagneticSensor.setMode(DigitalChannel.Mode.INPUT);

            if (SetInitialPositions) {

                // Servos

                // arm
                clawJawsServo.setPosition(ClawJawsClosePosition);
                clawFlipServo.setPosition(ClawFlipServoDownPosition);
                armFlipServoRight.setPosition(ArmFlipServoDownPosition);
                armFlipServoLeft.setPosition(ArmFlipServoDownPosition);

                // collector
                collectorRotationServo.setPosition(CollectorClawRotateMidPosition);
                collectorPitchServoRight.setPosition(CollectorPitchUpPosition);
                collectorPitchServoLeft.setPosition(CollectorPitchUpPosition);
                collectorClawServo.setPosition(CollectorClawOpenPosition);

                // Hang
                HangServo.setPosition(HangServoOpenPosition);

                LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                LiftMotor.setTargetPosition(0);
                LiftMotor.setTargetPositionTolerance(5);
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotor.setPower(1);

                // Motors Init
                armExtendMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                armExtendMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                collectorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                // Finished flags
                boolean armFinished = false;
                boolean collectorFinished = false;

                if (!ArmMagneticSensor.getState()) {
                    armFinished = true;
                    armExtendMotorRight.setPower(0);
                    armExtendMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    armExtendMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    armExtendMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    armExtendMotorLeft.setPower(0);
                    armExtendMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    armExtendMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    armExtendMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armExtendMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    armExtendMotorRight.setPower(-1);
                    armExtendMotorLeft.setPower(-1);
                }

                if (!CollectorMagneticSensor.getState()) {
                    collectorFinished = true;
                    collectorMotor.setPower(0);
                    collectorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    collectorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                } else {
                    collectorMotor.setPower(-1);
                }

                long timeout = 5000; // 1 second in milliseconds
                long startTime = System.currentTimeMillis();
                while (!collectorFinished || !armFinished && !CurrentOpMode.isStopRequested()) {
                    if (System.currentTimeMillis() - startTime >= timeout) {
                        armFinished = true;
                        collectorFinished = true;
                        break;
                    }
                    // Gradually increase motor power for arm motor if target is not reached
                    if (!armFinished && ArmMagneticSensor.getState()) {
                        armExtendMotorRight.setPower(-1);
                    }

                    // Gradually increase motor power for collector motor if target is not reached
                    if (!collectorFinished && CollectorMagneticSensor.getState()) {
                        collectorMotor.setPower(-1);
                    }

                    // Check the arm touch sensor
                    if (!armFinished) { // Corrected condition
                        if (!ArmMagneticSensor.getState()) {
                            armExtendMotorRight.setPower(0);
                            armExtendMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                            armExtendMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                            armExtendMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                            armExtendMotorLeft.setPower(0);
                            armExtendMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                            armExtendMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                            armExtendMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                            if (!ArmMagneticSensor.getState()) {
                                armExtendMotorOffset = armExtendMotorLeft.getCurrentPosition();
                            }
                            armFinished = true; // Set the flag to true
                        }
                    }

                    // Check the collector touch sensor
                    if (!collectorFinished) { // Corrected condition
                        if (!CollectorMagneticSensor.getState()) {
                            collectorMotor.setPower(0);
                            collectorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                            collectorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                            if (!CollectorMagneticSensor.getState()) {
                                collectorMotorOffset = collectorMotor.getCurrentPosition();
                            }
                            collectorFinished = true; // Set the flag to true
                        }
                    }
                    // Exit loop when both actions are complete
                    if (collectorFinished && armFinished) {
                        break;
                    }
                    if (CurrentOpMode.isStopRequested()) return false;
                }
            }

            CurrentOpMode.telemetry.addData(">", "Hardware Initialized!!!");
            CurrentOpMode.telemetry.update();
            return true;
        } catch (Exception e) {
            CurrentOpMode.telemetry.addData(">", "Hardware Initialization failed!!!");
            CurrentOpMode.telemetry.addData(">", "Please retry or verify you connections!");
            CurrentOpMode.telemetry.addData(">", e);
            CurrentOpMode.telemetry.update();
            return false;
        }
    }
}
