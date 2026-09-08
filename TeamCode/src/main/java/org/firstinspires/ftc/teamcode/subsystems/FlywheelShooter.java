package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.getMotorVelocityRPM;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculator;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorDistance;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotCalculatorManualCloseFar;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ShotSolution;

import java.util.List;
import java.util.Locale;

// not fleshed out, position stuff is being handled rn in flywheel but it will change
// TODO - refractor this class and all ShotCalculator classes to now use Ivy
@Configurable
public class FlywheelShooter implements Subsystem {
    public enum State {
        STOPPED,
        SPINNING
    }

    public enum SpinPosition {
        CLOSE,
        FAR
    }

    public enum ShotCalculatorMode {
        MANUAL_CLOSE_FAR,
        DISTANCE
    }

    public enum LedColor {
        OFF(0.0),
        GREEN(0.5),
        RED(0.29),
        YELLOW(0.388),
        BLUE(0.65),
        WHITE(1.0);

        private final double value;

        LedColor(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    private static final PIDFCoefficients COEFFICIENTS = new PIDFCoefficients(
            0.0005,
            0.0,
            0.0,
            1.0
    );

    private final LinearOpMode opMode;

    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public ServoImplEx light;

    private State state = State.STOPPED;
    private boolean readyToShoot = false;
    private boolean readyCandidate = false;

    // state trackers
    private SpinPosition spinPosition = SpinPosition.CLOSE;
    private final ShotCalculatorMode defaultShotCalculatorMode;
    private ShotCalculatorMode shotCalculatorMode;

    // powers
    private double commandedPower = 0.0;
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private LedColor commandedLedColor = LedColor.OFF;

    // location tracking
    private Pose2D robotPose;
    private Pose2D goalPose;
    private ShotSolution currentShotSolution = new ShotSolution(0.0, 0.0, 0.0, false);

    // pid stuff
    private double settlingTolerance = 75.0;
    private double settlingDerivativeTolerance = 50.0;
    private int settlingTimeThreshold = 300; // ms

    private final ElapsedTime settlingTimer = new ElapsedTime();
    private final PIDFController controller;

    // calculators
    private final ShotCalculatorDistance distanceCalculator;
    private final ShotCalculatorManualCloseFar manualCloseFarCalculator;

    public FlywheelShooter(LinearOpMode opMode, @NonNull ShotCalculatorMode shotCalculatorMode) {
        this.opMode = opMode;
        this.defaultShotCalculatorMode = shotCalculatorMode;
        this.shotCalculatorMode = shotCalculatorMode;

        controller = new PIDFController(COEFFICIENTS);

        distanceCalculator = new ShotCalculatorDistance();
        manualCloseFarCalculator = new ShotCalculatorManualCloseFar();

        syncManualPreset();
    }

    @Override
    public void init() {
        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "flywheelleft");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "flywheelright");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        light = opMode.hardwareMap.get(ServoImplEx.class, "light");

        state = State.STOPPED;
        spinPosition = SpinPosition.CLOSE;
        shotCalculatorMode = defaultShotCalculatorMode;
        readyToShoot = false;
        readyCandidate = false;
        commandedPower = 0.0;
        currentRPM = 0.0;
        targetRPM = 0.0;
        commandedLedColor = LedColor.OFF;
        robotPose = null;
        goalPose = null;
        currentShotSolution = new ShotSolution(0.0, 0.0, 0.0, false);

        controller.reset();

        manualCloseFarCalculator.reset();
        distanceCalculator.reset();
        syncManualPreset();

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        settlingTimer.reset();
        light.setPosition(LedColor.OFF.getValue());
    }

    @Override
    public void update() {
        calculateRPM();
        updateShotSolution();
        updateReadyToShoot();

        switch (state) {
            case STOPPED:
                commandedPower = 0.0;
                commandedLedColor = LedColor.OFF;
                controller.reset();
                break;

            case SPINNING:
                if (controller.getTargetPosition() != targetRPM) {
                    controller.setTargetPosition(targetRPM);
                }

                controller.updatePosition(currentRPM);
                controller.updateFeedForwardInput(0.5);

                commandedPower = currentShotSolution.isValid()
                                 ? clamp(controller.run(), -1.0, 1.0)
                                 : 0.0;

                commandedLedColor = currentShotSolution.isValid()
                                    ? (readyToShoot ? LedColor.GREEN : LedColor.RED)
                                    : LedColor.YELLOW;
                break;
        }

        leftMotor.setPower(commandedPower);
        rightMotor.setPower(commandedPower);
        light.setPosition(commandedLedColor.getValue());
    }

    private void updateShotSolution() {
        ShotCalculator activeCalculator = getActiveShotCalculator();

        if (robotPose != null) {
            activeCalculator.updateRobotPose(robotPose);
        }
        if (goalPose != null) {
            activeCalculator.updateGoalPose(goalPose);
        }

        currentShotSolution = activeCalculator.run();
        targetRPM = currentShotSolution.isValid()
                    ? currentShotSolution.getTargetRPM()
                    : 0.0;
    }

    private ShotCalculator getActiveShotCalculator() {
        switch (shotCalculatorMode) {
            case DISTANCE:
                return distanceCalculator;
            case MANUAL_CLOSE_FAR:
            default:
                return manualCloseFarCalculator;
        }
    }

    private void syncManualPreset() {
        switch (spinPosition) {
            case CLOSE:
                manualCloseFarCalculator.setPreset(ShotCalculatorManualCloseFar.ShotPreset.CLOSE);
                break;
            case FAR:
                manualCloseFarCalculator.setPreset(ShotCalculatorManualCloseFar.ShotPreset.FAR);
                break;
        }
    }

    private void updateReadyToShoot() {
        boolean sus =
                state == State.SPINNING
                && currentShotSolution.isValid()
                && Math.abs(controller.getError()) <= settlingTolerance
                && Math.abs(controller.getErrorDerivative()) <= settlingDerivativeTolerance;

        if (sus) {
            if (!readyCandidate) {
                readyCandidate = true;
                settlingTimer.reset();
            }

            readyToShoot = settlingTimer.milliseconds() >= settlingTimeThreshold;
        } else {
            readyCandidate = false;
            readyToShoot = false;
        }
    }

    private void calculateRPM() {
        currentRPM = (getMotorVelocityRPM(leftMotor) + getMotorVelocityRPM(rightMotor)) * 0.5;
    }

    public void startSpinning() {
        state = State.SPINNING;
    }

    public void stopSpinning() {
        state = State.STOPPED;
    }

    @Override
    public void stop() {
        state = State.STOPPED;
        spinPosition = SpinPosition.CLOSE;
        shotCalculatorMode = defaultShotCalculatorMode;
        readyToShoot = false;
        readyCandidate = false;
        commandedPower = 0.0;
        currentRPM = 0.0;
        targetRPM = 0.0;
        commandedLedColor = LedColor.OFF;
        currentShotSolution = new ShotSolution(0.0, 0.0, 0.0, false);
        robotPose = null;
        goalPose = null;

        controller.reset();

        manualCloseFarCalculator.reset();
        distanceCalculator.reset();
        syncManualPreset();

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        settlingTimer.reset();
        light.setPosition(LedColor.OFF.getValue());
    }

    @Override
    public List<String> getSimpleTelemetry() {
        return List.of(
                "Flywheel State: " + state,
                "Calculator Mode: " + shotCalculatorMode,
                "Spin Position: " + spinPosition,
                "Ready To Shoot: " + readyToShoot,
                "Power: " + String.format(Locale.US, "%.2f", commandedPower)
        );
    }

    @Override
    public List<String> getDetailedTelemetry() {
        return List.of(
                "Flywheel State: " + state,
                "Calculator Mode: " + shotCalculatorMode,
                "Spin Position: " + spinPosition,
                "Commanded Power: " + String.format(Locale.US, "%.2f", commandedPower),
                "Left Motor Power: " + String.format(Locale.US, "%.2f", leftMotor.getPower()),
                "Right Motor Power: " + String.format(Locale.US, "%.2f", rightMotor.getPower()),
                "Current RPM: " + String.format(Locale.US, "%.2f", currentRPM),
                "Target RPM: " + String.format(Locale.US, "%.2f", targetRPM),
                "Shot Valid: " + currentShotSolution.isValid(),
                "Shot Distance: " + String.format(Locale.US, "%.2f", currentShotSolution.getDistance()),
                "Shot Heading: " + String.format(Locale.US, "%.4f", currentShotSolution.getHeading()),
                "PID Error: " + String.format(Locale.US, "%.2f", controller.getError()),
                "PID Error Derivative: " + String.format(Locale.US, "%.2f", controller.getErrorDerivative()),
                "Ready Candidate: " + readyCandidate,
                "Ready To Shoot: " + readyToShoot,
                "Settling Timer (ms): " + String.format(Locale.US, "%.1f", settlingTimer.milliseconds()),
                "Settling Tolerance: " + String.format(Locale.US, "%.2f", settlingTolerance),
                "Settling Derivative Tolerance: " + String.format(Locale.US, "%.2f", settlingDerivativeTolerance),
                "Settling Time Threshold (ms): " + String.format(Locale.US, "%.1f", settlingTimeThreshold),
                "LED Color: " + commandedLedColor
        );
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public SpinPosition getSpinPosition() {
        return spinPosition;
    }

    public void setSpinPosition(@NonNull SpinPosition spinPosition) {
        this.spinPosition = spinPosition;
        syncManualPreset();
        readyToShoot = false;
        readyCandidate = false;
        settlingTimer.reset();
    }

    public ShotCalculatorMode getShotCalculatorMode() {
        return shotCalculatorMode;
    }

    public ShotCalculatorMode getDefaultShotCalculatorMode() {
        return defaultShotCalculatorMode;
    }

    public void setShotCalculatorMode(@NonNull ShotCalculatorMode shotCalculatorMode) {
        this.shotCalculatorMode = shotCalculatorMode;
        readyToShoot = false;
        readyCandidate = false;
        settlingTimer.reset();
        controller.reset();
    }

    public State getState() {
        return state;
    }

    public void setSettlingTolerance(double settlingTolerance) {
        this.settlingTolerance = Math.max(0.0, settlingTolerance);
    }

    public void setSettlingDerivativeTolerance(double settlingDerivativeTolerance) {
        this.settlingDerivativeTolerance = Math.max(0.0, settlingDerivativeTolerance);
    }

    public void setSettlingTimeThreshold(int settlingTimeThreshold) {
        this.settlingTimeThreshold = Math.max(0, settlingTimeThreshold);
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public ShotSolution getCurrentShotSolution() {
        return currentShotSolution;
    }

    public void updateRobotPose(Pose2D robotPose) {
        this.robotPose = robotPose;
    }

    public void updateGoalPose(Pose2D goalPose) {
        this.goalPose = goalPose;
    }

    public ShotCalculatorDistance getDistanceCalculator() {
        return distanceCalculator;
    }

    public ShotCalculatorManualCloseFar getManualCloseFarCalculator() {
        return manualCloseFarCalculator;
    }
}