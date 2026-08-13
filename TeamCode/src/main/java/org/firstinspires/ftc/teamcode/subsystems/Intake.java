package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.getMotorVelocityRPM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.Locale;

@Configurable
public class Intake implements Subsystem {
    public enum State {
        STOPPED,
        HOLDING,
        INTAKING,
        OUTTAKING,
        JAM_CLEARING
    }

    private final LinearOpMode opMode;
    public DcMotorEx motor;

    // driver intent
    private State desiredState = State.STOPPED;
    // actual subsystem state
    private State state = State.STOPPED;
    // used for transition detection
    private State previousState = State.STOPPED;

    private double commandedPower = 0.0;

    // powers are inverted in code
    private double holdingPower = 0.05;
    private double intakingPower = 1.0;
    private double outtakingPower = 0.3;
    private double jamClearingPower = 0.7;

    // jamming
    private double jamCurrentThreshold = 5.0; // amps
    private double jamVelocityThreshold = 50.0; // rpm
    private double jamTimeThreshold = 250.0; // ms
    private double jamMinPower = 0.4; // in [0, 1]
    private double jamSpinUpDelay = 500.0; // ms

    // jam clearing
    private double jamClearDuration = 250.0; // ms
    private boolean autoJamClearingEnabled = true;

    private boolean jamCandidate = false;
    private boolean jammed = false;

    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime jamClearTimer = new ElapsedTime();
    private final ElapsedTime intakeRunTimer = new ElapsedTime();

    // item detection
    private DistanceSensor distanceSensor;
    private boolean hasItem = false;
    private boolean itemCandidate = false;
    private double itemDistanceThreshold = 5.0; // cm

    // item must be seen continuously for this long
    private double itemDetectionTime = 60.0; // ms
    private final ElapsedTime itemDetectionTimer = new ElapsedTime();

    // single polling timer
    private double distanceSensorFastPollInterval = 20.0;  // ms
    private double distanceSensorSlowPollInterval = 200.0; // ms
    private final ElapsedTime distanceSensorPollTimer = new ElapsedTime();

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "intakeSensor");

        desiredState = State.STOPPED;
        state = State.STOPPED;
        previousState = State.STOPPED;
        commandedPower = 0.0;

        autoJamClearingEnabled = true;

        jamCandidate = false;
        jammed = false;
        jamTimer.reset();
        jamClearTimer.reset();
        intakeRunTimer.reset();

        hasItem = false;
        itemCandidate = false;
        itemDetectionTimer.reset();

        distanceSensorPollTimer.reset();
    }

    @Override
    public void update() {
        detectJam();
        detectItem();

        // before any state changing code
        previousState = state;

        boolean clear =
                state == State.JAM_CLEARING
                && desiredState == State.INTAKING // if user switches intent then change
                && jamClearTimer.milliseconds() < jamClearDuration;

        if (!clear) {
            state = desiredState;

            if (autoJamClearingEnabled && state == State.INTAKING && jammed) {
                enterJamClearing();
            }
        }

        switch (state) {
            case STOPPED:
                commandedPower = 0.0;
                break;
            case HOLDING:
                commandedPower = -holdingPower;
                break;
            case INTAKING:
                if (previousState != State.INTAKING) {
                    intakeRunTimer.reset();
                }
                commandedPower = intakingPower;
                break;
            case OUTTAKING:
                commandedPower = -outtakingPower;
                break;
            case JAM_CLEARING:
                commandedPower = -jamClearingPower;
                break;
        }

        motor.setPower(commandedPower);
    }

    private void enterJamClearing() {
        state = State.JAM_CLEARING;
        jamClearTimer.reset();
        jamCandidate = false;
        jammed = false;
    }

    private void detectJam() {
        boolean sus =
                state == State.INTAKING
                && intakeRunTimer.milliseconds() >= jamSpinUpDelay
                && Math.abs(motor.getPower()) >= jamMinPower
                && motor.getCurrent(CurrentUnit.AMPS) >= jamCurrentThreshold
                && getMotorVelocityRPM(motor) <= jamVelocityThreshold;

        if (sus) {
            if (!jamCandidate) {
                jamCandidate = true;
                jamTimer.reset();
            }

            jammed = jamTimer.milliseconds() >= jamTimeThreshold;
        } else {
            jamCandidate = false;
            jammed = false;
        }
    }

    private void detectItem() {
        double pollInterval = (desiredState == State.INTAKING || hasItem)
                              ? distanceSensorFastPollInterval
                              : distanceSensorSlowPollInterval;

        if (distanceSensorPollTimer.milliseconds() < pollInterval) {
            return;
        }
        distanceSensorPollTimer.reset();

        boolean sus = distanceSensor.getDistance(DistanceUnit.CM) <= itemDistanceThreshold;

        if (sus) {
            if (!itemCandidate) {
                itemCandidate = true;
                itemDetectionTimer.reset();
            }

            hasItem = itemDetectionTimer.milliseconds() >= itemDetectionTime;
        } else {
            itemCandidate = false;
            hasItem = false;
        }
    }

    public void startIntake() {
        desiredState = State.INTAKING;
    }

    public void stopIntake() {
        desiredState = State.STOPPED;
    }

    public void startOuttake() {
        desiredState = State.OUTTAKING;
    }

    public void startHolding() {
        desiredState = State.HOLDING;
    }

    // temp jam clear
    public void startJamClearing() {
        enterJamClearing();
    }

    @Override
    public void stop() {
        desiredState = State.STOPPED;
        state = State.STOPPED;
        previousState = State.STOPPED;
        commandedPower = 0.0;
        motor.setPower(0.0);

        autoJamClearingEnabled = true;

        jamCandidate = false;
        jammed = false;
        jamTimer.reset();
        jamClearTimer.reset();
        intakeRunTimer.reset();

        hasItem = false;
        itemCandidate = false;
        itemDetectionTimer.reset();

        distanceSensorPollTimer.reset();
    }

    @Override
    public List<String> getSimpleTelemetry() {
        return List.of(
                "Intake State: " + state,
                "Desired State: " + desiredState,
                "Has Item: " + hasItem,
                "Jammed: " + jammed,
                "Auto Jam Clear: " + autoJamClearingEnabled,
                "Power: " + String.format(Locale.US, "%.2f", commandedPower)
        );
    }

    @Override
    public List<String> getDetailedTelemetry() {
        return List.of(
                "Intake State: " + state,
                "Desired State: " + desiredState,
                "Commanded Power: " + String.format(Locale.US, "%.2f", commandedPower),
                "Motor Power: " + String.format(Locale.US, "%.2f", motor.getPower()),
                "Motor Current (A): " + String.format(Locale.US, "%.2f", motor.getCurrent(CurrentUnit.AMPS)),
                "Motor Velocity (RPM): " + String.format(Locale.US, "%.2f", getMotorVelocityRPM(motor)),
                "Auto Jam Clearing Enabled: " + autoJamClearingEnabled,
                "Has Item: " + hasItem,
                "Item Candidate: " + itemCandidate,
                "Last Item Distance (cm): " + String.format(Locale.US, "%.2f", distanceSensor.getDistance(DistanceUnit.CM)),
                "Item Distance Threshold (cm): " + String.format(Locale.US, "%.2f", itemDistanceThreshold),
                "Item Detection Time (ms): " + String.format(Locale.US, "%.1f", itemDetectionTime),
                "Item Fast Poll Interval (ms): " + String.format(Locale.US, "%.1f", distanceSensorFastPollInterval),
                "Item Slow Poll Interval (ms): " + String.format(Locale.US, "%.1f", distanceSensorSlowPollInterval),
                "Item Detect Timer (ms): " + String.format(Locale.US, "%.1f", itemDetectionTimer.milliseconds()),
                "Item Poll Timer (ms): " + String.format(Locale.US, "%.1f", distanceSensorPollTimer.milliseconds()),
                "Jam Candidate: " + jamCandidate,
                "Jammed: " + jammed,
                "Jam Timer (ms): " + String.format(Locale.US, "%.1f", jamTimer.milliseconds()),
                "Jam Clear Timer (ms): " + String.format(Locale.US, "%.1f", jamClearTimer.milliseconds()),
                "Jam Current Threshold: " + String.format(Locale.US, "%.2f", jamCurrentThreshold),
                "Jam Velocity Threshold (RPM): " + String.format(Locale.US, "%.2f", jamVelocityThreshold),
                "Jam Time Threshold (ms): " + String.format(Locale.US, "%.1f", jamTimeThreshold),
                "Jam Spin-Up Delay (ms): " + String.format(Locale.US, "%.1f", jamSpinUpDelay),
                "Intake Run Timer (ms): " + String.format(Locale.US, "%.1f", intakeRunTimer.milliseconds())
        );
    }

    // Manual override: disable automatic jam clearing so driver can take control.
    public void setAutoJamClearingEnabled(boolean enabled) {
        this.autoJamClearingEnabled = enabled;
    }

    public boolean isAutoJamClearingEnabled() {
        return autoJamClearingEnabled;
    }

    public void setHoldingPower(double power) {
        this.holdingPower = Math.abs(clamp(power, -1.0, 1.0));
    }

    public void setIntakingPower(double power) {
        this.intakingPower = Math.abs(clamp(power, -1.0, 1.0));
    }

    public void setOuttakingPower(double power) {
        this.outtakingPower = Math.abs(clamp(power, -1.0, 1.0));
    }

    public State getState() {
        return state;
    }

    public State getDesiredState() {
        return desiredState;
    }

    public double getMotorPower() {
        return motor.getPower();
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public boolean isJammed() {
        return jammed;
    }

    public boolean hasItem() {
        return hasItem;
    }

    public double getItemDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}