package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.Constants;
import frc.robot.subsystems.BaseSubsystem;

public class EndEffectorSubsystem extends BaseSubsystem {
    private final EndEffectorIO endEffectorIO;
    private final EndEffectorIOAutoLogged endEffectorInputs = new EndEffectorIOAutoLogged();
    private double wantedCoralVoltage = 0.0;
    private double wantedAlgaeVoltage = 0.0;

    public enum WantedState {
        OFF,
        INTAKE,
        OUTTAKE_CORAL_L1,
        OUTTAKE_CORAL,
        OUTTAKE_ALGAE,
        HOLD
    }

    private enum SystemState {
        OFF,
        INTAKING,
        OUTTAKING_CORAL_L1,
        OUTTAKING_CORAL,
        OUTTAKING_ALGAE,
        HOLDING
    }

    private WantedState wantedState = WantedState.OFF;
    private WantedState previousWantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;
    private boolean hasCoral = false;
    private boolean hasAlgae = false;

    public EndEffectorSubsystem(EndEffectorIO endEffectorIO) {
        this.endEffectorIO = endEffectorIO;
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public boolean hasAlgae() {
        return hasAlgae;
    }

    @Override
    public void periodic() {
        Logger.processInputs("Subsystems/endEffector", endEffectorInputs);

        systemState = handleStateTransitions();

        Logger.recordOutput("Subsystems/endEffector/SystemState", systemState);
        Logger.recordOutput("Subsystems/endEffector/WantedState", wantedState);

        applyStates();

        previousWantedState = wantedState;
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case OFF:
                return SystemState.OFF;
            case INTAKE:
                return SystemState.INTAKING;
            case OUTTAKE_CORAL_L1:
                return SystemState.OUTTAKING_CORAL_L1;
            case OUTTAKE_CORAL:
                return SystemState.OUTTAKING_CORAL;
            case OUTTAKE_ALGAE:
                return SystemState.OUTTAKING_ALGAE;
            case HOLD:
                return SystemState.HOLDING;
            default:
                return SystemState.OFF;
        }
    }

    public void applyStates() {
        switch (systemState) {
            case OFF:
                wantedCoralVoltage = Constants.IntakeConstants.OffVoltages.OFF_CORAL_VOLTAGE;
                wantedAlgaeVoltage = Constants.IntakeConstants.OffVoltages.OFF_ALGAE_VOLTAGE;
                break;
            case INTAKING:
                wantedCoralVoltage = Constants.IntakeConstants.CollectingVoltages.COLLECTING_CORAL_VOLTAGE; // example voltage
                wantedAlgaeVoltage = Constants.IntakeConstants.CollectingVoltages.COLLECTING_ALGAE_VOLTAGE; // example voltage
                break;
            case OUTTAKING_CORAL_L1:
                wantedCoralVoltage = Constants.IntakeConstants.EjectingVoltages.EJECTING_CORAL_VOLTAGE_L1;
                wantedAlgaeVoltage = Constants.IntakeConstants.OffVoltages.OFF_ALGAE_VOLTAGE;
                break;
            case OUTTAKING_CORAL:
                wantedCoralVoltage = Constants.IntakeConstants.EjectingVoltages.EJECTING_CORAL_VOLTAGE;
                wantedAlgaeVoltage = Constants.IntakeConstants.OffVoltages.OFF_ALGAE_VOLTAGE;
                break;
            case OUTTAKING_ALGAE:
                wantedCoralVoltage = Constants.IntakeConstants.OffVoltages.OFF_CORAL_VOLTAGE;
                wantedAlgaeVoltage = Constants.IntakeConstants.EjectingVoltages.EJECTING_ALGAE_VOLTAGE;
                break;
            case HOLDING:
                wantedCoralVoltage = Constants.IntakeConstants.HoldingVoltages.HOLDING_CORAL_VOLTAGE;
                wantedAlgaeVoltage = Constants.IntakeConstants.HoldingVoltages.HOLDING_ALGAE_VOLTAGE;
                break;
        }

        endEffectorIO.setCoralMotorVoltage(wantedCoralVoltage);
        endEffectorIO.setAlgaeMotorVoltage(wantedAlgaeVoltage);

    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    @Override
    public void stop() {
    }
}
