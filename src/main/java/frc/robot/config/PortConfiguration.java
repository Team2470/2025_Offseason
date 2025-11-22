package frc.robot.config;

import frc.robot.util.CanDeviceId;

public class PortConfiguration {
    public String CANBus;

    public CanDeviceId elevatorOneID;
    public CanDeviceId elevatorTwoID;

    public CanDeviceId shoulderID;

    public CanDeviceId wristID;

    public CanDeviceId climberID;
    public CanDeviceId climbCANRangeID;

    public CanDeviceId intakeMotorID;
    public CanDeviceId intakeCoralMotorID;
    public CanDeviceId intakeAlgaeMotorID;

    public CanDeviceId candleID;

    public int neutralModeSwitchID;
    public int homeButtonID;

    public int canandcolorID;

    public PortConfiguration withCandleID(CanDeviceId candleID) {
        this.candleID = candleID;
        return this;
    }

    public PortConfiguration withCANBus(String CANBus) {
        this.CANBus = CANBus;
        return this;
    }

    public PortConfiguration withIntakeMotorID(CanDeviceId intakeMotorID) {
        this.intakeMotorID = intakeMotorID;
        return this;
    }

    public PortConfiguration withIntakeCoralMotorID(CanDeviceId intakeCoralMotorID) {
        this.intakeCoralMotorID = intakeCoralMotorID;
        return this;
    }

    public PortConfiguration withIntakeAlgaeMotorID(CanDeviceId intakeAlgaeMotorID) {
        this.intakeAlgaeMotorID = intakeAlgaeMotorID;
        return this;
    }

    public PortConfiguration withClimbCANRangeID(CanDeviceId climbCANRangeID) {
        this.climbCANRangeID = climbCANRangeID;
        return this;
    }

    public PortConfiguration withClimberID(CanDeviceId climberID) {
        this.climberID = climberID;
        return this;
    }

    public PortConfiguration withElevatorOneID(CanDeviceId elevatorOneID) {
        this.elevatorOneID = elevatorOneID;
        return this;
    }

    public PortConfiguration withElevatorTwoID(CanDeviceId elevatorTwoID) {
        this.elevatorTwoID = elevatorTwoID;
        return this;
    }

    public PortConfiguration withShoulderID(CanDeviceId shoulderID) {
        this.shoulderID = shoulderID;
        return this;
    }

    public PortConfiguration withWristID(CanDeviceId wristID) {
        this.wristID = wristID;
        return this;
    }

    public PortConfiguration withNeutralModeSwitchID(int neutralModeSwitchId) {
        this.neutralModeSwitchID = neutralModeSwitchId;
        return this;
    }

    public PortConfiguration withHomeButtonID(int homeButtonID) {
        this.homeButtonID = homeButtonID;
        return this;
    }

    public PortConfiguration withCanandcolorID(int canandcolorID) {
        this.canandcolorID = canandcolorID;
        return this;
    }

}
