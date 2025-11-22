package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.config.PortConfiguration;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.util.phoenix6.TalonFXFactory;

public class ClimberIOPhoenix6 implements ClimberIO {
    private final TalonFX climberRollerMotor;

    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange climbCANRange;
    private final VoltageOut voltageController = new VoltageOut(0.0);

    private final StatusSignal<Voltage> climberRollerVoltage;
    private final StatusSignal<Current> climberRollerSupplyCurrent;
    private final StatusSignal<Current> climberRollerStatorCurrent;
    private final StatusSignal<Temperature> climberRollerTemperature;
    private final StatusSignal<AngularVelocity> climberRollerVelocity;
    private final StatusSignal<Boolean> climberRatchetEngaged;


    public ClimberIOPhoenix6(PortConfiguration ports) {

        climberRollerMotor = TalonFXFactory.createDefaultTalon(ports.climberID);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        configuration.CurrentLimits.SupplyCurrentLimit = 30.0;
        configuration.CurrentLimits.StatorCurrentLimit = 50.0;

        configuration.Slot0.kP = 5.0;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0;

        climberRollerMotor.getConfigurator().apply(configuration);

        configuration.MotionMagic.MotionMagicAcceleration = ClimberConstants.CLIMBER_ACCELERATION_CONSTRAINT;
        configuration.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.CLIMBER_VELOCITY_CONSTRAINT;

        climberRollerVoltage = climberRollerMotor.getMotorVoltage();
        climberRollerSupplyCurrent = climberRollerMotor.getSupplyCurrent();
        climberRollerStatorCurrent = climberRollerMotor.getStatorCurrent();
        climberRollerTemperature = climberRollerMotor.getDeviceTemp();
        climberRollerVelocity = climberRollerMotor.getVelocity();

        var canRangeConfigs = new CANrangeConfiguration();
        canRangeConfigs.ProximityParams.ProximityHysteresis = 0.01;
        canRangeConfigs.ProximityParams.ProximityThreshold = 0.07;
        canRangeConfigs.ProximityParams.MinSignalStrengthForValidMeasurement = 8000;

        climbCANRange = new CANrange(ports.climbCANRangeID.getDeviceNumber(), ports.climbCANRangeID.getBus());
        climberRatchetEngaged = climbCANRange.getIsDetected();
    }

    @Override
    public void setClimberIntakeVoltage(double voltage) {
        climberRollerMotor.setControl(voltageController.withOutput(voltage));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Update roller motor inputs
        inputs.climberRollerVoltage = climberRollerVoltage.getValueAsDouble();
        inputs.climberRollerSupplyCurrent = climberRollerSupplyCurrent.getValueAsDouble();
        inputs.climberRollerStatorCurrent = climberRollerStatorCurrent.getValueAsDouble();
        inputs.climberRollerTemperature = climberRollerTemperature.getValueAsDouble();
        inputs.climberVelocityRPS = climberRollerVelocity.getValueAsDouble();
        inputs.climberRatchetEngaged = climberRatchetEngaged.getValue(); // Placeholder for ratchet status
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                climberRollerVoltage,
                climberRollerSupplyCurrent,
                climberRollerStatorCurrent,
                climberRollerTemperature,
                climberRollerVelocity,
                climberRatchetEngaged);
    }
}
