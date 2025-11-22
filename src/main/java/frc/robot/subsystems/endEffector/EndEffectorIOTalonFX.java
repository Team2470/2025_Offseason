package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.config.ArmConfiguration;
import frc.robot.config.PortConfiguration;
import frc.robot.util.phoenix6.TalonFXFactory;

public class EndEffectorIOTalonFX implements EndEffectorIO{
    private final TalonFX endEffectorCoralMotor;
    private final TalonFX endEffectorAlgaeMotor;

    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange hasCoralCANRange;

    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange hasAlgaeCANRange;

    private final StatusSignal<Voltage> endEffectorCoralAppliedVolts;
    private final StatusSignal<Voltage> endEffectorAlgaeAppliedVolts;
    private final StatusSignal<Current>  endEffectorCoralSupplyCurrentAmps;
    private final StatusSignal<Current>  endEffectorCoralStatorCurrentAmps;
    private final StatusSignal<Current>  endEffectorAlgaeSupplyCurrentAmps;
    private final StatusSignal<Current>  endEffectorAlgaeStatorCurrentAmps;
    private final StatusSignal<Temperature> endEffectorCoralMotorTemp;
    private final StatusSignal<Temperature> endEffectorAlgaeMotorTemp;

    private final StatusSignal<Boolean> hasCoral;
    private final StatusSignal<Boolean> hasAlgae;

    public EndEffectorIOTalonFX(PortConfiguration ports, ArmConfiguration armConfiguration) {
        endEffectorCoralMotor = TalonFXFactory.createDefaultTalon(ports.endEffectorCoralMotorID);
        endEffectorAlgaeMotor = TalonFXFactory.createDefaultTalon(ports.endEffectorAlgaeMotorID);
        hasCoralCANRange    = new CANrange(ports.endEffectorHasCoralRangeID.getDeviceNumber(), ports.endEffectorHasCoralRangeID.getBus());
        hasAlgaeCANRange    = new CANrange(ports.endEffectorHasAlgaeRangeID.getDeviceNumber(), ports.endEffectorHasAlgaeRangeID.getBus());

        endEffectorCoralAppliedVolts        = endEffectorCoralMotor.getMotorVoltage();
        endEffectorAlgaeAppliedVolts        = endEffectorAlgaeMotor.getMotorVoltage();
        endEffectorCoralSupplyCurrentAmps   = endEffectorCoralMotor.getSupplyCurrent();
        endEffectorAlgaeSupplyCurrentAmps   = endEffectorAlgaeMotor.getSupplyCurrent();
        endEffectorCoralStatorCurrentAmps   = endEffectorCoralMotor.getStatorCurrent();
        endEffectorAlgaeStatorCurrentAmps   = endEffectorAlgaeMotor.getStatorCurrent();
        endEffectorCoralMotorTemp           = endEffectorCoralMotor.getDeviceTemp();
        endEffectorAlgaeMotorTemp           = endEffectorAlgaeMotor.getDeviceTemp();
        hasCoral                            = hasCoralCANRange.getIsDetected();
        hasAlgae                            = hasAlgaeCANRange.getIsDetected();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.endEffectorCoralAppliedVolts         = endEffectorCoralAppliedVolts.getValueAsDouble();
        inputs.endEffectorAlgaeAppliedVolts         = endEffectorAlgaeAppliedVolts.getValueAsDouble();
        inputs.endEffectorCoralSupplyCurrentAmps    = endEffectorCoralSupplyCurrentAmps.getValueAsDouble();
        inputs.endEffectorAlgaeSupplyCurrentAmps    = endEffectorAlgaeSupplyCurrentAmps.getValueAsDouble();
        inputs.endEffectorCoralStatorCurrentAmps    = endEffectorCoralStatorCurrentAmps.getValueAsDouble();
        inputs.endEffectorAlgaeStatorCurrentAmps    = endEffectorAlgaeStatorCurrentAmps.getValueAsDouble();
        inputs.endEffectorCoralMotorTemp            = endEffectorCoralMotorTemp.getValueAsDouble();
        inputs.endEffectorAlgaeMotorTemp            = endEffectorAlgaeMotorTemp.getValueAsDouble();
        inputs.hasCoral                             = hasCoral.getValue();
        inputs.hasAlgae                             = hasAlgae.getValue();
    }

    @Override
    public void setCoralMotorVoltage(double voltage) {
        endEffectorCoralMotor.setVoltage(voltage);
    }

    @Override
    public void setAlgaeMotorVoltage(double voltage) {
        endEffectorAlgaeMotor.setVoltage(voltage);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        endEffectorCoralMotor.setNeutralMode(neutralMode);
        endEffectorAlgaeMotor.setNeutralMode(neutralMode);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            endEffectorCoralAppliedVolts,
            endEffectorAlgaeAppliedVolts,
            endEffectorCoralSupplyCurrentAmps,
            endEffectorCoralStatorCurrentAmps,
            endEffectorAlgaeSupplyCurrentAmps,
            endEffectorAlgaeStatorCurrentAmps,
            endEffectorCoralMotorTemp,
            endEffectorAlgaeMotorTemp,
            hasCoral,
            hasAlgae);
    }
}
