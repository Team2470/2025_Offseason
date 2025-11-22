package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.config.ArmConfiguration;
import frc.robot.config.PortConfiguration;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.phoenix6.TalonFXFactory;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX elevatorOne;
    private final TalonFX elevatorTwo;
    Follower followControlRequest;
    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final StatusSignal<Angle> elevatorPositionInMeters;
    private final StatusSignal<Voltage> elevatorAppliedVolts;
    private final StatusSignal<Current> elevatorSupplyCurrentAmps;
    private final StatusSignal<Current> elevatorStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> elevatorVelocityMetersPerSec;
    private final StatusSignal<AngularAcceleration> elevatorAccelerationMetersPerSecSquared;
    private final StatusSignal<Temperature> elevatorOneMotorTemp;
    private final StatusSignal<Temperature> elevatorTwoMotorTemp;

    private Supplier<Rotation2d> shoulderAngleSupplier = () -> Rotation2d.kZero;

    public ElevatorIOTalonFX(PortConfiguration ports, ArmConfiguration armConfiguration) {
        elevatorOne = TalonFXFactory.createDefaultTalon(ports.elevatorOneID);
        elevatorTwo = TalonFXFactory.createDefaultTalon(ports.elevatorTwoID);
        followControlRequest = new Follower(ports.elevatorOneID.getDeviceNumber(), false);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable   = true;
        config.CurrentLimits.StatorCurrentLimitEnable   = true;
        config.CurrentLimits.SupplyCurrentLimit         = 60.0;
        config.CurrentLimits.StatorCurrentLimit         = 120.0;
        config.Slot0.kP                                 = armConfiguration.elevatorkP;
        config.Slot0.kI                                 = armConfiguration.elevatorkI;
        config.Slot0.kD                                 = armConfiguration.elevatorkD;
        config.Slot0.kS                                 = armConfiguration.elevatorkS;
        config.MotorOutput.NeutralMode                  = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicAcceleration      = ElevatorConstants.ELEVATOR_ACCELERATION_CONSTRAINT;
        config.MotionMagic.MotionMagicCruiseVelocity    = ElevatorConstants.ELEVATOR_VELOCITY_CONSTRAINT;
        config.MotorOutput.Inverted                     = InvertedValue.Clockwise_Positive;
        elevatorOne.getConfigurator().apply(config);
        elevatorTwo.getConfigurator().apply(config);
        elevatorTwo.setControl(followControlRequest);
        elevatorPositionInMeters                        = elevatorOne.getPosition();
        elevatorAppliedVolts                            = elevatorOne.getMotorVoltage();
        elevatorSupplyCurrentAmps                       = elevatorOne.getSupplyCurrent();
        elevatorStatorCurrentAmps                       = elevatorOne.getStatorCurrent();
        elevatorVelocityMetersPerSec                    = elevatorOne.getRotorVelocity();
        elevatorAccelerationMetersPerSecSquared         = elevatorOne.getAcceleration();
        elevatorOneMotorTemp                            = elevatorOne.getDeviceTemp();
        elevatorTwoMotorTemp                            = elevatorTwo.getDeviceTemp();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPositionInMeters                 = elevatorPositionInMeters.getValueAsDouble() * ElevatorConstants.ELEVATOR_POSITION_COEFFICIENT;
        inputs.elevatorAppliedVolts                     = elevatorAppliedVolts.getValueAsDouble();
        inputs.elevatorSupplyCurrentAmps                = elevatorSupplyCurrentAmps.getValueAsDouble();
        inputs.elevatorStatorCurrentAmps                = elevatorStatorCurrentAmps.getValueAsDouble();
        inputs.elevatorVelocityMetersPerSec             = elevatorVelocityMetersPerSec.getValueAsDouble() * ElevatorConstants.ELEVATOR_POSITION_COEFFICIENT;
        inputs.elevatorAccelerationMetersPerSecSquared  = elevatorAccelerationMetersPerSecSquared.getValueAsDouble() * ElevatorConstants.ELEVATOR_POSITION_COEFFICIENT;
        inputs.elevatorOneMotorTemp                     = elevatorOneMotorTemp.getValueAsDouble();
        inputs.elevatorTwoMotorTemp                     = elevatorTwoMotorTemp.getValueAsDouble();
    }

    @Override
    public void setTargetElevator(double positionInMeters) {
        elevatorOne.setControl(positionVoltage.withPosition(positionInMeters / ElevatorConstants.ELEVATOR_POSITION_COEFFICIENT));
    }

    @Override
    public void resetElevatorPosition(double positionInMeters) {
        elevatorOne.setPosition(positionInMeters / ElevatorConstants.ELEVATOR_POSITION_COEFFICIENT);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        elevatorOne.setNeutralMode(neutralMode);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        elevatorOne.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                elevatorPositionInMeters,
                elevatorAppliedVolts,
                elevatorSupplyCurrentAmps,
                elevatorStatorCurrentAmps,
                elevatorVelocityMetersPerSec,
                elevatorAccelerationMetersPerSecSquared,
                elevatorOneMotorTemp);
    }
}
