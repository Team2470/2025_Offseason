package frc.robot.subsystems.arm.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.util.phoenix6.TalonFXFactory;

public class ShoulderIOTalonFX implements ShoulderIO {
    public final TalonFX shoulder;

    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

    private final StatusSignal<Angle> shoulderAngle;
    private final StatusSignal<Voltage> shoulderAppliedVolts;
    private final StatusSignal<Current> shoulderSupplyCurrentAmps;
    private final StatusSignal<Current> shoulderStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> shoulderAngularVelocityRadPerSec;
    private final StatusSignal<AngularAcceleration> shoulderAngularAccelerationRadPerSecSquared;
    private final StatusSignal<Temperature> shoulderMotorTemp;

    public ShoulderIOTalonFX(PortConfiguration ports, ArmConfiguration armConfiguration) {
        shoulder = TalonFXFactory.createDefaultTalon(ports.shoulderID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimit = 90.0;

        config.Slot0.kP = armConfiguration.shoulderkP;
        config.Slot0.kI = armConfiguration.shoulderkI;
        config.Slot0.kD = armConfiguration.shoulderkD;

        config.Slot0.kS = armConfiguration.shoulderkS;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.SHOULDER_ACCELERATION_CONSTRAINT;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.SHOULDER_VELOCITY_CONSTRAINT;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shoulder.getConfigurator().apply(config);

        shoulderAngle = shoulder.getPosition();
        shoulderAppliedVolts = shoulder.getMotorVoltage();
        shoulderSupplyCurrentAmps = shoulder.getSupplyCurrent();
        shoulderStatorCurrentAmps = shoulder.getStatorCurrent();
        shoulderAngularVelocityRadPerSec = shoulder.getRotorVelocity();
        shoulderAngularAccelerationRadPerSecSquared = shoulder.getAcceleration();
        shoulderMotorTemp = shoulder.getDeviceTemp();
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        inputs.shoulderAngle =
                Rotation2d.fromRadians(shoulderAngle.getValueAsDouble() * ArmConstants.SHOULDER_POSITION_COEFFICIENT);

        inputs.shoulderAppliedVolts = shoulderAppliedVolts.getValueAsDouble();

        inputs.shoulderSupplyCurrentAmps = shoulderSupplyCurrentAmps.getValueAsDouble();
        inputs.shoulderStatorCurrentAmps = shoulderStatorCurrentAmps.getValueAsDouble();
        inputs.shoulderAngularVelocityRadPerSec =
                shoulderAngularVelocityRadPerSec.getValueAsDouble() * ArmConstants.SHOULDER_POSITION_COEFFICIENT;
        inputs.shoulderAngularAccelerationRadPerSecSquared =
                shoulderAngularAccelerationRadPerSecSquared.getValueAsDouble()
                        * ArmConstants.SHOULDER_POSITION_COEFFICIENT;

        inputs.shoulderMotorTemp = shoulderMotorTemp.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        shoulder.setControl(
                motionMagicVoltage.withPosition(target.getRadians() / ArmConstants.SHOULDER_POSITION_COEFFICIENT));
    }

    @Override
    public void resetShoulderAngle(Rotation2d angle) {
        shoulder.setPosition(angle.getRadians() / ArmConstants.SHOULDER_POSITION_COEFFICIENT);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        shoulder.setControl(dutyCycleOut.withOutput(dutyCycle));
        // Only main motor needs to be controlled, others will follow
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        shoulder.setNeutralMode(neutralMode);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                shoulderAngle,
                shoulderAppliedVolts,
                shoulderSupplyCurrentAmps,
                shoulderStatorCurrentAmps,
                shoulderAngularVelocityRadPerSec,
                shoulderAngularAccelerationRadPerSecSquared,
                shoulderMotorTemp);
    }
}

