package frc.robot.subsystems.hpintake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.config.PortConfiguration;

public class HPIntakeIOPhoenix6 implements HPIntakeIO {
    private final TalonFX motor;
    private final CANdi candi;
    private final Servo servo;


    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange table;

    private final VoltageOut motorRequest = new VoltageOut(0.0);

 //   private HPIntakeIOPhoenix6(PortConfiguration ports) {
        // motor = new TalonFX(ports.intakeMotorID, new TalonFXConfiguration());
        // candi = new CANdi(ports.canDiID);
        // servo = new Servo(ports.hpIntakeServoPort);
   // }

}
