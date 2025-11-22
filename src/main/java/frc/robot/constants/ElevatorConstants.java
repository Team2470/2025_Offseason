package frc.robot.constants;

import edu.wpi.first.units.Units;

public class ElevatorConstants {
    // Value in Inches
    public static final double ZEROED       =  0.00;
    public static final double CLIMB        =  0.00;
    public static final double DRIVE        =  0.33;
    public static final double HP_INTAKE    =  3.00;
    public static final double GROUND_ALGAE =  8.00;
    public static final double L2_ALGAE     = 31.50;
    public static final double L3_ALGAE     = 49.00;
    public static final double PROCESSOR    =  3.00;
    public static final double NET          = 60.00;
    public static final double L1_CORAL     =  5.00;
    public static final double L2_CORAL     = 14.00;
    public static final double L3_CORAL     = 30.00;
    public static final double L4_CORAL     = 56.00;

    // TODO: Update these constants based on mechanism characterization
    public static final double ELEVATOR_GEAR_RATIO                  = ((32.0 / 16.0) * (40.0 / 26.0) * (50.0 / 20.0) * (62.0 / 76.0));
    public static final double ELEVATOR_PULLEY_DIAMETER             = Units.Meter.convertFrom(0.25 * 16.0 / Math.PI, Units.Inch);
    public static final double ELEVATOR_CASCADE_COEFFICIENT         = 2.0;
    public static final double ELEVATOR_POSITION_COEFFICIENT        = Math.PI * ELEVATOR_PULLEY_DIAMETER / ELEVATOR_GEAR_RATIO * ELEVATOR_CASCADE_COEFFICIENT;
    public static final double ELEVATOR_ACCELERATION                = Units.Meter.convertFrom(400, Units.Inch);
    public static final double ELEVATOR_VELOCITY                    = Units.Meter.convertFrom(200, Units.Inch);
    public static final double ELEVATOR_ACCELERATION_CONSTRAINT     = 100.0; // Inches per second squared
    public static final double ELEVATOR_VELOCITY_CONSTRAINT         = 100.0; // Inches per second squared
    public static final double ELEVATOR_ZEROING_DUTY_CYCLE          = -0.07;
    public static final double ELEVATOR_SETPOINT_TOLERANCE_METERS   = Units.Meter.convertFrom(0.5, Units.Inch);
    public static final double ELEVATOR_HIGH_THRESHOLD              = Units.Meter.convertFrom(30, Units.Inch);

}
