package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Mult;
import edu.wpi.first.units.Velocity;

public class ArmConstants {
  /**
   * The factors by which encoder measurements are different from actual motor rotation; default
   * units.
   */
  public static final double MOTOR_GEARING = 12.0 / 64.0 * 20.0 / 70.0 * 36.0 / 56.0 * 16.0 / 54.0;

  public static final double THROUGHBORE_GEARING = 16.0 / 54.0;

  /** Unit conversions objects for the encoders. Uses the Java units library. */
  public static final Measure<Angle> POSITION_FACTOR = Rotations.of(THROUGHBORE_GEARING);

  public static final Measure<Velocity<Angle>> VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  /** Offset from the center of the robot to the pivot's axis of rotation */
  public static final Translation3d AXLE_FROM_CHASSIS =
      new Translation3d(Inches.of(-10.465), Inches.zero(), Inches.of(25));

  /** The arm's moment of inertia; resistance to rotational movement. */
  public static final Measure<Mult<Mult<Distance, Distance>, Mass>> MOI =
      (Meters).mult(Meters).mult(Kilograms).of(0.17845);

  public static final Measure<Angle> POSITION_TOLERANCE = Degrees.of(0.8);

  public static final Measure<Mass> MASS = Kilograms.of(1);
  public static final Measure<Distance> LENGTH = Inches.of(16);

  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(4);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL =
      RadiansPerSecond.per(Second).of(6);

  public static final Measure<Angle> STARTING_ANGLE = Degrees.of(20);

  public static final Measure<Angle> MIN_ANGLE = Degrees.of(-45);
  public static final Measure<Angle> MAX_ANGLE = Degrees.of(225);

  // desperately needs tuning (we love steady-state error)
  public static final double kP = 20;
  public static final double kI = 0.0;
  public static final double kD = 0.02;

  public static final double kS = 0.14296;
  public static final double kV = 1.7305;
  public static final double kA = 0.01;
  public static final double kG = 0.12055;
}
