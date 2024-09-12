package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Assertion.TruthAssertion;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class Arm extends SubsystemBase implements Logged, AutoCloseable {
  private final ArmIO hardware;

  @Log.NT private double voltageOut = 0;
  private final ProfiledPIDController pid;
  @Log.NT private final PIDController pidBad;

  private final ArmFeedforward ff;

  @Log.NT private final Mechanism2d mech = new Mechanism2d(2, 2);

  private final MechanismRoot2d chassis =
      mech.getRoot("Chassis", 1 + AXLE_FROM_CHASSIS.getX(), AXLE_FROM_CHASSIS.getZ());
  private final MechanismLigament2d arm =
      chassis.append(
          new MechanismLigament2d("Arm", LENGTH.in(Meters), 0, 4, new Color8Bit(Color.kAliceBlue)));

  public static Arm create() {
    return Robot.isReal() ? new Arm(new RealArm()) : new Arm(new SimArm());
  }

  public Arm(ArmIO hardware) {
    this.hardware = hardware;
    pid =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));

    pidBad = new PIDController(kP, kI, kD);

    ff = new ArmFeedforward(kS, kV, kA);

    pidBad.setTolerance(POSITION_TOLERANCE.in(Radians));
    // pid.setTolerance(POSITION_TOLERANCE.in(Radians));

    // pid.setGoal(STARTING_ANGLE.in(Radians));
    setDefaultCommand(moveTo(() -> STARTING_ANGLE.in(Radians)).withName("default"));
  }

  public Command moveTo(Measure<Angle> goal) {
    return moveTo(() -> goal.in(Radians));
  }

  public Command moveTo(DoubleSupplier goal) {
    return run(() -> updateGoal(goal.getAsDouble()))
        .withTimeout(4)
        .until(pid::atGoal)
        .finallyDo(() -> hardware.setVoltage(0));
  }

  public Command manualControl(DoubleSupplier stickInput) {
    return moveTo(
        InputStream.of(stickInput)
            .scale(MAX_VELOCITY.in(RadiansPerSecond) / 4)
            .scale(Constants.PERIOD.in(Seconds))
            .add(() -> pid.getGoal().position));
  }

  public void updateGoal(double goal) {
    pid.setGoal(goal);
    double pidOutput = pid.calculate(hardware.position(), goal);
    double ffOutput = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    voltageOut = pidOutput + ffOutput;
    hardware.setVoltage(pidOutput + ffOutput);
  }

  //   public Command moveTo(Measure<Angle> setpoint) {
  //   return moveTo(() -> setpoint.in(Radians));
  // }

  // public Command moveTo(DoubleSupplier setpoint) {
  //   return run(() -> updateGoal(setpoint.getAsDouble())).withTimeout(4)
  //       .until(pidBad::atSetpoint)
  //       .finallyDo(() -> hardware.setVoltage(0));
  // }

  // public Command manualControl(DoubleSupplier stickInput) {
  //   return moveTo(
  //       InputStream.of(stickInput)
  //           .scale(MAX_VELOCITY.in(RadiansPerSecond) / 4)
  //           .scale(Constants.PERIOD.in(Seconds))
  //           .add(() -> pidBad.getSetpoint()));
  // }

  // public void updateGoal(double setpoint) {
  //   pidBad.setSetpoint(setpoint);
  //   double pidOutput = pidBad.calculate(hardware.position(), setpoint);
  //   double ffOutput = ff.calculate(pidBad.getSetpoint(), 0);
  //   voltageOut = pidOutput + ffOutput;
  //   hardware.setVoltage(pidOutput + ffOutput);
  // }

  public void setState(double angle) {
    arm.setAngle(Units.radiansToDegrees(angle));
  }

  public Test moveToTest(Measure<Angle> angle) {
    Command testCommand = moveTo(angle);
    EqualityAssertion positionCheck =
        Assertion.eAssert(
            "Arm Position Check: expected "
                + angle.in(Radians)
                + " rad, actually "
                + hardware.position()
                + " rad",
            () -> Degrees.of(45).in(Radians),
            hardware::position,
            0.5);
    TruthAssertion atGoalCheck =
        Assertion.tAssert(
            pid::atGoal,
            "Arm At Goal Check",
            "expected " + angle.in(Radians) + " rad, actually " + hardware.position() + " rad");
    return new Test(testCommand, Set.of(positionCheck, atGoalCheck));
  }

  // public Test systemsCheck() {
  //   ChassisSpeeds speeds = new ChassisSpeeds(1, 1, 0);
  //   Command testCommand =
  //       run(() -> setChassisSpeeds(speeds, ControlMode.OPEN_LOOP_VELOCITY)).withTimeout(0.5);
  //   Function<ModuleIO, TruthAssertion> speedCheck =
  //       m ->
  //           tAssert(
  //               () -> m.state().speedMetersPerSecond * Math.signum(m.position().angle.getCos()) >
  // 1,
  //               "Drive Syst Check " + m.name() + " Module Speed",
  //               "expected: >= 1; actual: " + m.state().speedMetersPerSecond);
  //   Function<ModuleIO, EqualityAssertion> atAngle =
  //       m ->
  //           eAssert(
  //               "Drive Syst Check " + m.name() + " Module Angle (degrees)",
  //               () -> 45,
  //               () -> Units.radiansToDegrees(atan(m.position().angle.getTan())),
  //               1);
  //   Set<Assertion> assertions =
  //       modules.stream()
  //           .flatMap(m -> Stream.of(speedCheck.apply(m), atAngle.apply(m)))
  //           .collect(Collectors.toSet());
  //   return new Test(testCommand, assertions);
  // }

  @Override
  public void periodic() {
    setState(hardware.position());
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
    log("at goal", pid.atGoal());
    log("at setpoint", pidBad.atSetpoint());
    log("position", (hardware.position()));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
