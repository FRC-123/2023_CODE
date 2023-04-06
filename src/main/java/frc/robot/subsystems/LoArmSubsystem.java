package frc.robot.subsystems;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoArmConstants;

/**
 * Subsystem for wrist mechanism
 */
public class LoArmSubsystem extends SubsystemBase {

  private static final int SMART_MOTION_SLOT = 0;
  private static final double GRAVITY_FF = 0.01;
  private static final double GEARBOX_RATIO = 16.0;
  private static final double CHAIN_RAIO = 2.0;

  // limits in degrees rotation approximately 122 degreess active
  private static final float LIMIT_BOTTOM = -5.0f;
  private static final float LIMIT_TOP = 95.0f;

  private final static double OBJ_INTAKE_SPEED = -0.8;
  private final static double OBJ_EXPELL_SPEED = 1;

  private final CANSparkMax LoarmMotor;
  private final SparkMaxPIDController LopidController;
  private final RelativeEncoder LoarmEncoder;

  private final CANSparkMax LoRollerMotor;

  private Double LotargetPosition = 0.0;

  //private final static DigitalInput LoProx = new DigitalInput(1);

  private final HiArmSubsystem m_hiarm;

  public LoArmSubsystem(HiArmSubsystem hi) {
    m_hiarm = hi;

    LoRollerMotor = new CANSparkMax(LoArmConstants.LOROLLER_MOTOR_CANID, MotorType.kBrushed);

    LoRollerMotor.restoreFactoryDefaults();
    // Voltage compensation and current limits
    LoRollerMotor.enableVoltageCompensation(12);
    LoRollerMotor.setSmartCurrentLimit(35);
    LoRollerMotor.setIdleMode(IdleMode.kBrake);
    LoRollerMotor.burnFlash();

    LoarmMotor = new CANSparkMax(LoArmConstants.LOARM_MOTOR_CANID, MotorType.kBrushless);
    LoarmMotor.restoreFactoryDefaults();
    
    // Get the motor relative encoder
    LoarmEncoder = LoarmMotor.getEncoder();
    LoarmEncoder.setPositionConversionFactor(360/CHAIN_RAIO/GEARBOX_RATIO);
    LoarmEncoder.setVelocityConversionFactor(1);
    LoarmEncoder.setPosition(0);
    LopidController = LoarmMotor.getPIDController();
    LopidController.setFeedbackDevice(LoarmEncoder);

    // Configure closed-loop control - FIX ME!!!
    double kP = 0.005;
    double kI = 0.0005;
    double kD = 0.1;  // ???
    double kIz = 0.5; 
    double kFF = 0;
    double kMaxOutput = .15;
    double kMinOutput = -.5;
    double allowedErr = 0.125; // Error in rotations, not radians

    // Smart Motion Coefficients
    double maxVel = 1500; // rpm
    double maxAcc = 1000;
    double minVel = 0;

    LopidController.setP(kP);
    LopidController.setI(kI);
    LopidController.setD(kD);
    LopidController.setIZone(kIz);
    LopidController.setFF(kFF);
    LopidController.setOutputRange(kMinOutput, kMaxOutput); 

    // LopidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    // LopidController.setSmartMotionMinOutputVelocity(minVel, SMART_MOTION_SLOT);
    // LopidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
    // LopidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);

    // Voltage compensation and current limits
    LoarmMotor.enableVoltageCompensation(12);
    LoarmMotor.setSmartCurrentLimit(20);
 
    // Configure soft limits
    LoarmMotor.setSoftLimit(kForward, LIMIT_TOP);
    LoarmMotor.setSoftLimit(kReverse, LIMIT_BOTTOM);
    LoarmMotor.enableSoftLimit(kForward, true);
    LoarmMotor.enableSoftLimit(kReverse, true);

    // Disable limit switches, we don't have any
    LoarmMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    LoarmMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    
    // Brake mode helps hold arm in place - but not for closed loop
    LoarmMotor.setIdleMode(IdleMode.kCoast);
 
    // Save settings to motor flash, so they persist between power cycles
    LoarmMotor.burnFlash();
    
  }

  @Override
  public void periodic() {
    if (LotargetPosition != null) {
      // Calculate feed forward based on angle to counteract gravity
      //   double cosineScalar = Math.cos(getArmPosition());
      //   double feedForward = GRAVITY_FF * cosineScalar;
        SmartDashboard.putNumber("LoArm Setpoint",LotargetPosition);
      LopidController.setReference(LotargetPosition, 
          ControlType.kPosition ); // feedForward, ArbFFUnits.kPercentOut);
    }

    // SmartDashboard.putNumber("Arm Setpoint",targetPosition);
    SmartDashboard.putNumber("LoArm Position Raw", LoarmEncoder.getPosition());
    SmartDashboard.putNumber("LoArm Output", LoarmMotor.getAppliedOutput());
  }

/**
   * Moves the rollers using duty cycle. There is no motor safety, so calling this will continue to move until another
   * method is called.
   * @param speed duty cycle [-1,1]
   */
  public void moveRollers(double speed){
    LoRollerMotor.set(speed);
  }

  /**
   * Stop the rollers
   */
  public void stopRollers() {
    LoRollerMotor.stopMotor();
  }

  /**
   * intake cube
   * 
   */
  public void intakeObj() {
    LoRollerMotor.set(OBJ_INTAKE_SPEED);
  }

    /**
   * expell cube
   * 
   */
  public void expellObj() {
    LoRollerMotor.set(OBJ_EXPELL_SPEED);
  }

  /**
   * Moves the wrist using duty cycle. There is no motor safety, so calling this will continue to move until another
   * method is called.
   * @param speed duty cycle [-1,1]
   */
  public void moveArm(double speed){
    LotargetPosition =  null;
    LoarmMotor.set(speed);
  }

  /**
   * Moves the elevator to a position. Zero is horizontal, up is positive. There is no motor safety, so calling this
   * will continue to move to this position, and hold it until another method is called.
   * @param radians position in radians
   */
  public void moveToPosition(double degrees) {
    LotargetPosition = degrees;
    // Set the target position, but move in execute() so feed forward keeps updating
  }

  /**
   * Gets the arm position
   * @return position in degrees
   */
  public double getArmPosition() {
    return (LoarmEncoder.getPosition());
  }

  /**
   * Stop the arm
   */
  public void stop() {
    // targetPosition = null;
    LoarmMotor.stopMotor();
    LotargetPosition = LoarmEncoder.getPosition();
  }

  public boolean atPoint() {
    return Math.abs(LotargetPosition - getArmPosition()) < 5;
  }
}