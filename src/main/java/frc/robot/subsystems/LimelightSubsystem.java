package frc.robot.subsystems;

import java.util.Optional;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.limelight.LimelightResults;
import frc.robot.subsystems.limelight.LimelightResultsWrapper;
import frc.robot.subsystems.limelight.LimelightRetroTarget;


/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {

  private LimelightResults latestLimelightResults = null;

  private final NetworkTable limelightNetworkTable;
  private final String networkTableName;

  private boolean takeSnapshot = false;

  private boolean enabled;
  private boolean driverMode;
  private double activePipelineId;
  private ObjectMapper mapper;

  public LimelightSubsystem(String networkTableName) {
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
    this.networkTableName = networkTableName;

    limelightNetworkTable.getEntry("snapshot").setDouble(0.0);

    new Trigger(RobotState::isEnabled)
        .onTrue(Commands.runOnce(this::enable))
        .onFalse(Commands.runOnce(this::disable, this).ignoringDisable(true));
  }

  /**
   * Parses Limelight's JSON results dump into a LimelightResults Object
   */
  public LimelightResults getLatestResults() {
    if (latestLimelightResults == null) {
      if (mapper == null) {
        mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
      }

      try {
        var json = limelightNetworkTable.getEntry("json").getString("");
        var wrapper = mapper.readValue(json, LimelightResultsWrapper.class);
        latestLimelightResults = wrapper.targetingResults;
      } catch (JsonProcessingException e) {
        System.err.println("lljson error: " + e.getMessage());
      }
    }
    return latestLimelightResults;
  }
  
  public Optional<LimelightRetroTarget> getLatestRetroTarget() {
    var results = getLatestResults();
    if (results != null && results.valid && results.RetroreflectiveTargets.length > 0) {
      return Optional.of(results.RetroreflectiveTargets[0]);
    }
    return Optional.empty();
  }

  @Override
  public void periodic() {
    latestLimelightResults = null;
    // Flush NetworkTable to send LED mode and pipeline updates immediately
    var shouldFlush = (limelightNetworkTable.getEntry("ledMode").getDouble(0.0) != (enabled ? 0.0 : 1.0) || 
        limelightNetworkTable.getEntry("pipeline").getDouble(0.0) != activePipelineId);
    
    limelightNetworkTable.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
    limelightNetworkTable.getEntry("camMode").setDouble(driverMode ? 1.0 : 0.0);
    limelightNetworkTable.getEntry("pipeline").setDouble(activePipelineId);
  
    if (shouldFlush)  {
      NetworkTableInstance.getDefault().flush();
    }

    if(takeSnapshot) {
      limelightNetworkTable.getEntry("snapshot").setDouble(1.0);
      takeSnapshot = false;
    } else {
      limelightNetworkTable.getEntry("snapshot").setDouble(0.0);
    }
  }

  /**
   * Turns the LEDS off and switches the camera mode to vision processor.
   */
  public void disable() {
    enabled = false;
    driverMode = false;
  }

  /**
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode
   * to vision processor.
   */
  public void enable() {
    enabled = true;
    driverMode = false;
  }

  /**
   * Sets the LEDs to off and switches the camera to driver mode.
   */
  public void driverMode() {
    enabled = false;
    driverMode = true;
  }

  public String getNetworkTableName() {
    return networkTableName;
  }

  public void takeSnapshot() {
    takeSnapshot = true;
  }

  public void setPipelineId(int pipelineId) {
    activePipelineId = pipelineId;
  }

}