// LimelightHelpers v1.3.0 (Feb 24, 2024)

package frc.robot;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

@SuppressWarnings("all")
public class LimelightHelpers {

    public static class LimelightTargetRetro {

        @JsonProperty("t6c_ts")
        private double[] m_cameraPoseTargetSpace;

        @JsonProperty("t6r_fs")
        private double[] m_robotPoseFieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] m_targetPoseCameraSpace;

        @JsonProperty("t6t_rs")
        private double[] m_targetPoseRobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ta")
        public double ta;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("tx")
        public double tx;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("txp")
        public double tx_pixels;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ty")
        public double ty;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("typ")
        public double ty_pixels;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ts")
        public double ts;

        public LimelightTargetRetro() {
            m_cameraPoseTargetSpace = new double[6];
            m_robotPoseFieldSpace = new double[6];
            m_robotPoseTargetSpace = new double[6];
            m_targetPoseCameraSpace = new double[6];
            m_targetPoseRobotSpace = new double[6];
        }

    }

    public static class LimelightTargetFiducial {

        @SuppressWarnings({
                "MemberNameCheck",
                "AbbreviationAsWordInNameCheck",
                "AbbreviationAsWordInNameCheck"
        })
        @JsonProperty("fID")
        public double fiducialID;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("fam")
        public String fiducialFamily;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("tx")
        public double tx;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("txp")
        public double tx_pixels;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ty")
        public double ty;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("typ")
        public double ty_pixels;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    public static class LimelightTargetBarcode {

    }

    public static class LimelightTargetClassifier {

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("class")
        public String className;

        @SuppressWarnings({
                "MemberNameCheck",
                "AbbreviationAsWordInNameCheck"
        })
        @JsonProperty("classID")
        public double classID;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("conf")
        public double confidence;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("zone")
        public double zone;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("tx")
        public double tx;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("txp")
        public double tx_pixels;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ty")
        public double ty;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTarget_Classifier() {
        }
    }

    public static class LimelightTargetDetector {

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("class")
        public String className;

        @SuppressWarnings({
                "MemberNameCheck",
                "AbbreviationAsWordInNameCheck"
        })
        @JsonProperty("classID")
        public double classID;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("conf")
        public double confidence;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ta")
        public double ta;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("tx")
        public double tx;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("txp")
        public double tx_pixels;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("ty")
        public double ty;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTargetDetector() {
        }
    }

    public static class Results {

        @SuppressWarnings({
                "MemberNameCheck", "AbbreviationAsWordInNameCheck"
        })
        @JsonProperty("pID")
        public double pipelineID;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("tl")
        public double latency_pipeline;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("cl")
        public double latency_capture;

        public double m_latencyJsonParse;

        @SuppressWarnings({
                "MemberNameCheck", "AbbreviationAsWordInNameCheck"
        })
        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @SuppressWarnings({
                "MemberNameCheck", "AbbreviationAsWordInNameCheck"
        })
        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("botpose")
        public double[] botpose;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("botpose_tagcount")
        public double botpose_tagcount;

        @JsonProperty("botpose_span")
        public double botpose_span;

        @JsonProperty("botpose_avgdist")
        public double botpose_avgdist;

        @JsonProperty("botpose_avgarea")
        public double botpose_avgarea;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }

        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }

        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }

        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }

        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("Retro")
        public LimelightTargetRetro[] targets_Retro;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("Fiducial")
        public LimelightTargetFiducial[] targets_Fiducials;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("Classifier")
        public LimelightTargetClassifier[] targets_Classifier;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("Detector")
        public LimelightTargetDetector[] targets_Detector;

        @SuppressWarnings("MemberNameCheck")
        @JsonProperty("Barcode")
        public LimelightTargetBarcode[] targets_Barcode;

        public Results() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTargetRetro[0];
            targets_Fiducials = new LimelightTargetFiducial[0];
            targets_Classifier = new LimelightTargetClassifier[0];
            targets_Detector = new LimelightTargetDetector[0];
            targets_Barcode = new LimelightTargetBarcode[0];

        }
    }

    public static class LimelightResults {
        @JsonProperty("Results")
        public Results m_targetingResults;

        public String error;

        public LimelightResults() {
            targetingResults = new Results();
            error = "";
        }

    }

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;

        public PoseEstimate(
                Pose2d pose,
                double timestampSeconds,
                double latency,
                int tagCount,
                double tagSpan,
                double avgTagDist,
                double avgTagArea) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
        }
    }

    private static ObjectMapper mapper;

    /**
     * Print JSON Parse time to the console in milliseconds.
     */
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }

    private static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            // System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            // System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    private static double extractBotPoseEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName) {
        var poseEntry = LimelightHelpers.getLimelightNTTableEntry(limelightName, entryName);
        var poseArray = poseEntry.getDoubleArray(new double[0]);
        var pose = toPose2D(poseArray);
        double latency = extractBotPoseEntry(poseArray, 6);
        int tagCount = (int) extractBotPoseEntry(poseArray, 7);
        double tagSpan = extractBotPoseEntry(poseArray, 8);
        double tagDist = extractBotPoseEntry(poseArray, 9);
        double tagArea = extractBotPoseEntry(poseArray, 10);
        // getlastchange() in microseconds, ll latency in milliseconds
        var timestamp = (poseEntry.getLastChange() / 1000000.0) - (latency / 1000.0);
        return new PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea);
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static NetworkTableEntry getLimelightNtTableEntry(String tableName, String entryName) {
        return getLimelightNtTable(tableName).getEntry(entryName);
    }

    public static double getLimelightNtDouble(String tableName, String entryName) {
        return getLimelightNtTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNtDouble(String tableName, String entryName, double val) {
        getLimelightNtTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setLimelightNtDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNtTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getLimelightNtDoubleArray(String tableName, String entryName) {
        return getLimelightNtTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    public static String getLimelightNtString(String tableName, String entryName) {
        return getLimelightNtTableEntry(tableName, entryName).getString("");
    }

    public static URL getLimelightUrlString(String tableName, String request) {
        String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        }
        catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
    /////

    public static double getTx(String limelightName) {
        return getLimelightNtDouble(limelightName, "tx");
    }

    public static double getTy(String limelightName) {
        return getLimelightNtDouble(limelightName, "ty");
    }

    public static double getTa(String limelightName) {
        return getLimelightNtDouble(limelightName, "ta");
    }

    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNtDouble(limelightName, "tl");
    }

    public static double getLatency_Capture(String limelightName) {
        return getLimelightNtDouble(limelightName, "cl");
    }

    public static double getCurrentPipelineIndex(String limelightName) {
        return getLimelightNtDouble(limelightName, "getpipe");
    }

    public static String getJsonDump(String limelightName) {
        return getLimelightNtString(limelightName, "json");
    }

    /**
     * Switch to getBotPose.
     */
    @Deprecated
    public static double[] getBotpose(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed.
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue.
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "botpose_targetspace");
    }

    public static double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "camerapose_targetspace");
    }

    public static double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "targetpose_cameraspace");
    }

    public static double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "targetpose_robotspace");
    }

    public static double[] getTargetColor(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "tc");
    }

    public static double getFiducialId(String limelightName) {
        return getLimelightNtDouble(limelightName, "tid");
    }

    public static double getNeuralClassId(String limelightName) {
        return getLimelightNtDouble(limelightName, "tclass");
    }

    /////
    /////

    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "botpose");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getCameraPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNtDoubleArray(limelightName, "camerapose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement).
     */
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {

        double[] result = getBotPose_wpiBlue(limelightName);
        return toPose2D(result);
    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when
     * you are on the BLUE
     * alliance
     * 
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue");
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement).
     */
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {

        double[] result = getBotPose_wpiRed(limelightName);
        return toPose2D(result);

    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when
     * you are on the RED
     * alliance
     * 
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired");
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement).
     */
    public static Pose2d getBotPose2d(String limelightName) {

        double[] result = getBotPose(limelightName);
        return toPose2D(result);

    }

    public static boolean getTv(String limelightName) {
        return 1.0 == getLimelightNtDouble(limelightName, "tv");
    }

    /////
    /////

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNtDouble(limelightName, "pipeline", pipelineIndex);
    }

    public static void setPriorityTagID(String limelightName, int ID) {
        setLimelightNTDouble(limelightName, "priorityid", ID);
    }

    /**
     * The LEDs will be controlled by Limelight pipeline settings, and not by robot
     * code.
     */
    public static void setLedMode_PipelineControl(String limelightName) {
        setLimelightNtDouble(limelightName, "ledMode", 0);
    }

    public static void setLedMode_ForceOff(String limelightName) {
        setLimelightNtDouble(limelightName, "ledMode", 1);
    }

    public static void setLedMode_ForceBlink(String limelightName) {
        setLimelightNtDouble(limelightName, "ledMode", 2);
    }

    public static void setLedMode_ForceOn(String limelightName) {
        setLimelightNtDouble(limelightName, "ledMode", 3);
    }

    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNtDouble(limelightName, "stream", 0);
    }

    public static void setStreamMode_PipMain(String limelightName) {
        setLimelightNtDouble(limelightName, "stream", 1);
    }

    public static void setStreamMode_PipSecondary(String limelightName) {
        setLimelightNtDouble(limelightName, "stream", 2);
    }

    public static void setCameraMode_Processor(String limelightName) {
        setLimelightNtDouble(limelightName, "camMode", 0);
    }

    public static void setCameraMode_Driver(String limelightName) {
        setLimelightNtDouble(limelightName, "camMode", 1);
    }

    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    public static void setCropWindow(
            String limelightName,
            double cropXMin,
            double cropXMax,
            double cropYMin,
            double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropxMin;
        entries[1] = cropxMax;
        entries[2] = cropyMin;
        entries[3] = cropyMax;
        setLimelightNtDoubleArray(limelightName, "crop", entries);
    }

    public static void setCameraPose_RobotSpace(
            String limelightName,
            double forward,
            double side,
            double up,
            double roll,
            double pitch,
            double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNtDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    /////
    /////

    public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
        setLimelightNtDoubleArray(limelightName, "llrobot", outgoingPythonData);
    }

    public static double[] getPythonScriptData(String limelightName) {
        return getLimelightNtDoubleArray(limelightName, "llpython");
    }

    /////
    /////

    /**
     * Asynchronously take snapshot.
     */
    public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return synch_Takesnapshot(tableName, snapshotName);
        });
    }

    private static boolean synch_Takesnapshot(String tableName, String snapshotName) {
        URL url = getLimelightUrlString(tableName, "capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            }
            else {
                System.err.println("Bad LL Request");
            }
        }
        catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

    /**
     * Parses Limelight's JSON results dump into a LimelightResults Object.
     */
    public static LimelightResults getLatestResults(String limelightName) {

        long start = System.nanoTime();
        LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper()
                    .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
        }
        catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.m_targetingResults.m_latencyJsonParse = millis;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return results;
    }
}
