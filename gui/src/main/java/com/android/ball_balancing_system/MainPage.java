package com.android.ball_balancing_system;

import com.fazecast.jSerialComm.SerialPort;
import com.google.gson.Gson;
import com.google.gson.JsonParser;
import com.google.gson.JsonSyntaxException;
import javafx.animation.Animation;
import javafx.animation.AnimationTimer;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.*;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.paint.Color;
import javafx.util.Duration;
import java.io.*;
import java.net.Socket;
import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.util.concurrent.*;
import java.util.stream.Collectors;


public class MainPage implements Initializable {
    private static final int packet_size = 97;
    public static float desiredX = 0;
    public static float desiredY = 0;
    public static float currentX = 0;
    public static float currentY = 0;
    public static float current_velocity_X = 0;
    public static float current_velocity_Y = 0;
    public static float desired_velocity_X = 0;
    public static float desired_velocity_Y = 0;
    //private static final float safe_margin= 1.2F;
    public static int sendingFixedRate = 1;//(int)((packet_size * 10 * safe_margin*1000)/9600.0);
    public static Ball_System ballSystem;
    public static float Servo_X = 0;
    public static float Servo_Y = 0;
    // Grid range from -100 to 100 in both axes
    private final double MIN_VALUE = -13.5;
    private final double MAX_VALUE = 13.5;
    private final Color TRAJECTORY_COLOR = Color.GREEN;
    private final Color DESIRED_COLOR = Color.RED;
    private final Color CURRENT_COLOR = Color.BLUE;
    private final Color GRID_COLOR = Color.LIGHTGRAY;
    private final Color ORIGIN_COLOR = Color.BLACK;
    private final Color TOLERANCE_COLOR = Color.rgb(255, 0, 0, 0.2);
    private final StringBuilder messageBuffer = new StringBuilder();
    private final List<double[]> trajectoryPoints = new ArrayList<>();
    private final List<double[]> pathPoints = new ArrayList<>();
    ScheduledExecutorService sendToSTM32executor = Executors.newSingleThreadScheduledExecutor();
    private float base_angularVelocity = 3.49F;
    private float TOLERANCE = 0; 
    private byte operationMode;
    @FXML
    private Canvas plateCanvas;
    @FXML
    private Label desiredPositionLabel;
    @FXML
    private Label currentPositionLabel;
    @FXML
    private ChoiceBox<String> COM_choiceBox;
    @FXML
    private Slider trajectory_scale_Slider;
    @FXML
    private Slider PID_KP_Slider;
    @FXML
    private Slider PID_KI_Slider;
    @FXML
    private Slider PID_KD_Slider;
    @FXML
    private Slider PV_KP_Slider;
    @FXML
    private Slider PV_KV_Slider;
    @FXML
    private Label PID_KP_Label;
    @FXML
    private Label PID_KI_Label;
    @FXML
    private Label PID_KD_Label;
    @FXML
    private Label PV_KP_Label;
    @FXML
    private Label PV_KV_Label;
    @FXML
    private Label trajectory_scale_Label;
    @FXML
    private Label x_axis_Label;
    @FXML
    private Label y_axis_Label;
    @FXML
    private float alpha;
    @FXML
    private Label alpha_Label;
    @FXML
    private Slider alpha_Slider;
    @FXML
    private Slider x_axis_Slider;
    @FXML
    private Slider y_axis_Slider;
    @FXML
    private Slider tolerance_Slider;
    @FXML
    private Label tolerance_Label;
    @FXML
    private Button save_calibration_Button;
    @FXML
    private ToggleButton cameraTrackingToggleButton;
    @FXML
    private ToggleButton kalmanFilterToggleButton;

    @FXML
    private RadioButton PIDToggleButton;
    @FXML
    private TextField angularVelocity_TextField;
    @FXML
    private Button reset_calib_Button;
    @FXML
    private RadioButton PVToggleButton;
    @FXML
    private RadioButton CustomControl1ToggleButton;
    @FXML
    private RadioButton CustomControl2ToggleButton;
    @FXML
    private Slider speedSlider;
    @FXML
    private RadioButton automatic_RadioButton;
    @FXML
    private RadioButton manual_RadioButton;
    @FXML
    private RadioButton calibrate_RadioButton;
    @FXML
    private RadioButton idle_RadioButton;
    @FXML
    private RadioButton point_RadioButton;
    @FXML
    private RadioButton circle_RadioButton;
    @FXML
    private RadioButton draw_RadioButton;
    @FXML
    private RadioButton infinity_RadioButton;
    @FXML
    private Label speedSliderLabel;
    private float trajectory_scale;
    private double trajectorySpeedMultiplier = 0;
    private ExecutorService executorService = Executors.newSingleThreadExecutor();
    private SerialPort serialPort;
    private Process pythonProcess;
    private volatile boolean readingTerminal = false;
    private float PID_KP = -1.0f;
    private float PID_KI = -1.0f;
    private float PID_KD = -1.0f;
    private float PV_KP = -1.0f;
    private float PV_KV = -1.0f;
    private byte controlMode;
    private byte calibrationMode;
    private float manualAngleX;
    private float manualAngleY;
    private float calibAngleX;
    private float calibAngleY;
    private byte trajectoryMode = 0;
    private boolean sendingData = false;
    @FXML
    private AnchorPane graphsContainer;
    private KalmanFilter kf;
    private boolean kalmanFilterisEnabled;
    @FXML
    private Label currentVelocityLabel;
    @FXML
    private Label desiredVelocityLabel;
    // Previous position variables
    private ScheduledFuture<?> sendTask;

    public static void showAlert(Alert.AlertType type, String title, String message) {
        Alert alert = new Alert(type);
        alert.setTitle(title);
        alert.setHeaderText(null);
        alert.setContentText(message);
        alert.showAndWait();
    }
    private final Map<String, SerialPort> portMap = new HashMap<>();
    private List<String> lastPortNames = new ArrayList<>();

    public void populateAvailablePorts() {
        COM_choiceBox.getItems().clear();
        portMap.clear();

        for (SerialPort port : SerialPort.getCommPorts()) {
            String label = port.getDescriptivePortName();
            COM_choiceBox.getItems().add(label);
            portMap.put(label, port);
        }

        if (!COM_choiceBox.getItems().isEmpty()) {
            COM_choiceBox.setValue(COM_choiceBox.getItems().get(0));
        }
    }
    private Timeline refreshTimeline;

    private void checkPortChanges() {
        List<String> currentPortNames = Arrays.stream(SerialPort.getCommPorts())
                .map(SerialPort::getDescriptivePortName)
                .collect(Collectors.toList());

        if (!currentPortNames.equals(lastPortNames)) {
            lastPortNames = new ArrayList<>(currentPortNames);
            populateAvailablePorts();
        }
    }

    public void startPortAutoRefresh() {
        refreshTimeline = new Timeline(
                new KeyFrame(Duration.seconds(1), e -> checkPortChanges())
        );
        refreshTimeline.setCycleCount(Animation.INDEFINITE);
        refreshTimeline.play();
    }

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {

        System.out.printf("Sending fixed rate = %dms\n", sendingFixedRate);
        try {
            FXMLLoader graphsLoader = new FXMLLoader(getClass().getResource("Graphs.fxml"));
            AnchorPane graphsPane = graphsLoader.load();
            graphsContainer.getChildren().add(graphsPane);
            Graphs graphsLoaderController = graphsLoader.getController();
            graphsLoaderController.reset();
        } catch (IOException e) {
            e.printStackTrace();
        }

        drawPlate();
        startPortAutoRefresh();
        // Set up mouse click handler
        plateCanvas.setOnMouseClicked(this::handlePlateClick);
        plateCanvas.setOnMouseDragged(this::handlePlateClick);

        PV_KV_Slider.valueProperty().addListener((_, _, newValue) -> {
            PV_KV = newValue.floatValue();
            PV_KV_Label.setText(String.format("KD Y: %.3f", PV_KV));
        });

        PV_KP_Slider.valueProperty().addListener((_, _, newValue) -> {
            PV_KP = newValue.floatValue();
            PV_KP_Label.setText(String.format("KP Y: %.3f", PV_KP));
        });

        PID_KP_Slider.valueProperty().addListener((_, _, newValue) -> {
            PID_KP = newValue.floatValue();
            PID_KP_Label.setText(String.format("KP X: %.3f", PID_KP));
        });

        PID_KI_Slider.valueProperty().addListener((_, _, newValue) -> {
            PID_KI = newValue.floatValue();
            PID_KI_Label.setText(String.format("KI: %.3f", PID_KI));
        });

        PID_KD_Slider.valueProperty().addListener((_, _, newValue) -> {
            PID_KD = newValue.floatValue();
            PID_KD_Label.setText(String.format("KD X: %.3f", PID_KD));
        });

        x_axis_Slider.valueProperty().addListener((_, _, newValue) -> {
            x_axis_Label.setText(String.format("X-Axis: %.1f°", newValue.floatValue()));
            switch (operationMode) {
                case 1:
                    manualAngleX = newValue.floatValue();
                    break;
                case 2:
                    calibAngleX = newValue.floatValue();
                    break;
            }

            x_axis_Slider.setOnMouseReleased(_ -> {
                //sendToSTM32();
            });
        });
        y_axis_Slider.valueProperty().addListener((_, _, newValue) -> {
            y_axis_Label.setText(String.format("Y-Axis: %.1f°", newValue.floatValue()));
            switch (operationMode) {
                case 1: //MANUAL
                    manualAngleY = newValue.floatValue();
                    break;
                case 2://CALIBRATE
                    calibAngleY = newValue.floatValue();
                    break;

            }
            y_axis_Slider.setOnMouseReleased(_ -> {
                //sendToSTM32();
            });
        });
        alpha_Slider.setValue(0.5);
        alpha = 0.5f;
        alpha_Label.setText(String.format("EMA Filter α = %.2f", alpha));
        alpha_Slider.valueProperty().addListener((_, _, newValue) -> {
            alpha = newValue.floatValue();
            alpha_Label.setText(String.format("EMA Filter α = %.2f", alpha));
        });
        tolerance_Slider.valueProperty().addListener((_, _, newValue) -> {
            TOLERANCE = (float) ((newValue.floatValue() / 100.0) * MAX_VALUE);
            tolerance_Label.setText(String.format("Tolerance: ±%.1fcm", TOLERANCE));
            drawPlate();
            tolerance_Slider.setOnMouseReleased(_ -> {
                //sendToSTM32();
            });
        });
        trajectory_scale_Slider.valueProperty().addListener((_, _, newValue) -> {
            trajectory_scale_Label.setText(String.format("Scale: %d%%", newValue.intValue()));
            trajectory_scale = (float) (newValue.floatValue() / 100.0);
            //drawPlate();
            trajectory_scale_Slider.setOnMouseReleased(_ -> {
                //sendToSTM32();
            });
        });
        speedSlider.valueProperty().addListener((_, _, newValue) -> {
            int percentage = newValue.intValue();
            speedSliderLabel.setText(String.format("Speed: %d%%", percentage));
            trajectorySpeedMultiplier = percentage / 100.0;
        });
        ToggleGroup control_toggleGroup = new ToggleGroup();
        PIDToggleButton.setToggleGroup(control_toggleGroup);
        PVToggleButton.setToggleGroup(control_toggleGroup);
        CustomControl1ToggleButton.setToggleGroup(control_toggleGroup);
        CustomControl2ToggleButton.setToggleGroup(control_toggleGroup);


        ToggleGroup servo_modes_toggleGroup = new ToggleGroup();
        automatic_RadioButton.setToggleGroup(servo_modes_toggleGroup);
        manual_RadioButton.setToggleGroup(servo_modes_toggleGroup);
        calibrate_RadioButton.setToggleGroup(servo_modes_toggleGroup);
        idle_RadioButton.setToggleGroup(servo_modes_toggleGroup);
        control_toggleGroup.selectedToggleProperty().addListener((_, _, newVal) -> {
            if (newVal == null) {
                PIDToggleButton.setSelected(true);
                return;
            }

            if (newVal == PIDToggleButton) {
                controlMode = (byte) ControlMode.PID.ordinal();
            } else if (newVal == PVToggleButton) {
                controlMode = (byte) ControlMode.PV.ordinal();
            } else if (newVal == CustomControl1ToggleButton) {
                controlMode = (byte) ControlMode.CUSTOM1.ordinal();
            } else if (newVal == CustomControl2ToggleButton) {
                controlMode = (byte) ControlMode.CUSTOM2.ordinal();
            }
            //sendToSTM32();
        });
        Chirp_toggle.setSelected(false);
        PBRS_toggle.setSelected(false);
        PBRS_toggle.setDisable(true);
        Chirp_toggle.setDisable(true);
        PBRS_toggle.setOpacity(0.5);
        Chirp_toggle.setOpacity(0.5);
        ToggleGroup trajectory_toggleGroup = new ToggleGroup();
        point_RadioButton.setToggleGroup(trajectory_toggleGroup);
        circle_RadioButton.setToggleGroup(trajectory_toggleGroup);
        infinity_RadioButton.setToggleGroup(trajectory_toggleGroup);
        draw_RadioButton.setToggleGroup(trajectory_toggleGroup);
        point_RadioButton.setSelected(true);
        PIDToggleButton.setSelected(true);
        // Set default selection
        idle_RadioButton.setSelected(true);
        trajectory_toggleGroup.selectedToggleProperty().addListener((_, _, newVal) -> {
            pathToggleButton.setDisable(true);
            pathToggleButton.setOpacity(0.5);
            clearTrajectory();
            if (newVal == null) {
                trajectoryMode=0;
                point_RadioButton.setSelected(true);
                return;
            }

            if (newVal == point_RadioButton) {
                trajectoryMode=0;
            } else if (newVal == circle_RadioButton) {
                trajectoryMode=1;
            } else if (newVal == infinity_RadioButton) {
                trajectoryMode=2;
            }
            else if (newVal == draw_RadioButton) {
                trajectoryMode=3;
                pathToggleButton.setDisable(false);
                pathToggleButton.setOpacity(1);
            }
            //sendToSTM32();
        });
        // Add listener for mode changes
        servo_modes_toggleGroup.selectedToggleProperty().addListener((_, _, newVal) -> {
            if (newVal == null) {
                // If somehow nothing is selected, revert to automatic
                idle_RadioButton.setSelected(true);
                return;
            }

            if (newVal == automatic_RadioButton) {
                handleModeChange(OperationMode.AUTOMATIC);
            } else if (newVal == manual_RadioButton) {
                handleModeChange(OperationMode.MANUAL);
            } else if (newVal == calibrate_RadioButton) {
                handleModeChange(OperationMode.CALIBRATE);
            } else if (newVal == idle_RadioButton) {
                handleModeChange(OperationMode.IDLE);
            }
            sendToSTM32();
        });
        enableIdleMode();
        smoothButton.addEventHandler(MouseEvent.MOUSE_PRESSED, _ -> startSmoothingLoop());
        smoothButton.addEventHandler(MouseEvent.MOUSE_RELEASED, _ -> stopSmoothingLoop());
        Platform.runLater(() -> {
            Scene scene = cameraTrackingToggleButton.getScene();
            scene.windowProperty().addListener((_, _, newWindow) -> {
                if (newWindow != null) {
                    newWindow.setOnCloseRequest(_ -> stopReadingTerminalOutput());
                }
            });
        });
        PID_KP_Slider.setValue(0.95);
        PID_KI_Slider.setValue(0.089);
        PID_KD_Slider.setValue(0.708);
        PV_KP_Slider.setValue(0.95);
        PV_KV_Slider.setValue(0.708);
    }
    private Timeline smoothTimeline;
    private void startSmoothingLoop() {
        if (smoothTimeline != null && smoothTimeline.getStatus() == Timeline.Status.RUNNING)
            return;

        smoothTimeline = new Timeline(
                new KeyFrame(Duration.millis(30), _ -> smoothPathPoints())
        );
        smoothTimeline.setCycleCount(Timeline.INDEFINITE);
        smoothTimeline.play();
    }

    private void stopSmoothingLoop() {
        if (smoothTimeline != null) {
            smoothTimeline.stop();
        }
    }
    private Timeline prbsTimeline;

    public void startPRBSInput() {
        prbsTimeline = new Timeline();

        double center = 0;      
        double amplitude = 16.4*alpha;      
        double interval = 0.01;        
        int durationInSeconds = 10;
        double frequency = 0.1;      

        int steps = (int) (durationInSeconds / interval);

        for (int i = 0; i < steps; i++) {
            double t = i * interval;
            double value = center + amplitude * Math.sin(2 * Math.PI * frequency * t);

            prbsTimeline.getKeyFrames().add(new KeyFrame(
                    Duration.seconds(t),
                    _ -> x_axis_Slider.setValue(value)
            ));
        }

        prbsTimeline.setCycleCount(1);
        prbsTimeline.play();
    }
    private Timeline chirpTimeline;
    public void startChirpInput() {
        chirpTimeline = new Timeline();

        double center = 0;
        double amplitude = 16.4*alpha;
        double interval = 0.2;
        int durationInSeconds = 10;
        double startFreq = 0.001;
        double endFreq = 0.10;

        int steps = (int) (durationInSeconds / interval);

        for (int i = 0; i < steps; i++) {
            double t = i * interval;
            double freq = startFreq + (endFreq - startFreq) * (t / durationInSeconds);
            double angle = center + amplitude * Math.sin(2 * Math.PI * freq * t);

            chirpTimeline.getKeyFrames().add(new KeyFrame(
                    Duration.seconds(t),
                    _ -> x_axis_Slider.setValue(angle)
            ));
        }

        chirpTimeline.setCycleCount(1);
        chirpTimeline.play();
    }
    @FXML
    private ToggleButton PBRS_toggle;
    @FXML
    private ToggleButton Chirp_toggle;
    public void stopInputs() {
        if (prbsTimeline != null) prbsTimeline.stop();
        if (chirpTimeline != null) chirpTimeline.stop();
    }
    @FXML
    private void togglePRBS(){
        stopInputs();
        Chirp_toggle.setSelected(false);
        if(PBRS_toggle.isSelected()){
            startPRBSInput();
        }

    }
    @FXML
    private void toggleChirp(){
        stopInputs();
        PBRS_toggle.setSelected(false);
        if(Chirp_toggle.isSelected()){
            startChirpInput();
        }

    }
    @FXML
    private void updateBaseAngularVelocity() {
        try {
            base_angularVelocity = (float) (Float.parseFloat(angularVelocity_TextField.getText()) * Math.PI / 180.0);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @FXML
    private void toggleKalman() {
        kalmanFilterisEnabled = kalmanFilterToggleButton.isSelected();
        if (kalmanFilterisEnabled) {
            kalmanFilterToggleButton.setText("Disable Kalman Filter");
        } else {
            kalmanFilterToggleButton.setText("Enable Kalman Filter");
        }
    }

    private void enableTuning() {
        PID_KP_Slider.setDisable(false);
        PID_KI_Slider.setDisable(false);
        PID_KD_Slider.setDisable(false);
        PV_KP_Slider.setDisable(false);
        PV_KV_Slider.setDisable(false);
        PID_KP_Slider.setOpacity(1);
        PID_KI_Slider.setOpacity(1);
        PID_KD_Slider.setOpacity(1);
        PV_KP_Slider.setOpacity(1);
        PV_KV_Slider.setOpacity(1);
        PIDToggleButton.setDisable(false);
        PVToggleButton.setDisable(false);
        CustomControl1ToggleButton.setDisable(false);
        CustomControl2ToggleButton.setDisable(false);
        PIDToggleButton.setOpacity(1);
        PVToggleButton.setOpacity(1);
        CustomControl1ToggleButton.setOpacity(1);
        CustomControl2ToggleButton.setOpacity(1);
    }

    private void disableTuning() {
        PID_KP_Slider.setDisable(true);
        PID_KI_Slider.setDisable(true);
        PID_KD_Slider.setDisable(true);
        PV_KP_Slider.setDisable(true);
        PV_KV_Slider.setDisable(true);
        PID_KP_Slider.setOpacity(0.5);
        PID_KI_Slider.setOpacity(0.5);
        PID_KD_Slider.setOpacity(0.5);
        PV_KP_Slider.setOpacity(0.5);
        PV_KV_Slider.setOpacity(0.5);
        PIDToggleButton.setDisable(true);
        PVToggleButton.setDisable(true);
        CustomControl1ToggleButton.setDisable(true);
        CustomControl2ToggleButton.setDisable(true);
        PIDToggleButton.setOpacity(0.5);
        PVToggleButton.setOpacity(0.5);
        CustomControl1ToggleButton.setOpacity(0.5);
        CustomControl2ToggleButton.setOpacity(0.5);

    }

    private void handleModeChange(OperationMode newMode) {
        save_calibration_Button.setDisable(true);
        save_calibration_Button.setOpacity(0.5);
        point_RadioButton.setDisable(true);
        circle_RadioButton.setDisable(true);
        draw_RadioButton.setDisable(true);
        infinity_RadioButton.setDisable(true);
        point_RadioButton.setOpacity(0.5);
        circle_RadioButton.setOpacity(0.5);
        draw_RadioButton.setOpacity(0.5);
        pathToggleButton.setDisable(true);
        pathToggleButton.setOpacity(0.5);
        clearTrajectory();
        infinity_RadioButton.setOpacity(0.5);
        x_axis_Slider.setValue(0);
        y_axis_Slider.setValue(0);
        x_axis_Slider.setMax(35);
        x_axis_Slider.setMin(-35);
        y_axis_Slider.setMax(35);
        y_axis_Slider.setMin(-35);
        Chirp_toggle.setSelected(false);
        PBRS_toggle.setSelected(false);
        PBRS_toggle.setDisable(true);
        Chirp_toggle.setDisable(true);
        PBRS_toggle.setOpacity(0.5);
        Chirp_toggle.setOpacity(0.5);
        trajectoryMode = 0;
        disableTuning();
        switch (newMode) {
            case IDLE:
                operationMode = (byte) OperationMode.IDLE.ordinal();
                enableIdleMode();
                break;
            case MANUAL:
                operationMode = (byte) OperationMode.MANUAL.ordinal();
                enableManualMode();
                PBRS_toggle.setDisable(false);
                Chirp_toggle.setDisable(false);
                PBRS_toggle.setOpacity(1);
                Chirp_toggle.setOpacity(1);
                break;
            case CALIBRATE:
                operationMode = (byte) OperationMode.CALIBRATE.ordinal();
                enableCalibrateMode();
                break;
            case AUTOMATIC:
                operationMode = (byte) OperationMode.AUTOMATIC.ordinal();
                enableAutomaticMode();
                break;
        }
    }

    private void enableIdleMode() {
        save_calibration_Button.setDisable(true);
        save_calibration_Button.setOpacity(0.5);
        reset_calib_Button.setDisable(true);
        pathToggleButton.setDisable(true);
        pathToggleButton.setOpacity(0.5);
        reset_calib_Button.setOpacity(0.5);
        x_axis_Slider.setDisable(true);
        y_axis_Slider.setDisable(true);
        x_axis_Slider.setOpacity(0.5);
        y_axis_Slider.setOpacity(0.5);
        point_RadioButton.setDisable(true);
        circle_RadioButton.setDisable(true);
        infinity_RadioButton.setDisable(true);
        point_RadioButton.setOpacity(0.5);
        circle_RadioButton.setOpacity(0.5);
        infinity_RadioButton.setOpacity(0.5);
        draw_RadioButton.setDisable(true);
        draw_RadioButton.setOpacity(0.5);
        //stopTrajectoryMotion();
        disableTuning();
    }

    private void enableAutomaticMode() {
        save_calibration_Button.setDisable(true);
        save_calibration_Button.setOpacity(0.5);
        reset_calib_Button.setDisable(true);
        reset_calib_Button.setOpacity(0.5);
        pathToggleButton.setDisable(true);
        pathToggleButton.setOpacity(0.5);
        x_axis_Slider.setDisable(true);
        point_RadioButton.setDisable(false);
        point_RadioButton.setSelected(true);
        enableTuning();
        y_axis_Slider.setDisable(true);
        x_axis_Slider.setOpacity(0.5);
        y_axis_Slider.setOpacity(0.5);
        trajectoryMode = 0;
        circle_RadioButton.setDisable(false);
        infinity_RadioButton.setDisable(false);
        point_RadioButton.setOpacity(1);
        circle_RadioButton.setOpacity(1);
        infinity_RadioButton.setOpacity(1);
        draw_RadioButton.setDisable(false);
        draw_RadioButton.setOpacity(1);

    }

    private void enableManualMode() {
        save_calibration_Button.setDisable(true);
        save_calibration_Button.setOpacity(0.5);
        reset_calib_Button.setDisable(true);
        reset_calib_Button.setOpacity(0.5);
        x_axis_Slider.setDisable(false);
        pathToggleButton.setDisable(true);
        pathToggleButton.setOpacity(0.5);
        y_axis_Slider.setDisable(false);
        x_axis_Slider.setOpacity(1);
        y_axis_Slider.setOpacity(1);
        point_RadioButton.setDisable(true);
        circle_RadioButton.setDisable(true);
        infinity_RadioButton.setDisable(true);
        point_RadioButton.setOpacity(0.5);
        circle_RadioButton.setOpacity(0.5);
        infinity_RadioButton.setOpacity(0.5);
        draw_RadioButton.setDisable(true);
        draw_RadioButton.setOpacity(0.5);
        //stopTrajectoryMotion();
        trajectoryMode = 0;
//        systemMessages.appendText("\nSwitched to Manual mode");
    }

    private void enableCalibrateMode() {
        x_axis_Slider.setDisable(false);
        y_axis_Slider.setDisable(false);
        x_axis_Slider.setMax(90);
        x_axis_Slider.setMin(-90);
        y_axis_Slider.setMax(90);
        y_axis_Slider.setMin(-90);
        x_axis_Slider.setOpacity(1);
        pathToggleButton.setDisable(true);
        pathToggleButton.setOpacity(0.5);
        y_axis_Slider.setOpacity(1);
        save_calibration_Button.setDisable(false);
        save_calibration_Button.setOpacity(1);
        reset_calib_Button.setDisable(false);
        reset_calib_Button.setOpacity(1);
        point_RadioButton.setDisable(true);
        circle_RadioButton.setDisable(true);
        infinity_RadioButton.setDisable(true);
        point_RadioButton.setOpacity(0.5);
        circle_RadioButton.setOpacity(0.5);
        infinity_RadioButton.setOpacity(0.5);
        draw_RadioButton.setDisable(true);
        draw_RadioButton.setOpacity(0.5);
        //stopTrajectoryMotion();
        calibrationMode = (byte) CalibrationMode.IDLE.ordinal();
//        systemMessages.appendText("\nSwitched to Calibration mode");
    }

//    public void stopReadingTerminalOutput() {
//        readingTerminal = false;
//        trajectoryMode=0;
//        if (pythonProcess != null) {
//            pythonProcess.destroy();
//        }
//        if (outputReaderThread != null) {
//            outputReaderThread.interrupt();
//        }
//    }

    @FXML
    private void calibrate_servo() throws InterruptedException {
        pauseSending();
        calibrationMode = (byte) CalibrationMode.SAVE.ordinal();
        Thread.sleep(100);
        sendToSTM32();
        Thread.sleep(100);
        x_axis_Slider.setValue(0);
        y_axis_Slider.setValue(0);
        calibrationMode = (byte) CalibrationMode.IDLE.ordinal();
        resumeSending();
    }

    @FXML
    private void reset_calibration() throws InterruptedException {
        pauseSending();
        calibrationMode = (byte) CalibrationMode.RESET.ordinal();
        Thread.sleep(100);
        sendToSTM32();
        Thread.sleep(100);
        x_axis_Slider.setValue(0);
        y_axis_Slider.setValue(0);
        calibAngleX = 0;
        calibAngleY = 0;
        calibrationMode = (byte) CalibrationMode.IDLE.ordinal();
        resumeSending();
    }

    public byte[] toByteArray() {
        //each float = 4 Bytes,
        ByteBuffer buffer = ByteBuffer.allocate(packet_size).order(ByteOrder.LITTLE_ENDIAN);
        buffer.putFloat(TOLERANCE);//4 bytes
        buffer.putFloat(desiredX);//4 bytes
        buffer.putFloat(desiredY);//4 bytes
        buffer.putFloat(current_velocity_X);//4 bytes
        buffer.putFloat(current_velocity_Y);//4 bytes
        buffer.putFloat(desired_velocity_X);//4 bytes
        buffer.putFloat(desired_velocity_Y);//4 bytes
        buffer.putFloat(calibAngleX);//4 bytes
        buffer.putFloat(calibAngleY);//4 bytes
        buffer.put(operationMode); //1 byte
        buffer.put(calibrationMode); //1 byte
        buffer.put(controlMode);//1 byte
        buffer.put(trajectoryMode);//1 byte
        buffer.putFloat(manualAngleX);//4 bytes
        buffer.putFloat(manualAngleY);//4 bytes
        buffer.putFloat(currentX);//4 bytes
        buffer.putFloat(currentY);//4 bytes
        buffer.putFloat(PID_KP);//4 bytes
        buffer.putFloat(PID_KI);//4 bytes
        buffer.putFloat(PID_KD);//4 bytes
        buffer.putFloat(PV_KP);//4 bytes
        buffer.putFloat(PV_KV);//4 bytes
        buffer.putFloat(alpha);//4 bytes
        buffer.putFloat(trajectory_scale);//4 bytes
        buffer.putFloat((float) trajectorySpeedMultiplier); //4
        buffer.putFloat(base_angularVelocity); //4
        buffer.putFloat(dt); //4
        buffer.put((byte) (ball_detected ? 1 : 0));
        return buffer.array();
    }

//    private void updateCircularMotion() {
//        long currentTime = System.nanoTime();
//        double dt = (currentTime - lastUpdateTime) / 1.0e9; // Convert ns to seconds
//        lastUpdateTime = currentTime; // Update last time
//        if (!isWithinTolerance() && dt > 0) { // Ensure dt is positive
//            currentAngle += angularVelocity * dt; // Use time-based angle update
//
//            // Compute new position
//            double x = circleCenterX + circleRadius * Math.cos(currentAngle);
//            double y = circleCenterY + circleRadius * Math.sin(currentAngle);
//
//            // Compute velocity using finite differences
//            desired_velocity_X = (float) ((x - desiredX) / dt);
//            desired_velocity_Y = (float) ((y - desiredY) / dt);
//
//
//            setDesiredPosition(x, y);
//        }
//    }
    private Socket matlabSocket;
    private DataInputStream matlabIn;
    private DataOutputStream matlabOut;
    private Thread matlabThread;
    private volatile boolean running = false;

    @FXML
    public void connectToMatlab() {
        try {
            matlabSocket = new Socket("localhost", 23456);
            matlabIn = new DataInputStream(matlabSocket.getInputStream());
            matlabOut = new DataOutputStream(matlabSocket.getOutputStream());
            System.out.println("Connected to MATLAB");
            startMatlabListener();
        } catch (IOException e) {
            System.err.println("MATLAB connection failed");
        }
    }

    private void startMatlabListener() {
        running = true;
        matlabThread = new Thread(() -> {
            try {
                while (running && !matlabSocket.isClosed()) {
                    float position = 0; 
                    matlabOut.writeFloat(position);
                    matlabOut.writeFloat(desiredX);
                    matlabOut.writeFloat(desiredY);
                    matlabOut.writeFloat(currentX);
                    matlabOut.writeFloat(currentY);
                    matlabOut.writeFloat(current_velocity_X);
                    matlabOut.writeFloat(current_velocity_Y);
                    matlabOut.flush();
                    manualAngleX = matlabIn.readFloat();
                    manualAngleY = matlabIn.readFloat();
                    System.out.println("X: " + manualAngleX + ", Y: " + manualAngleY);

                    Thread.sleep(5);
                }
            } catch (IOException | InterruptedException e) {
                if (running) {
                    e.printStackTrace();
                } // If not running, probably intentional disconnect
            }
        });
        matlabThread.setDaemon(true);
        matlabThread.start();
    }

    @FXML
    public void disconnectFromMatlab() {
        running = false;
        try {
            if (matlabSocket != null && !matlabSocket.isClosed()) {
                matlabSocket.close();
            }
            if (matlabIn != null) matlabIn.close();
            if (matlabOut != null) matlabOut.close();
            if (matlabThread != null && matlabThread.isAlive()) {
                matlabThread.join(); 
            }
            System.out.println("Disconnected from MATLAB.");
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    JavaSocketClient javaSocketClient=new JavaSocketClient();
    public void sendToPython(){
        if(javaSocketClient.isConnected()){
            javaSocketClient.sendData(desiredX,desiredY,currentX,currentY,current_velocity_X,current_velocity_Y);
        }
    }
    JavaSocketClient.SharedServoValues sharedServoValues=new JavaSocketClient.SharedServoValues();
    @FXML
    public void connectToPython(){
        javaSocketClient.connect();
        if(javaSocketClient.isConnected()){
            javaSocketClient.startListeningInBackground(sharedServoValues);
        }
    }
    @FXML
    public void disconnectedFromPython(){
        javaSocketClient.disconnect();
    }



    public void sendToSTM32() {
        if (serialPort == null || !serialPort.isOpen() || sendingData) {
            return;
        }
        new Thread(() -> {
            synchronized (serialPort) { // Thread-safe serial access
                sendingData = true;
                try {
                    sendToPython();
                    if(operationMode==OperationMode.MANUAL.ordinal()){
                        if(javaSocketClient.isConnected()){
                            double[] servos=sharedServoValues.get();
                            manualAngleX= (float) servos[0];
                            manualAngleY= (float) servos[1];
                        }

                    }
                    byte[] data = toByteArray();
                    byte checksum = calculateChecksum(data);

                    serialPort.setComPortTimeouts(
                            SerialPort.TIMEOUT_READ_SEMI_BLOCKING,
                            5000,  // Read timeout
                            5000   // Write timeout
                    );

                    // Send header
                    serialPort.getOutputStream().write(new byte[]{(byte) 0xAA, (byte) 0x55});
                    int chunkSize = 8;
                    for (int i = 0; i < data.length; i += chunkSize) {
                        int length = Math.min(chunkSize, data.length - i);
                        serialPort.getOutputStream().write(data, i, length);
                        Thread.sleep(1); // Reduce buffer pressure
                    }

                    // Send checksum
                    serialPort.getOutputStream().write(checksum);
                    serialPort.getOutputStream().flush();

                } catch (Exception e) {
                    System.err.println("UART Error: " + e.getClass().getSimpleName() +
                            " - " + e.getMessage());
                    close();

                } finally {
                    sendingData = false;
                }
            }
        }).start();
    }

    private byte calculateChecksum(byte[] data) {
        byte checksum = 0;
        for (byte b : data) {
            checksum ^= b; 
        }
        return checksum;
    }

    @FXML
    private void toggleCameraTracking() {
        if (cameraTrackingToggleButton.isSelected()) {
            // Button was toggled ON
            startReadingTerminalOutput();
            cameraTrackingToggleButton.setText("Stop Camera Tracking");
//            systemMessages.appendText("\nStarted reading ball position from camera...");
        } else {
            stopReadingTerminalOutput();
            cameraTrackingToggleButton.setText("Start Camera Tracking");
            currentX=0;
            currentY=0;
            current_velocity_X=0;
            current_velocity_Y=0;
            updateLabels();
            drawPlate();
//            systemMessages.appendText("\nStopped camera tracking.");
            //stopAllMotions();
        }
    }
    private KalmanFilter kf_v;
    private KalmanFilter kf_laser;
    private float dt;
    private boolean ball_detected=false;
    private boolean laser_detected=false;
    private static int ball_counter=0;
    public void startReadingTerminalOutput() {
        if (readingTerminal) return;

        readingTerminal = true;
        // x,y,vx,vy
        Thread outputReaderThread = new Thread(() -> {
            try {
                String workingDir = System.getProperty("user.dir");
                String[] command = {"python", "-u", workingDir + File.separator + "scripts" + File.separator + "ball_tracker2.py"};

                pythonProcess = Runtime.getRuntime().exec(command);

                BufferedReader reader = new BufferedReader(
                        new InputStreamReader(pythonProcess.getInputStream())
                );

                BufferedReader errorReader = new BufferedReader(
                        new InputStreamReader(pythonProcess.getErrorStream())
                );

                // Error reader thread
                Thread errorReaderThread = new Thread(() -> {
                    String errorLine;
                    try {
                        while ((errorLine = errorReader.readLine()) != null) {
                            final String errorMessage = errorLine;
                            Platform.runLater(() -> {
                                System.out.println("\n[Python Error] " + errorMessage);
                            });
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                });
                errorReaderThread.setDaemon(true);
                errorReaderThread.start();

                String line;
                while (readingTerminal && (line = reader.readLine()) != null) {
                    final String message = line;
                    Platform.runLater(() -> {
                        try {
                            //System.out.println(message);
                            String[] data = message.split(",");
                            if (data.length == 9) {  // x,y,vx,vy,dt,ball_detected,laser_detected,laser_x,laser_y

                                double x = -Double.parseDouble(data[1]) / 10;
                                double y = Double.parseDouble(data[0]) / 10;
                                double vx = -Double.parseDouble(data[3]) / 10;
                                double vy = Double.parseDouble(data[2]) / 10;
                                dt=Float.parseFloat(data[4]);
                                ball_detected=Boolean.parseBoolean(data[5]);
                                laser_detected=Boolean.parseBoolean(data[6]);
                                double laser_x=-Double.parseDouble(data[8]) / 10;
                                double laser_y=Double.parseDouble(data[7]) / 10;
                                //System.out.println(dt);
                                //x = vx;
                                //y = vy;

                                if (kalmanFilterisEnabled) {
                                    if (ball_detected) {
                                        ball_counter = 0;
                                        if (kf == null) {
                                            kf = new KalmanFilter(x, y);
                                            kf_v = new KalmanFilter(vx, vy, 0.01, 0.3);
                                            kf_laser = new KalmanFilter(laser_x, laser_y, 0.001, 5);
                                        } else {
                                            kf.enta_shayf_eh();
                                            kf_v.enta_shayf_eh();
                                            kf_v.khod_faltar(vx, vy);
                                            kf.khod_faltar(x, y);
                                            kf_laser.enta_shayf_eh();
                                            kf_laser.khod_faltar(laser_x, laser_y);

                                            // Get filtered values
                                            x = kf.getX();
                                            y = kf.getY();
                                            vx = kf_v.getX();
                                            vy = kf_v.getY();
                                            laser_x = kf_laser.getX();
                                            laser_y = kf_laser.getY();
                                        }
                                    } else if (ball_counter == 0) {
                                        setDesiredPosition(0, 0);
                                        ball_counter++;
                                        kf = null;
                                        kf_v = null;
                                        kf_laser = null;
                                        x=0;
                                        y=0;
                                        vx=0;
                                        vy=0;
                                        laser_x=0;
                                        laser_y=0;
                                    }
                                }
                                if (laser_detected && laserToggleButton.isSelected()) {
                                    setDesiredPosition(laser_x, laser_y);
                                }
                                if (trajectoryMode != 0 && auto_speed_toggle.isSelected()) {
                                    pid_speed_controller();
                                }

                                if(trajectoryMode!=0 && auto_speed_toggle.isSelected()){
                                    pid_speed_controller();
                                }
                                updateCurrentPosition(x, y);
                                updateCurrentVelocity((float) vx, (float) vy);
                                double v_total = Math.sqrt(vx * vx + vy * vy);
                                currentVelocityLabel.setText(
                                        String.format("Current Velocity: (%.1f, %.1f) = %.1fcm/s", vx, vy, v_total));
                            }
                        } catch (NumberFormatException e) {
                            System.out.println("\nInvalid data: " + message);
                        }
                    });
                }
            } catch (IOException e) {
                Platform.runLater(() -> {
                    if (readingTerminal) {
                        System.out.println("\nError: " + e.getMessage());
                        cameraTrackingToggleButton.setSelected(false);
                        cameraTrackingToggleButton.setText("Start Tracking");
                    }
                });
            }
        });
        outputReaderThread.setDaemon(true);
        outputReaderThread.start();
    }
    @FXML
    private ToggleButton auto_speed_toggle;
    private void stopReadingTerminalOutput() {
        readingTerminal = false;
        if (pythonProcess != null) {
            pythonProcess.destroy();
            pythonProcess = null;
        }
        Platform.runLater(() -> {
            cameraTrackingToggleButton.setSelected(false);
            cameraTrackingToggleButton.setText("Start Tracking");
            updateCurrentVelocity((float) 0, (float) 0);

            currentVelocityLabel.setText(
                    String.format("Current Velocity: (%.1f, %.1f) = %.1fcm/s", 0.0, 0.0, 0.0));
        });


    }
    private double integratorX = 0.0;
    private double integratorY = 0.0;
    private double lastErrorX = 0.0;
    private double lastErrorY = 0.0;

    private void pid_speed_controller() {
        double kp = 1.7;
        double ki = 0.01;
        double kd = 0.1;

        double errorX = (desiredX - currentX);
        double proportionalX = errorX * kp;
        double derivativeX = (errorX - lastErrorX) / dt;
        integratorX += errorX * dt;
        double controlX = proportionalX + (ki * integratorX) + (kd * derivativeX);
        lastErrorX = errorX;

        double errorY = (desiredY - currentY);
        double proportionalY = errorY * kp;
        double derivativeY = (errorY - lastErrorY) / dt;
        integratorY += errorY * dt;
        double controlY = proportionalY + (ki * integratorY) + (kd * derivativeY);
        lastErrorY = errorY;

        double result = -Math.sqrt(controlX * controlX + controlY * controlY);
        result+=50;
        result = Math.max(0, Math.min(100, result));

        speedSlider.setValue(result);

    }


    // Velocity update method
    public void updateCurrentVelocity(float vx, float vy) {
        current_velocity_X = vx;
        current_velocity_Y = vy;
        // Add any additional logic (e.g., filtering) here
    }

    @FXML
    public void startInfinityMotion() {
        trajectoryMode = 2;
//        double centerX = 0;
//        double centerY = 0;
//        if (currentX != 0 || currentY != 0) {
//            double dx = currentX - centerX;
//            double dy = currentY - centerY;
//            infinityPhase = Math.atan2(dy, dx);
//        } else {
//            infinityPhase = 0;
//        }
//        isInfinityMotionActive = true;
//        isCircularMotionActive = false;
//        this.circleCenterX = centerX;
//        this.circleCenterY = centerY;
//        infinityMotionTimer.start();
    }

    @FXML
    public void startCircularMotion() {
        trajectoryMode = 1;
//        //can be customized
//        double centerX = 0;
//        double centerY = 0;
//        // start motion near the circle
//        if (currentX != 0 || currentY != 0) {
//            double dx = currentX - centerX;
//            double dy = currentY - centerY;
//            double distance = Math.sqrt(dx * dx + dy * dy);
//
//            if (Math.abs(distance - circleRadius) < 5) {
//                currentAngle = Math.atan2(dy, dx);
//            } else {
//                // Start from closest point on circle
//                currentAngle = Math.atan2(dy, dx);
//                currentX = (float) (centerX + circleRadius * Math.cos(currentAngle));
//                currentY = (float) (centerY + circleRadius * Math.sin(currentAngle));
//            }
//        } else {
//            currentAngle = 0;
//        }
//        isCircularMotionActive = true;
//        isInfinityMotionActive = false;
//        this.circleCenterX = centerX;
//        this.circleCenterY = centerY;
//        circularMotionTimer.start();
    }

    private void handlePlateClick(MouseEvent event) {
        if(!(pathToggleButton.isSelected() || laser_detected)){
            double[] logicalPos = screenToLogical(event.getX(), event.getY());
            setDesiredPosition(logicalPos[0], logicalPos[1]);
            desired_velocity_X = 0;
            desired_velocity_Y = 0;
        }

    }

    public void pauseSending() {
        if (sendTask != null && !sendTask.isCancelled()) {
            sendTask.cancel(true);  // Cancel without interrupting
            try {
                Thread.sleep(100);  // Small delay to ensure the task is properly canceled
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void resumeSending() {
        if (sendTask == null || sendTask.isCancelled()) {
            sendTask = sendToSTM32executor.scheduleAtFixedRate(
                    this::sendToSTM32,
                    0, sendingFixedRate, TimeUnit.MILLISECONDS
            );
        }
    }

    @FXML
    public void connect() {
        String selectedLabel = COM_choiceBox.getValue();
        if (selectedLabel == null || !portMap.containsKey(selectedLabel)) return;

        if (serialPort != null && serialPort.isOpen()) {
            close();
        }
        serialPort = portMap.get(selectedLabel);
        serialPort.setBaudRate(921600);
        serialPort.setNumDataBits(8);
        serialPort.setNumStopBits(SerialPort.ONE_STOP_BIT);
        serialPort.setParity(SerialPort.NO_PARITY);
        serialPort.setComPortTimeouts(
                SerialPort.TIMEOUT_READ_BLOCKING | SerialPort.TIMEOUT_WRITE_BLOCKING,
                5000,  // Read timeout (ms)
                5000   // Write timeout (ms) - Increase this!
        );
        executorService = Executors.newSingleThreadExecutor();

        if (serialPort.openPort()) {
            sendTask = sendToSTM32executor.scheduleAtFixedRate(
                    this::sendToSTM32,  // Method reference
                    0, sendingFixedRate, TimeUnit.MILLISECONDS  // 500Hz
            );
            showAlert(Alert.AlertType.INFORMATION, "Success", "Connected Successfully!");
            executorService.submit(() -> {
                while (serialPort.isOpen()) {
                    if (serialPort.bytesAvailable() > 0) {
                        byte[] readBuffer = new byte[serialPort.bytesAvailable()];
                        int numRead = serialPort.readBytes(readBuffer, readBuffer.length);
                        String data = new String(readBuffer, 0, numRead);
                        messageBuffer.append(data);
                        int endIndex = messageBuffer.indexOf("\n");
                        if (endIndex != -1) {
                            String completeMessage = messageBuffer.substring(0, endIndex);
                            messageBuffer.delete(0, endIndex + 1);
                            if (isValidJson(completeMessage)) {
                                Platform.runLater(() -> updateLabelsFromData(completeMessage));
                            } else {
                                if (!completeMessage.isEmpty()) {
                                    StringBuilder temp = new StringBuilder(completeMessage);
                                    int start = completeMessage.indexOf("{");
                                    int end = completeMessage.indexOf("}", start + 1);
                                    if (start != -1) {
                                        if (end == -1) {
                                            temp.delete(start, temp.length());
                                        } else {
                                            temp.delete(start, end + 1);
                                        }
                                    }
                                    if (end == -1 && !temp.isEmpty()) {
                                        String completeMessage2 = temp.toString();
                                        System.out.println(completeMessage2);
                                    }
                                }
                            }
                        }
                    }
                }
            });
        } else {
            showAlert(Alert.AlertType.ERROR, "Error", "Failed to connect!");
        }
    }

    public void close() {
        if (serialPort != null && serialPort.isOpen()) {
            serialPort.closePort();
            executorService.shutdown();
            System.out.println("Port closed.");
        } else {
            System.out.println("Port is not open");
        }
    }
    private float prop_x;
    private float integ_x;
    private float deriv_x;
    private float prop_y;
    private float integ_y;
    private float deriv_y;
    public void updateLabelsFromData(String jsonData) {
        try {
            Gson gson = new Gson();
            MainPage.ballSystem = gson.fromJson(jsonData, Ball_System.class);
            //System.out.println(jsonData);
            Servo_X = ballSystem.Servo_X;
            Servo_Y = ballSystem.Servo_Y;
            prop_x = ballSystem.kp_x;
            integ_x = ballSystem.ki_x;
            deriv_x = ballSystem.kd_x;

            prop_y = ballSystem.kp_y;
            integ_y = ballSystem.ki_y;
            deriv_y = ballSystem.kd_y;
            if (trajectoryMode != 0 && !draw_RadioButton.isSelected()) {
                desiredX = ballSystem.desired_X;
                desiredY = ballSystem.desired_Y;
                setDesiredPosition(desiredX, desiredY);
            }
            //System.out.println(jsonData);

        } catch (Exception e) {
            System.out.println(jsonData);
            //e.printStackTrace();
        }
    }

    private boolean isValidJson(String json) {
        try {
            JsonParser.parseString(json);
            return true;
        } catch (JsonSyntaxException e) {
            return false;
        }
    }

    public void setDesiredPosition(double x, double y) {
        desiredX = (float) clamp(x, MIN_VALUE, MAX_VALUE);
        desiredY = (float) clamp(y, MIN_VALUE, MAX_VALUE);
//        if(kf_path==null){
//            kf_path=new KalmanFilter(x,y);
//        }
//        kf_path.enta_shayf_eh();
//        kf_path.khod_faltar(x, y);
        pathPoints.add(new double[]{x, y});
        if(pathPoints.size()>5000){
            pathPoints.removeFirst();
        }
        updateLabels();
        drawPlate();
    }
    public void updateCurrentPosition(double x, double y) {
        currentX = (float) clamp(x, MIN_VALUE, MAX_VALUE);
        currentY = (float) clamp(y, MIN_VALUE, MAX_VALUE);

        // Add point to trajectory
        trajectoryPoints.add(new double[]{currentX, currentY});
        if(pathToggleButton.isSelected()){
            if (trajectoryPoints.size() > 50) {
                trajectoryPoints.removeFirst();
            }
        }
        else{
            if (trajectoryPoints.size() > 200) {
                trajectoryPoints.removeFirst();
            }
        }

        updateLabels();
        drawPlate();
    }

    // Trajectory management
    public void clearTrajectory() {
        trajectoryPoints.clear();
        pathPoints.clear();
        stopPathFollowing();
        pathToggleButton.setSelected(false);
        drawPlate();
    }
    private void updateLabels() {
        desiredPositionLabel.setText(
                String.format("Desired Position: (%.1f, %.1f)cm", desiredX, desiredY)
        );
        currentPositionLabel.setText(
                String.format("Current Position: (%.1f, %.1f)cm", currentX, currentY)
        );
        double v_total = Math.sqrt(desired_velocity_X * desired_velocity_X + desired_velocity_Y * desired_velocity_Y);
        desiredVelocityLabel.setText(
                String.format("Desired Velocity: (%.1f, %.1f) = %.1fcm/s", desired_velocity_X, desired_velocity_Y, v_total)
        );
    }
    @FXML
    private ToggleButton laserToggleButton;
    private int trajectoryIndex = 0;
    private double trajectoryProgress = 0.0;
    @FXML
    private ToggleButton pathToggleButton;
    @FXML
    private void handlepathToggle() {
        if (pathToggleButton.isSelected()) {
            startPathFollowing();
        } else {
            stopPathFollowing();
        }
    }
    private AnimationTimer pathTimer;
    private void stopPathFollowing() {
        if (pathTimer != null) {
            pathTimer.stop();
            pathTimer = null;
        }
    }
    private void startPathFollowing() {
        trajectoryIndex = 0;
        trajectoryProgress = 0.0;
        pathTimer = new AnimationTimer() {
            private long lastUpdate = 0;
            @Override
            public void handle(long now) {
                if (!pathToggleButton.isSelected()) {
                    stopPathFollowing();
                    return;
                }

                if (lastUpdate == 0) {
                    lastUpdate = now;
                    return;
                }

                double deltaTimeSeconds = (now - lastUpdate) / 1_000_000_000.0;
                lastUpdate = now;
                updateDesiredPosition(deltaTimeSeconds);
                if (trajectoryIndex >= pathPoints.size() - 1) {
                    trajectoryIndex = 0;
                    trajectoryProgress = 0.0;
                }
            }
        };
        pathTimer.start();
    }

    private void updateDesiredPosition(double deltaTimeSeconds) {
        if (pathPoints.size() < 2) return;
        double basespeed=base_angularVelocity*Math.PI*5/180;
        double speed = (basespeed+ deltaTimeSeconds)*trajectorySpeedMultiplier;
        while (trajectoryIndex < pathPoints.size() - 1 && speed > 0) {
            double[] current = pathPoints.get(trajectoryIndex);
            double[] next = pathPoints.get(trajectoryIndex + 1);

            double dx = next[0] - current[0];
            double dy = next[1] - current[1];
            double segmentLength = Math.sqrt(dx * dx + dy * dy);

            if (trajectoryProgress + speed < segmentLength) {
                trajectoryProgress += speed;
                double t = trajectoryProgress / segmentLength;
                desiredX = (float)(current[0] + t * dx);
                desiredY = (float)(current[1] + t * dy);
                updateLabels();
                drawPlate();
                return;
            } else {
                speed -= (segmentLength - trajectoryProgress);
                trajectoryIndex++;
                trajectoryProgress = 0.0;
                updateLabels();
                drawPlate();
            }
        }
        if (trajectoryIndex >= pathPoints.size() - 1) {
            double[] last = pathPoints.get(pathPoints.size() - 1);
            desiredX = (float) last[0];
            desiredY = (float) last[1];
            updateLabels();
            drawPlate();
        }
    }
    public static void rotateList(List<double[]> list, int n) {
        // Normalize n to range [0, list.size())
        int size = list.size();
        if (size == 0) return;
        n = n % size;
        if (n < 0) n += size; // convert left shift to equivalent right shift
        Collections.rotate(list, n); // rotates right by n
    }
    @FXML
    private Button smoothButton;

    @FXML
    private void smoothPathPoints() {

        // Need at least 3 points to create a meaningful closed loop
        if (pathPoints.size() < 3) return;
        if (!pathPoints.isEmpty() && !isPathClosed(pathPoints)) {
            pathPoints.add(new double[]{pathPoints.get(0)[0], pathPoints.get(0)[1]});
        }
        rotateList(pathPoints,pathPoints.size()/3);

        PathSmoother.simplifyAndSmoothPath(pathPoints,50, 3);
        rotateList(pathPoints,-pathPoints.size()/2);
        if (!pathPoints.isEmpty() && !isPathClosed(pathPoints)) {
            pathPoints.add(new double[]{pathPoints.get(0)[0], pathPoints.get(0)[1]});
        }
        drawPlate();
    }


    private boolean isPathClosed(List<double[]> points) {
        if (points.size() < 2) return false;
        double[] first = points.get(0);
        double[] last = points.get(points.size() - 1);
        double distance = Math.sqrt(Math.pow(first[0] - last[0], 2) + Math.pow(first[1] - last[1], 2));
        return distance < 0.05;
    }



    private void drawPlate() {
        GraphicsContext gc = plateCanvas.getGraphicsContext2D();
        double width = plateCanvas.getWidth();
        double height = plateCanvas.getHeight();
        double centerX = width / 2;
        double centerY = height / 2;

        gc.clearRect(0, 0, width, height);

        gc.setStroke(GRID_COLOR);
        gc.setLineWidth(1);

        for (int x = (int) MIN_VALUE; x <= MAX_VALUE; x += 1) {
            double screenX = logicalToScreenX(x);
            gc.strokeLine(screenX, 0, screenX, height);
            // Draw grid labels
            if (x % 2 == 0) {
                gc.fillText(Integer.toString(x), screenX, centerY + 15);
            }
        }

        for (int y = (int) MIN_VALUE; y <= MAX_VALUE; y += 1) {
            double screenY = logicalToScreenY(y);
            gc.strokeLine(0, screenY, width, screenY);
            if (y % 2 == 0) {
                gc.fillText(Integer.toString(y), centerX + 10, screenY);
            }

        }

        gc.setStroke(ORIGIN_COLOR);
        gc.setLineWidth(2);
        // X axis
        gc.strokeLine(0, centerY, width, centerY);
        // Y axis
        gc.strokeLine(centerX, 0, centerX, height);
        if (!trajectoryPoints.isEmpty()) {
            gc.setStroke(TRAJECTORY_COLOR);
            double TRAJECTORY_LINE_WIDTH = 1.5;
            gc.setLineWidth(TRAJECTORY_LINE_WIDTH);
            gc.setStroke(Color.LIMEGREEN);
            double[] firstPoint = logicalToScreen(trajectoryPoints.getFirst()[0], trajectoryPoints.getFirst()[1]);
            gc.beginPath();
            gc.moveTo(firstPoint[0], firstPoint[1]);
            for (int i = 1; i < trajectoryPoints.size(); i++) {
                double[] point = logicalToScreen(trajectoryPoints.get(i)[0], trajectoryPoints.get(i)[1]);
                gc.lineTo(point[0], point[1]);
                if (i > trajectoryPoints.size() - 50) { 
                    double opacity = 0.2 + 0.8 * (i - (trajectoryPoints.size() - 50)) / 50.0;
                    gc.setStroke(Color.rgb(50, 205, 50, opacity)); 
                }
            }
            gc.stroke();
            if (trajectoryPoints.size() > 5) {
                gc.setStroke(Color.LIMEGREEN);
                gc.setLineWidth(TRAJECTORY_LINE_WIDTH * 1.5);
                int recentPoints = Math.min(10, trajectoryPoints.size());
                for (int i = trajectoryPoints.size() - recentPoints; i < trajectoryPoints.size() - 1; i++) {
                    double[] p1 = logicalToScreen(trajectoryPoints.get(i)[0], trajectoryPoints.get(i)[1]);
                    double[] p2 = logicalToScreen(trajectoryPoints.get(i + 1)[0], trajectoryPoints.get(i + 1)[1]);
                    gc.strokeLine(p1[0], p1[1], p2[0], p2[1]);
                }
            }
        }
        if (!pathPoints.isEmpty() && draw_RadioButton.isSelected()) {

            gc.setStroke(TRAJECTORY_COLOR);
            double TRAJECTORY_LINE_WIDTH = 1.5;
            gc.setLineWidth(TRAJECTORY_LINE_WIDTH);
            gc.setStroke(Color.RED); 
            double[] firstPoint = logicalToScreen(pathPoints.getFirst()[0], pathPoints.getFirst()[1]);
            gc.beginPath();
            gc.moveTo(firstPoint[0], firstPoint[1]);
            for (int i = 1; i < pathPoints.size(); i++) {
                double[] point = logicalToScreen(pathPoints.get(i)[0], pathPoints.get(i)[1]);
                gc.lineTo(point[0], point[1]);

//                // Fade out older points
//                if (i > pathPoints.size() - 50) { // Last 50 points
//                    double opacity = 0.2 + 0.8 * (i - (pathPoints.size() - 50)) / 50.0;
//                    gc.setStroke(Color.rgb(50, 205, 50, opacity)); // LimeGreen with opacity
//                }
            }
            gc.stroke();
//
//            // Draw a glowing effect for the most recent points
//            if (pathPoints.size() > 5) {
//                gc.setStroke(Color.RED);
//                gc.setLineWidth(TRAJECTORY_LINE_WIDTH * 1.5);
//                int recentPoints = Math.min(10, pathPoints.size());
//                for (int i = pathPoints.size() - recentPoints; i < pathPoints.size() - 1; i++) {
//                    double[] p1 = logicalToScreen(pathPoints.get(i)[0], pathPoints.get(i)[1]);
//                    double[] p2 = logicalToScreen(pathPoints.get(i + 1)[0], pathPoints.get(i + 1)[1]);
//                    gc.strokeLine(p1[0], p1[1], p2[0], p2[1]);
//                }
//            }
        }


        // Draw desired position tolerance circle
        double[] desiredScreen = logicalToScreen(desiredX, desiredY);
        gc.setFill(TOLERANCE_COLOR);
        double toleranceRadius = logicalToScreenSize(TOLERANCE);
        gc.fillOval(
                desiredScreen[0] - toleranceRadius,
                desiredScreen[1] - toleranceRadius,
                toleranceRadius * 2,
                toleranceRadius * 2
        );

        // Draw desired position
        gc.setFill(DESIRED_COLOR);
        double BALL_RADIUS = 0.675;
        double desiredBallRadius = logicalToScreenSize(BALL_RADIUS / 2);
        gc.fillOval(
                desiredScreen[0] - desiredBallRadius,
                desiredScreen[1] - desiredBallRadius,
                desiredBallRadius * 2,
                desiredBallRadius * 2
        );

        // Draw crosshair for desired position
        gc.setStroke(DESIRED_COLOR);
        gc.setLineWidth(2);
        double crossSize = logicalToScreenSize(BALL_RADIUS);
        gc.strokeLine(desiredScreen[0] - crossSize, desiredScreen[1], desiredScreen[0] + crossSize, desiredScreen[1]);
        gc.strokeLine(desiredScreen[0], desiredScreen[1] - crossSize, desiredScreen[0], desiredScreen[1] + crossSize);

        // Draw current position with glow effect
        double[] currentScreen = logicalToScreen(currentX, currentY);

//        // Glow effect
//        gc.setFill(Color.rgb(0, 0, 255, 0.3)); // Semi-transparent blue
//        double glowRadius = logicalToScreenSize(BALL_RADIUS * 1.5);
//        gc.fillOval(
//                currentScreen[0] - glowRadius,
//                currentScreen[1] - glowRadius,
//                glowRadius * 2,
//                glowRadius * 2
//        );

        // Main ball
        gc.setFill(CURRENT_COLOR);
        double currentBallRadius = logicalToScreenSize(BALL_RADIUS / 2);
        gc.fillOval(
                currentScreen[0] - currentBallRadius,
                currentScreen[1] - currentBallRadius,
                currentBallRadius * 2,
                currentBallRadius * 2
        );

        // Highlight for current position
        gc.setStroke(Color.WHITE);
        gc.setLineWidth(1);
        gc.strokeOval(
                currentScreen[0] - currentBallRadius,
                currentScreen[1] - currentBallRadius,
                currentBallRadius * 2,
                currentBallRadius * 2
        );
        gc.setStroke(Color.BLUE);
        gc.setLineWidth(2);
        // --- Draw PID Component Arrows ---
        double arrowScale = 10.0;
        gc.setLineWidth(3);

// Proportional (light blue)
        gc.setStroke(Color.BLUE);
        drawArrow(gc,
                currentScreen[0], currentScreen[1],
                currentScreen[0] + prop_x * arrowScale,
                currentScreen[1] - prop_y * arrowScale
        );

// Integral (yellow-green)
        gc.setStroke(Color.ORANGE);
        drawArrow(gc,
                currentScreen[0], currentScreen[1],
                currentScreen[0] + integ_x * arrowScale,
                currentScreen[1] - integ_y * arrowScale
        );

// Derivative (magenta)
        gc.setStroke(Color.MAGENTA);
        drawArrow(gc,
                currentScreen[0], currentScreen[1],
                currentScreen[0] + deriv_x * arrowScale,
                currentScreen[1] - deriv_y * arrowScale
        );


    }
    private void drawDesiredPosition(){
//        GraphicsContext gc = plateCanvas.getGraphicsContext2D();
//
//        // Draw desired position tolerance circle
//        double[] desiredScreen = logicalToScreen(desiredX, desiredY);
//        gc.setFill(TOLERANCE_COLOR);
//        double toleranceRadius = logicalToScreenSize(TOLERANCE);
//        gc.fillOval(
//                desiredScreen[0] - toleranceRadius,
//                desiredScreen[1] - toleranceRadius,
//                toleranceRadius * 2,
//                toleranceRadius * 2
//        );
//
//        // Draw desired position
//        gc.setFill(DESIRED_COLOR);
//        double BALL_RADIUS = 0.675;
//        double desiredBallRadius = logicalToScreenSize(BALL_RADIUS / 2);
//        gc.fillOval(
//                desiredScreen[0] - desiredBallRadius,
//                desiredScreen[1] - desiredBallRadius,
//                desiredBallRadius * 2,
//                desiredBallRadius * 2
//        );
//
//        // Draw crosshair for desired position
//        gc.setStroke(DESIRED_COLOR);
//        gc.setLineWidth(2);
//        double crossSize = logicalToScreenSize(BALL_RADIUS);
//        gc.strokeLine(desiredScreen[0] - crossSize, desiredScreen[1], desiredScreen[0] + crossSize, desiredScreen[1]);
//        gc.strokeLine(desiredScreen[0], desiredScreen[1] - crossSize, desiredScreen[0], desiredScreen[1] + crossSize);
    }
    private void drawArrow(GraphicsContext gc, double x1, double y1, double x2, double y2) {
        gc.strokeLine(x1, y1, x2, y2);


        double angle = Math.atan2(y2 - y1, x2 - x1);
        double arrowHeadLength = 10;
        double arrowHeadAngle = Math.toRadians(20);

        double xArrow1 = x2 - arrowHeadLength * Math.cos(angle - arrowHeadAngle);
        double yArrow1 = y2 - arrowHeadLength * Math.sin(angle - arrowHeadAngle); 

        double xArrow2 = x2 - arrowHeadLength * Math.cos(angle + arrowHeadAngle);
        double yArrow2 = y2 - arrowHeadLength * Math.sin(angle + arrowHeadAngle);  

        gc.strokeLine(x2, y2, xArrow1, yArrow1);
        gc.strokeLine(x2, y2, xArrow2, yArrow2);
    }


    private double logicalToScreenX(double logicalX) {
        return plateCanvas.getWidth() / 2 + (logicalX * plateCanvas.getWidth() / (MAX_VALUE - MIN_VALUE));
    }

    private double logicalToScreenY(double logicalY) {
        return plateCanvas.getHeight() / 2 - (logicalY * plateCanvas.getHeight() / (MAX_VALUE - MIN_VALUE));
    }

    private double[] logicalToScreen(double logicalX, double logicalY) {
        return new double[]{logicalToScreenX(logicalX), logicalToScreenY(logicalY)};
    }

    private double[] screenToLogical(double screenX, double screenY) {
        double logicalX = (screenX - plateCanvas.getWidth() / 2) * (MAX_VALUE - MIN_VALUE) / plateCanvas.getWidth();
        double logicalY = (plateCanvas.getHeight() / 2 - screenY) * (MAX_VALUE - MIN_VALUE) / plateCanvas.getHeight();
        return new double[]{logicalX, logicalY};
    }

    private double logicalToScreenSize(double logicalSize) {
        return logicalSize * plateCanvas.getWidth() / (MAX_VALUE - MIN_VALUE);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    public enum OperationMode {
        IDLE,
        MANUAL,
        CALIBRATE,
        AUTOMATIC
    }

    public enum CalibrationMode {
        IDLE,
        RESET,
        SAVE
    }

    public enum ControlMode {
        PID,
        PV,
        CUSTOM1,
        CUSTOM2
    }
}

class Ball_System {
    float Servo_X;
    float Servo_Y;
    float desired_X;
    float desired_Y;
    float kp_x;
    float ki_x;
    float kd_x;
    float kp_y;
    float ki_y;
    float kd_y;
}
