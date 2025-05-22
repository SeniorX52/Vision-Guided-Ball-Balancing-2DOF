package com.android.ball_balancing_system;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollBar;
import javafx.scene.effect.DropShadow;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.stage.FileChooser;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.ResourceBundle;

public class Graphs implements Initializable {
    private final List<DataPoint> dataLog = new ArrayList<>();
    private boolean isLogging = false;
    private static final SimpleDateFormat TIMESTAMP_FORMAT =
            new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");

    // Data point class for system identification
    private static class DataPoint {
        final double timestamp;
        final float desiredX, desiredY;
        final float currentX, currentY;
        final float servoX, servoY;

        DataPoint(double timestamp, float desiredX, float desiredY,
                  float currentX, float currentY,
                  float servoX, float servoY) {
            this.timestamp = timestamp;
            this.desiredX = desiredX;
            this.desiredY = desiredY;
            this.currentX = currentX;
            this.currentY = currentY;
            this.servoX = servoX;
            this.servoY = servoY;
        }
    }
    private static final double BUFFER = 10.0;
    public static boolean isTimerStarted;
    private static float current_X_Value;
    private static float desired_X_Value;
    private static float servo_X_Value;
    private static float servo_Y_Value;
    private static float current_Y_Value;
    private static float desired_Y_Value;
    @FXML
    private LineChart<Number, Number> X_LineChart;
    private XYChart.Series<Number, Number> desired_X;
    private XYChart.Series<Number, Number> current_X;
    private XYChart.Series<Number, Number> Servo_X;
    @FXML
    private LineChart<Number, Number> Y_LineChart;
    private XYChart.Series<Number, Number> desired_Y;
    private XYChart.Series<Number, Number> current_Y;
    private XYChart.Series<Number, Number> Servo_Y;
    @FXML
    private ScrollBar X_ScrollBar;
    @FXML
    private ScrollBar Y_ScrollBar;
    @FXML
    private ImageView pause;
    @FXML
    private ImageView run;
    @FXML
    private ImageView reset;
    private double max_X = Double.NEGATIVE_INFINITY;
    private double min_X = Double.POSITIVE_INFINITY;
    private double max_Y = Double.NEGATIVE_INFINITY;
    private double min_Y = Double.POSITIVE_INFINITY;
    private double elapsedTime;
    @FXML
    private Button auto_X_Button;
    @FXML
    private Button auto_Y_Button;
    private long startTime = System.currentTimeMillis();
    private AnimationTimer timer;

    public static void addHoverEffect(ImageView imageView) {
        DropShadow shadow = new DropShadow();
        shadow.setColor(Color.GRAY);
        shadow.setRadius(20);
        imageView.addEventHandler(MouseEvent.MOUSE_ENTERED, e -> imageView.setEffect(shadow));
        imageView.addEventHandler(MouseEvent.MOUSE_EXITED, e -> imageView.setEffect(null));
    }

    @Override
    public void initialize(URL url, ResourceBundle resourceBundle) {
        addHoverEffect(pause);
        addHoverEffect(run);
        addHoverEffect(reset);
        X_ScrollBar.valueProperty().addListener((obs, oldVal, newVal) -> {
            NumberAxis xAxis = (NumberAxis) X_LineChart.getXAxis();
            double lowerBound = newVal.intValue();
            double upperBound = lowerBound + 10;
            xAxis.setLowerBound(lowerBound);
            xAxis.setUpperBound(upperBound);
            if (!isTimerStarted) {
                auto_X();
            }

        });
        Y_ScrollBar.valueProperty().addListener((obs, oldVal, newVal) -> {
            NumberAxis xAxis = (NumberAxis) Y_LineChart.getXAxis();
            double lowerBound = newVal.intValue();
            double upperBound = lowerBound + 10;
            xAxis.setLowerBound(lowerBound);
            xAxis.setUpperBound(upperBound);
            if (!isTimerStarted) {
                auto_Y();
            }

        });
        desired_X = new XYChart.Series<>();
        desired_X.setName("Desired X");
        current_X = new XYChart.Series<>();
        current_X.setName("Current X");
        Servo_X = new XYChart.Series<>();
        Servo_X.setName("Servo X");

        desired_Y = new XYChart.Series<>();
        desired_Y.setName("Desired Y");
        current_Y = new XYChart.Series<>();
        current_Y.setName("Current Y");
        Servo_Y = new XYChart.Series<>();
        Servo_Y.setName("Servo Y");

        X_LineChart.getData().addAll(desired_X, current_X, Servo_X);
        Y_LineChart.getData().addAll(desired_Y, current_Y, Servo_Y);
        startGraphing();
    }
    @FXML
    public void toggleLogging() {
        isLogging = !isLogging;
        if (isLogging) {
            dataLog.clear(); // Clear previous data when starting new log
            System.out.println("Data logging started");
        } else {
            System.out.println("Data logging stopped. Collected " + dataLog.size() + " points.");
        }
    }

    /**
     * Exports collected data to CSV for system identification
     */
    @FXML
    public void exportSystemIdData() {
        if (dataLog.isEmpty()) {
            System.out.println("No data to export");
            return;
        }

        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Save System Identification Data");
        fileChooser.setInitialFileName("Ball_Balancing_System_ID" +".csv");
        fileChooser.getExtensionFilters().add(
                new FileChooser.ExtensionFilter("CSV Files", "*.csv"));

        File file = fileChooser.showSaveDialog(null);
        if (file != null) {
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
                // Write header
                writer.write("timestamp,desired_x,desired_y,current_x,current_y,servo_x,servo_y\n");

                // Write data
                for (DataPoint point : dataLog) {
                    writer.write(String.format("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                            point.timestamp,
                            point.desiredX,
                            point.desiredY,
                            point.currentX,
                            point.currentY,
                            point.servoX,
                            point.servoY));
                }
                System.out.println("Data exported successfully to: " + file.getAbsolutePath());
            } catch (IOException e) {
                System.err.println("Error exporting data: " + e.getMessage());
            }
        }
    }

    private void styleSeries(XYChart.Series<Number, Number> series, String color) {
        Node line = series.getNode().lookup(".chart-series-line");
        if (line != null) {
            line.setStyle("-fx-stroke: " + color + "; -fx-stroke-width: 2px;");
        }
        for (Node node : X_LineChart.lookupAll(".chart-legend-item")) {
            if (node instanceof Label label) {
                if (label.getText().equals(series.getName())) {
                    Node symbol = label.getGraphic();
                    if (symbol != null) {
                        symbol.setStyle("-fx-background-color: " + color + ", white;");
                    }
                }
            }
        }
        for (Node node : Y_LineChart.lookupAll(".chart-legend-item")) {
            if (node instanceof Label label) {
                if (label.getText().equals(series.getName())) {
                    Node symbol = label.getGraphic();
                    if (symbol != null) {
                        symbol.setStyle("-fx-background-color: " + color + ", white;");
                    }
                }
            }
        }
        for (XYChart.Data<Number, Number> data : series.getData()) {
            data.getNode().setVisible(false);
        }
    }

    private void updateScrollBarRange(ScrollBar scrollBar, int newTime) {
        scrollBar.setMax(newTime);
        if (newTime > scrollBar.getVisibleAmount()) {
            scrollBar.setVisibleAmount(scrollBar.getVisibleAmount());
        } else {
            scrollBar.setVisibleAmount(newTime);
        }
        scrollBar.setMax(newTime);
        if (newTime > 10)
            scrollBar.setValue(newTime);
    }
    private long elapsedTime2;
    private long lastUpdateTime = 0;
    private void startGraphing() {
        isTimerStarted = true;
        timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                // Calculate elapsed time in seconds
                elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;
                long currentTime = System.currentTimeMillis();
                elapsedTime2 = currentTime - startTime;
                if (true) {
                    lastUpdateTime = currentTime;
                    current_X_Value = MainPage.currentX;
                    desired_X_Value = MainPage.desiredX;
                    current_Y_Value = MainPage.currentY;
                    desired_Y_Value = MainPage.desiredY;
                    servo_X_Value=MainPage.Servo_X;
                    servo_Y_Value=MainPage.Servo_Y;
                    desired_X.getData().add(new XYChart.Data<>(elapsedTime, desired_X_Value));
                    current_X.getData().add(new XYChart.Data<>(elapsedTime, current_X_Value));
                    desired_Y.getData().add(new XYChart.Data<>(elapsedTime, desired_Y_Value));
                    current_Y.getData().add(new XYChart.Data<>(elapsedTime, current_Y_Value));
                    Servo_X.getData().add(new XYChart.Data<>(elapsedTime, servo_X_Value));
                    Servo_Y.getData().add(new XYChart.Data<>(elapsedTime, servo_Y_Value));

                    if (isLogging) {
                        dataLog.add(new DataPoint(
                                elapsedTime,
                                desired_X_Value,
                                desired_Y_Value,
                                current_X_Value,
                                current_Y_Value,
                                servo_X_Value,
                                servo_Y_Value
                        ));
                    }
                    updateScrollBarRange(X_ScrollBar, (int) elapsedTime);
                    updateScrollBarRange(Y_ScrollBar, (int) elapsedTime);

                    max_X = Math.max(Math.max(Math.max(desired_X_Value, current_X_Value), servo_X_Value), max_X);
                    min_X = Math.min(Math.min(Math.min(desired_X_Value, current_X_Value), servo_X_Value), min_X);
                    max_Y = Math.max(Math.max(Math.max(desired_Y_Value, current_Y_Value), servo_Y_Value), max_Y);
                    min_Y = Math.min(Math.min(Math.min(desired_Y_Value, current_Y_Value), servo_Y_Value), min_Y);

                    NumberAxis yAxis = (NumberAxis) X_LineChart.getYAxis();
                    yAxis.setUpperBound(max_X + max_X * (10.0 / 100.0));
                    yAxis.setLowerBound(min_X + min_X * (10.0 / 100.0));
                    NumberAxis yAxis2 = (NumberAxis) Y_LineChart.getYAxis();
                    yAxis2.setUpperBound(max_Y + max_Y * (10.0 / 100.0));
                    yAxis2.setLowerBound(min_Y + min_Y * (10.0 / 100.0));


                    if (auto_X_Button.isPressed()) {
                        auto_X();
                    }
                    if (auto_Y_Button.isPressed()) {
                        auto_Y();
                    }

                    if (elapsedTime > 10) {
                        NumberAxis xAxis = (NumberAxis) X_LineChart.getXAxis();
                        xAxis.setLowerBound(elapsedTime - 10);
                        xAxis.setUpperBound(elapsedTime);

                        NumberAxis xAxis2 = (NumberAxis) Y_LineChart.getXAxis();
                        xAxis2.setLowerBound(elapsedTime - 10);
                        xAxis2.setUpperBound(elapsedTime);

                    }
                }
                if (desired_X.getData().size() > 5000) {
                    desired_X.getData().remove(0, 1000);
                    current_X.getData().remove(0, 1000);
                    desired_Y.getData().remove(0, 1000);
                    current_Y.getData().remove(0, 1000);
                    Servo_X.getData().remove(0, 1000);
                    Servo_Y.getData().remove(0, 1000);
                }
                Platform.runLater(() -> {
                    styleSeries(current_X, "blue");
                    styleSeries(desired_X, "red");
                    styleSeries(Servo_X, "lime");
                    styleSeries(current_Y, "blue");
                    styleSeries(desired_Y, "red");
                    styleSeries(Servo_Y, "lime");
                });
            }
        };
        timer.start();
    }

    public void auto_X() {
        NumberAxis xAxis = (NumberAxis) X_LineChart.getXAxis();
        NumberAxis yAxis = (NumberAxis) X_LineChart.getYAxis();
        double lowerXBound = xAxis.getLowerBound();
        double upperXBound = xAxis.getUpperBound();

        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (XYChart.Series<Number, Number> series : X_LineChart.getData()) {
            for (XYChart.Data<Number, Number> data : series.getData()) {
                double xValue = data.getXValue().doubleValue();
                double yValue = data.getYValue().doubleValue();

                if (xValue >= lowerXBound && xValue <= upperXBound) {
                    if (yValue > max) {
                        max = yValue;
                    }
                    if (yValue < min) {
                        min = yValue;
                    }
                }
            }
        }
        max_X = max;
        min_X = min;
        double range = max_X - min_X;
        if (range <= 0) {
            max_X += BUFFER;
            min_X -= BUFFER;
        } else {
            max_X *= 1.1;
            min_X *= 1.1;
        }
        yAxis.setUpperBound(max_X);
        yAxis.setLowerBound(min_X);
    }

    public void auto_Y() {
        NumberAxis xAxis = (NumberAxis) Y_LineChart.getXAxis();
        NumberAxis yAxis = (NumberAxis) Y_LineChart.getYAxis();
        double lowerXBound = xAxis.getLowerBound();
        double upperXBound = xAxis.getUpperBound();

        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (XYChart.Series<Number, Number> series : Y_LineChart.getData()) {
            for (XYChart.Data<Number, Number> data : series.getData()) {
                double xValue = data.getXValue().doubleValue();
                double yValue = data.getYValue().doubleValue();

                if (xValue >= lowerXBound && xValue <= upperXBound) {
                    if (yValue > max) {
                        max = yValue;
                    }
                    if (yValue < min) {
                        min = yValue;
                    }
                }
            }
        }
        max_Y = max;
        min_Y = min;
        double range = max_Y - min_Y;
        if (range <= 0) {
            max_Y += BUFFER;
            min_Y -= BUFFER;
        } else {
            max_Y *= 1.1;
            min_Y *= 1.1;
        }
        yAxis.setUpperBound(max_Y);
        yAxis.setLowerBound(min_Y);
    }

    public void reset() {
        isTimerStarted = false;
        timer.stop();
        startTime = System.currentTimeMillis();
        updateScrollBarRange(X_ScrollBar, 0);
        updateScrollBarRange(Y_ScrollBar, 0);
        NumberAxis xAxis = (NumberAxis) X_LineChart.getXAxis();
        xAxis.setLowerBound(0);
        xAxis.setUpperBound(10);
        NumberAxis xAxis2 = (NumberAxis) Y_LineChart.getXAxis();
        xAxis2.setLowerBound(0);
        xAxis2.setUpperBound(10);
        desired_X.getData().clear();
        current_X.getData().clear();
        desired_Y.getData().clear();
        current_Y.getData().clear();
        Servo_X.getData().clear();
        Servo_Y.getData().clear();
    }

    @FXML
    public void pause() {
        if (isTimerStarted) {
            timer.stop();
            isTimerStarted = false;
            Servo_X.getData().add(new XYChart.Data<>(elapsedTime, 0));
            Servo_Y.getData().add(new XYChart.Data<>(elapsedTime, 0));
            desired_X.getData().add(new XYChart.Data<>(elapsedTime, 0));
            current_X.getData().add(new XYChart.Data<>(elapsedTime, 0));
            desired_Y.getData().add(new XYChart.Data<>(elapsedTime, 0));
            current_Y.getData().add(new XYChart.Data<>(elapsedTime, 0));
        }
    }

    @FXML
    public void run() {
        if (!isTimerStarted) {

            elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;
            desired_X.getData().add(new XYChart.Data<>(elapsedTime, 0));
            current_X.getData().add(new XYChart.Data<>(elapsedTime, 0));
            Servo_X.getData().add(new XYChart.Data<>(elapsedTime, 0));
            Servo_Y.getData().add(new XYChart.Data<>(elapsedTime, 0));
            desired_Y.getData().add(new XYChart.Data<>(elapsedTime, 0));
            current_Y.getData().add(new XYChart.Data<>(elapsedTime, 0));
            timer.start();
            isTimerStarted = true;
        }
    }
}
