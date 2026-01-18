package com.drones.ui;

import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import javafx.animation.AnimationTimer;
import javafx.stage.FileChooser;

import com.drones.config.SimulationParams;
import com.drones.control.SimulationEngine;
import com.drones.metrics.ExportUtils;
import com.drones.model.*;
import java.io.File;
import java.util.List;
import java.util.Map;

public class MainApp extends Application {
    
    private SimulationEngine engine;
    private Canvas canvas;
    private AnimationTimer animationTimer;
    private TextArea logArea;
    private Label metricsLabel;
    private Button startButton, pauseButton, stopButton, resetButton, exportButton;
    private Slider speedSlider;
    private double speedFactor = 1.0;
    private boolean isPaused = false;
    private int lastEventCount = 0;
    
    @Override
    public void start(Stage primaryStage) {
        engine = new SimulationEngine();
        
        // Root layout
        BorderPane root = new BorderPane();
        
        // Center: Canvas for simulation
        canvas = new Canvas(
            SimulationParams.GRID_WIDTH * SimulationParams.CELL_SIZE_PX,
            SimulationParams.GRID_HEIGHT * SimulationParams.CELL_SIZE_PX
        );
        root.setCenter(canvas);
        
        // Right: Control panel
        VBox controlPanel = createControlPanel();
        root.setRight(controlPanel);
        
        // Bottom: Log area
        logArea = new TextArea();
        logArea.setPrefHeight(120);
        logArea.setEditable(false);
        logArea.setWrapText(true);
        root.setBottom(logArea);
        
        // Scene
        Scene scene = new Scene(root, 1000, 850);
        primaryStage.setTitle("Autonomous Drone Swarm Simulator - IASC 2025");
        primaryStage.setScene(scene);
        primaryStage.show();
        
        // Start animation loop
        startAnimationLoop();
        
        engine.logEvent("Simulation initialisée. Appuyez sur Start pour commencer.");
    }
    
    private VBox createControlPanel() {
        VBox panel = new VBox(10);
        panel.setPadding(new Insets(10));
        panel.setStyle("-fx-border-color: #ccc; -fx-border-width: 0 0 0 1;");
        panel.setPrefWidth(220);
        
        // Title
        Label title = new Label("Contrôle Simulation");
        title.setStyle("-fx-font-size: 14; -fx-font-weight: bold;");
        panel.getChildren().add(title);
        
        // Buttons
        startButton = new Button("▶ Start");
        pauseButton = new Button("⏸ Pause");
        stopButton = new Button("⏹ Stop");
        resetButton = new Button("🔄 Reset");
        exportButton = new Button("💾 Export CSV");
        
        startButton.setPrefWidth(200);
        pauseButton.setPrefWidth(200);
        stopButton.setPrefWidth(200);
        resetButton.setPrefWidth(200);
        exportButton.setPrefWidth(200);
        
        startButton.setOnAction(e -> {
            engine.start();
            isPaused = false;
            engine.logEvent("▶ Simulation démarrée.");
        });
        
        pauseButton.setOnAction(e -> {
            if (!isPaused) {
                engine.stop();
                isPaused = true;
                engine.logEvent("⏸ Simulation en pause.");
            } else {
                engine.start();
                isPaused = false;
                engine.logEvent("▶ Simulation reprise.");
            }
        });
        
        stopButton.setOnAction(e -> {
            engine.stop();
            isPaused = false;
            engine.logEvent("⏹ Simulation arrêtée.");
        });
        
        resetButton.setOnAction(e -> {
            engine.reset();
            logArea.clear();
            lastEventCount = 0;
            engine.logEvent("🔄 Simulation réinitialisée.");
        });
        
        exportButton.setOnAction(e -> exportResults());
        
        panel.getChildren().addAll(startButton, pauseButton, stopButton, resetButton, exportButton);
        
        // Separator
        Separator sep1 = new Separator();
        panel.getChildren().add(sep1);
        
        // Speed control
        Label speedLabel = new Label("Vitesse Simulation");
        speedLabel.setStyle("-fx-font-size: 11; -fx-font-weight: bold;");
        panel.getChildren().add(speedLabel);
        
        speedSlider = new Slider(0.5, 4.0, 1.0);
        speedSlider.setShowTickLabels(true);
        speedSlider.setShowTickMarks(true);
        speedSlider.setMajorTickUnit(1.0);
        speedSlider.setBlockIncrement(0.5);
        speedSlider.setPrefWidth(200);
        speedSlider.valueProperty().addListener((obs, oldVal, newVal) -> {
            speedFactor = newVal.doubleValue();
            engine.logEvent("Vitesse: " + String.format("%.1f", speedFactor) + "x");
        });
        panel.getChildren().add(speedSlider);
        
        // Separator
        Separator sep2 = new Separator();
        panel.getChildren().add(sep2);
        
        // Metrics
        Label metricsTitle = new Label("Métriques");
        metricsTitle.setStyle("-fx-font-size: 12; -fx-font-weight: bold;");
        panel.getChildren().add(metricsTitle);
        
        metricsLabel = new Label();
        metricsLabel.setStyle("-fx-font-size: 10;");
        metricsLabel.setWrapText(true);
        panel.getChildren().add(metricsLabel);
        
        // Separator
        Separator sep3 = new Separator();
        panel.getChildren().add(sep3);
        
        // Info
        Label infoTitle = new Label("Configuration");
        infoTitle.setStyle("-fx-font-size: 11; -fx-font-weight: bold;");
        panel.getChildren().add(infoTitle);
        
        Label infoText = new Label(
            "Grille: " + SimulationParams.GRID_WIDTH + "x" + SimulationParams.GRID_HEIGHT + "\n" +
            "Drones: " + SimulationParams.NUM_DRONES + "\n" +
            "Tick: " + SimulationParams.TICK_DURATION_MS + " ms\n" +
            "Autonomie: 30 min\n" +
            "Recharge: 10 min"
        );
        infoText.setStyle("-fx-font-size: 9;");
        panel.getChildren().add(infoText);
        
        return panel;
    }
    
    private void startAnimationLoop() {
        animationTimer = new AnimationTimer() {
            private long lastTick = 0;
            private static final long TICK_NANOS = SimulationParams.TICK_DURATION_MS * 1_000_000L;
            
            @Override
            public void handle(long now) {
                if (engine.isRunning()) {
                    long adjustedTick = (long) (TICK_NANOS / speedFactor);
                    if (now - lastTick >= adjustedTick) {
                        engine.tick();
                        lastTick = now;
                    }
                }
                
                render();
                updateMetrics();
                updateLog();
            }
        };
        animationTimer.start();
    }
    
    private void render() {
        GraphicsContext gc = canvas.getGraphicsContext2D();
        
        // Clear canvas
        gc.setFill(Color.WHITE);
        gc.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());
        
        // Draw grid lines
        gc.setStroke(Color.web("#e0e0e0"));
        gc.setLineWidth(0.5);
        for (int i = 0; i <= SimulationParams.GRID_WIDTH; i++) {
            gc.strokeLine(i * SimulationParams.CELL_SIZE_PX, 0,
                         i * SimulationParams.CELL_SIZE_PX, canvas.getHeight());
        }
        for (int i = 0; i <= SimulationParams.GRID_HEIGHT; i++) {
            gc.strokeLine(0, i * SimulationParams.CELL_SIZE_PX,
                         canvas.getWidth(), i * SimulationParams.CELL_SIZE_PX);
        }
        
        // Draw anomaly heatmap
        Environment env = engine.getEnvironment();
        double[][] anomalyGrid = env.getAnomalyIntensity();
        
        for (int y = 0; y < env.getHeight(); y++) {
            for (int x = 0; x < env.getWidth(); x++) {
                double intensity = anomalyGrid[y][x];
                if (intensity > 0.01) {
                    Color color = interpolateColor(intensity);
                    gc.setFill(color);
                    gc.fillRect(x * SimulationParams.CELL_SIZE_PX,
                               y * SimulationParams.CELL_SIZE_PX,
                               SimulationParams.CELL_SIZE_PX,
                               SimulationParams.CELL_SIZE_PX);
                }
            }
        }
        
        // Draw drone trajectories
        drawTrajectories(gc);
        
        // Draw base
        gc.setFill(Color.GREEN);
        gc.fillRect(0, 0, SimulationParams.CELL_SIZE_PX, SimulationParams.CELL_SIZE_PX);
        gc.setStroke(Color.DARKGREEN);
        gc.setLineWidth(2);
        gc.strokeRect(0, 0, SimulationParams.CELL_SIZE_PX, SimulationParams.CELL_SIZE_PX);
        gc.setFill(Color.BLACK);
        gc.setFont(new javafx.scene.text.Font(9));
        gc.fillText("BASE", 2, 11);
        
        // Draw drones
        for (Drone drone : engine.getDrones()) {
            drawDrone(gc, drone);
        }
    }
    
    private void drawTrajectories(GraphicsContext gc) {
        for (Map.Entry<Integer, List<double[]>> entry : engine.getDroneTrajectories().entrySet()) {
            List<double[]> trajectory = entry.getValue();
            
            if (trajectory.size() < 2) continue;
            
            gc.setStroke(Color.web("#cccccc", 0.3));
            gc.setLineWidth(1);
            
            for (int i = 1; i < trajectory.size(); i++) {
                double[] prev = trajectory.get(i - 1);
                double[] curr = trajectory.get(i);
                
                int x1 = (int) (prev[0] * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
                int y1 = (int) (prev[1] * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
                int x2 = (int) (curr[0] * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
                int y2 = (int) (curr[1] * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
                
                gc.strokeLine(x1, y1, x2, y2);
            }
        }
    }
    
    private void drawDrone(GraphicsContext gc, Drone drone) {
        int px = (int) (drone.getX() * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
        int py = (int) (drone.getY() * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
        int radius = 5;
        
        Color stateColor;
        switch (drone.getState()) {
            case ACTIVE:
                stateColor = Color.GREEN;
                break;
            case MEASURING:
                stateColor = Color.BLUE;
                break;
            case RETURNING:
                stateColor = Color.ORANGE;
                break;
            case CHARGING:
                stateColor = Color.RED;
                break;
            default:
                stateColor = Color.GRAY;
        }
        
        gc.setFill(stateColor);
        gc.fillOval(px - radius, py - radius, radius * 2, radius * 2);
        gc.setStroke(Color.BLACK);
        gc.setLineWidth(1);
        gc.strokeOval(px - radius, py - radius, radius * 2, radius * 2);
        
        gc.setFill(Color.BLACK);
        gc.setFont(new javafx.scene.text.Font(8));
        gc.fillText(String.valueOf(drone.getId()), px - 2, py + 3);
    }
    
    private Color interpolateColor(double intensity) {
        if (intensity < 0.5) {
            double t = intensity / 0.5;
            return Color.color(1.0, 1.0 * (1 - t * 0.5), 0);
        } else {
            double t = (intensity - 0.5) / 0.5;
            return Color.color(1.0, 0.5 * (1 - t), 0);
        }
    }
    
    private void updateMetrics() {
        SimulationEngine.SimulationMetrics m = engine.getMetrics();
        String metricsText = String.format(
            "Temps: %.1f s\n" +
            "Couverture: %.1f%%\n" +
            "Anomalies: %d\n" +
            "Actifs: %d\n" +
            "Recharge: %d\n" +
            "Mesures: %d\n" +
            "Vitesse: %.1fx",
            engine.getSimulationTime() / 1000.0,
            m.coveragePercentage,
            m.anomaliesDetected,
            m.activeDrones,
            m.rechargingDrones,
            (int)engine.getDrones().stream()
                .flatMap(d -> d.getMeasurements().stream())
                .count(),
            speedFactor
        );
        metricsLabel.setText(metricsText);
    }
    
    private void updateLog() {
        List<String> events = engine.getEventLog();
        if (events.size() > lastEventCount) {
            logArea.clear();
            for (String event : events) {
                logArea.appendText(event + "\n");
            }
            logArea.setScrollTop(Double.MAX_VALUE);
            lastEventCount = events.size();
        }
    }
    
    private void exportResults() {
        FileChooser fc = new FileChooser();
        fc.setTitle("Exporter résultats");
        fc.setInitialFileName("simulation_" + System.currentTimeMillis() + ".csv");
        fc.getExtensionFilters().add(new FileChooser.ExtensionFilter("CSV Files", "*.csv"));
        
        File file = fc.showSaveDialog(null);
        if (file != null) {
            try {
                String basePath = file.getAbsolutePath().replace(".csv", "");
                ExportUtils.exportMetricsToCSV(engine.getMetrics().toSnapshots(), basePath + "_metrics.csv");
                ExportUtils.exportMeasurementsToCSV(engine.getDrones(), basePath + "_measurements.csv");
                engine.logEvent("✅ Export réussi: " + basePath);
                showAlert("Succès", "Données exportées:\n" + basePath + "_metrics.csv\n" + basePath + "_measurements.csv");
            } catch (Exception ex) {
                engine.logEvent("❌ Erreur export: " + ex.getMessage());
                showAlert("Erreur", "Erreur lors de l'export: " + ex.getMessage());
            }
        }
    }
    
    private void showAlert(String title, String message) {
        Alert alert = new Alert(Alert.AlertType.INFORMATION);
        alert.setTitle(title);
        alert.setHeaderText(null);
        alert.setContentText(message);
        alert.showAndWait();
    }
    
    @Override
    public void stop() {
        if (animationTimer != null) {
            animationTimer.stop();
        }
    }
    
    public static void main(String[] args) {
        launch(args);
    }
}
