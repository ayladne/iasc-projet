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

import com.drones.config.SimulationParams;
import com.drones.control.SimulationEngine;
import com.drones.model.*;

public class MainApp extends Application {
    
    private SimulationEngine engine;
    private Canvas canvas;
    private AnimationTimer animationTimer;
    private TextArea logArea;
    private Label metricsLabel;
    private Button startButton, stopButton, resetButton;
    private boolean isPaused = false;
    
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
        logArea.setPrefHeight(100);
        logArea.setEditable(false);
        logArea.setWrapText(true);
        root.setBottom(logArea);
        
        // Scene
        Scene scene = new Scene(root, 900, 800);
        primaryStage.setTitle("Autonomous Drone Swarm Simulator");
        primaryStage.setScene(scene);
        primaryStage.show();
        
        // Start animation loop
        startAnimationLoop();
        
        log("Simulation initialized. Press Start to begin.");
    }
    
    private VBox createControlPanel() {
        VBox panel = new VBox(10);
        panel.setPadding(new Insets(10));
        panel.setStyle("-fx-border-color: #ccc; -fx-border-width: 0 0 0 1;");
        panel.setPrefWidth(200);
        
        // Title
        Label title = new Label("Simulation Control");
        title.setStyle("-fx-font-size: 14; -fx-font-weight: bold;");
        panel.getChildren().add(title);
        
        // Buttons
        startButton = new Button("Start");
        stopButton = new Button("Stop");
        resetButton = new Button("Reset");
        
        startButton.setPrefWidth(180);
        stopButton.setPrefWidth(180);
        resetButton.setPrefWidth(180);
        
        startButton.setOnAction(e -> {
            engine.start();
            isPaused = false;
            log("Simulation started.");
        });
        
        stopButton.setOnAction(e -> {
            engine.stop();
            isPaused = true;
            log("Simulation stopped.");
        });
        
        resetButton.setOnAction(e -> {
            engine.reset();
            logArea.clear();
            log("Simulation reset.");
        });
        
        panel.getChildren().addAll(startButton, stopButton, resetButton);
        
        // Separator
        Separator sep1 = new Separator();
        panel.getChildren().add(sep1);
        
        // Metrics
        Label metricsTitle = new Label("Metrics");
        metricsTitle.setStyle("-fx-font-size: 12; -fx-font-weight: bold;");
        panel.getChildren().add(metricsTitle);
        
        metricsLabel = new Label();
        metricsLabel.setStyle("-fx-font-size: 10;");
        metricsLabel.setWrapText(true);
        panel.getChildren().add(metricsLabel);
        
        // Separator
        Separator sep2 = new Separator();
        panel.getChildren().add(sep2);
        
        // Info
        Label infoTitle = new Label("Info");
        infoTitle.setStyle("-fx-font-size: 12; -fx-font-weight: bold;");
        panel.getChildren().add(infoTitle);
        
        Label infoText = new Label(
            "Grid: " + SimulationParams.GRID_WIDTH + "x" + SimulationParams.GRID_HEIGHT + "\n" +
            "Drones: " + SimulationParams.NUM_DRONES + "\n" +
            "Tick: " + SimulationParams.TICK_DURATION_MS + " ms"
        );
        infoText.setStyle("-fx-font-size: 10;");
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
                    if (now - lastTick >= TICK_NANOS) {
                        engine.tick();
                        lastTick = now;
                    }
                }
                
                render();
                updateMetrics();
            }
        };
        animationTimer.start();
    }
    
    private void render() {
        GraphicsContext gc = canvas.getGraphicsContext2D();
        
        // Clear canvas (white background)
        gc.setFill(Color.WHITE);
        gc.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());
        
        // Draw grid lines (light gray)
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
                    // Color gradient from yellow (low) to red (high)
                    Color color = interpolateColor(intensity);
                    gc.setFill(color);
                    gc.fillRect(x * SimulationParams.CELL_SIZE_PX,
                               y * SimulationParams.CELL_SIZE_PX,
                               SimulationParams.CELL_SIZE_PX,
                               SimulationParams.CELL_SIZE_PX);
                }
            }
        }
        
        // Draw base (green square at origin)
        gc.setFill(Color.GREEN);
        gc.fillRect(0, 0, SimulationParams.CELL_SIZE_PX, SimulationParams.CELL_SIZE_PX);
        gc.setStroke(Color.DARKGREEN);
        gc.setLineWidth(2);
        gc.strokeRect(0, 0, SimulationParams.CELL_SIZE_PX, SimulationParams.CELL_SIZE_PX);
        
        // Draw drones
        for (Drone drone : engine.getDrones()) {
            drawDrone(gc, drone);
        }
    }
    
    private void drawDrone(GraphicsContext gc, Drone drone) {
        int px = (int) (drone.getX() * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
        int py = (int) (drone.getY() * SimulationParams.CELL_SIZE_PX) + SimulationParams.CELL_SIZE_PX / 2;
        int radius = 5;
        
        // State color
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
        
        // Draw ID
        gc.setFill(Color.BLACK);
        gc.setFont(new javafx.scene.text.Font(8));
        gc.fillText(String.valueOf(drone.getId()), px - 2, py + 3);
    }
    
    private Color interpolateColor(double intensity) {
        // Yellow (0) -> Orange -> Red (1)
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
            "Mesures: %d",
            engine.getSimulationTime() / 1000.0,
            m.coveragePercentage,
            m.anomaliesDetected,
            m.activeDrones,
            m.rechargingDrones,
            (int)engine.getDrones().stream()
                .flatMap(d -> d.getMeasurements().stream())
                .count()
        );
        metricsLabel.setText(metricsText);
    }
    
    private void log(String message) {
        long now = System.currentTimeMillis();
        String timestamp = String.format("[%.1f s] ", now / 1000.0);
        logArea.appendText(timestamp + message + "\n");
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
