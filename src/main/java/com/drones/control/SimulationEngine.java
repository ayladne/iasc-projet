package com.drones.control;
import com.drones.config.SimulationParams;
import com.drones.model.*;
import java.util.*;
public class SimulationEngine {
    private Environment environment;
    private List<Drone> drones;
    private long simulationTime;
    private boolean running;
    private SimulationMetrics metrics;
    private Coordinator coordinator;
    private int tickCount;
    private List<String> eventLog; // Journal des événements
    private Map<Integer, List<double[]>> droneTrajectories; // Trajectoires des drones
    
    public SimulationEngine() {
        this.environment = new Environment(SimulationParams.GRID_WIDTH, SimulationParams.GRID_HEIGHT);
        this.drones = new ArrayList<>();
        this.simulationTime = 0;
        this.running = false;
        this.metrics = new SimulationMetrics();
        this.coordinator = new Coordinator();
        this.tickCount = 0;
        this.eventLog = new ArrayList<>();
        this.droneTrajectories = new HashMap<>();
        
        // Initialiser les drones à la base (0, 0)
        for (int i = 0; i < SimulationParams.NUM_DRONES; i++) {
            Drone drone = new Drone(i, 0, 0);
            drones.add(drone);
            droneTrajectories.put(i, new ArrayList<>());
        }
        
        // Initialiser les points de passage (balayage raster simple)
        initializeCoverageWaypoints();
    }
    
    private void initializeCoverageWaypoints() {
        int dronesPerRow = (int) Math.ceil(Math.sqrt(SimulationParams.NUM_DRONES));
        int cellsPerDrone = SimulationParams.GRID_WIDTH / dronesPerRow;
        
        for (int i = 0; i < drones.size(); i++) {
            List<double[]> waypoints = new ArrayList<>();
            
            // Affecter une région à chaque drone
            int row = i / dronesPerRow;
            int col = i % dronesPerRow;
            
            int startX = col * cellsPerDrone;
            int startY = row * cellsPerDrone;
            int endX = Math.min((col + 1) * cellsPerDrone, SimulationParams.GRID_WIDTH);
            int endY = Math.min((row + 1) * cellsPerDrone, SimulationParams.GRID_HEIGHT);
            
            // Motif de balayage raster
            for (int y = startY; y < endY; y++) {
                if ((y - startY) % 2 == 0) {
                    for (int x = startX; x < endX; x++) {
                        waypoints.add(new double[]{x, y});
                    }
                } else {
                    for (int x = endX - 1; x >= startX; x--) {
                        waypoints.add(new double[]{x, y});
                    }
                }
            }
            
            // Retour à la base
            waypoints.add(new double[]{0, 0});
            
            drones.get(i).setWaypoints(waypoints);
        }
    }
    
    public void tick() {
        if (!running) return;
        
        // Mettre à jour l'environnement
        environment.update(SimulationParams.TICK_DURATION_MS);
        
        // Mettre à jour les drones
        for (Drone drone : drones) {
            double oldState = drone.getState().ordinal();
            drone.update(SimulationParams.TICK_DURATION_MS);
            
            // Suivi de la trajectoire
            droneTrajectories.get(drone.getId()).add(new double[]{drone.getX(), drone.getY()});
            
            // Journal des changements d'état
            if (drone.getState().ordinal() != oldState) {
                logEvent("Drone " + drone.getId() + " → " + drone.getState().getLabel());
            }
            
            // Si le drone est actif et à un point de passage, mesurer
            if (drone.getState() == DroneState.ACTIVE) {
                double intensity = environment.getAnomalyAt(drone.getX(), drone.getY());
                if (intensity > SimulationParams.ANOMALY_DETECTION_THRESHOLD) {
                    // Ajouter du bruit à la mesure
                    double measured = intensity + (Math.random() - 0.5) * 0.1;
                    drone.addMeasurement(measured, simulationTime, drone.getX(), drone.getY());
                    logEvent("Drone " + drone.getId() + " détecte anomalie à (" + 
                    String.format("%.1f", drone.getX()) + "," + 
                    String.format("%.1f", drone.getY()) + ") - Intensité: " +
                    String.format("%.2f", measured));
                }
            }
            
            // Vérifier si retour à la base
            if (drone.getState() == DroneState.RETURNING && drone.isAtBase()) {
                logEvent("Drone " + drone.getId() + " est retourné à la base");
            }
        }
        
        // Réaffectation adaptative chaque 30 ticks (6 secondes)
        if (tickCount++ % 30 == 0) {
            coordinator.adaptiveRetasking(drones, environment);
        }
        
        // Mettre à jour les métriques
        metrics.update(drones, environment, simulationTime);
        
        simulationTime += SimulationParams.TICK_DURATION_MS;
    }
    
    public void start() {
        this.running = true;
    }
    
    public void stop() {
        this.running = false;
    }
    
    public void reset() {
        environment.reset();
        for (Drone d : drones) {
            d.clearMeasurements();
        }
        simulationTime = 0;
        tickCount = 0;
        running = false;
        metrics.reset();
        coordinator.reset();
        eventLog.clear();
        for (List<double[]> traj : droneTrajectories.values()) {
            traj.clear();
        }
        initializeCoverageWaypoints();
    }
    
    public void logEvent(String message) {
        String timestamp = String.format("[%.1f s] ", simulationTime / 1000.0);
        eventLog.add(timestamp + message);
    }
    
    public List<String> getEventLog() {
        return eventLog;
    }
    
    public Map<Integer, List<double[]>> getDroneTrajectories() {
        return droneTrajectories;
    }
    
    public Environment getEnvironment() { return environment; }
    public List<Drone> getDrones() { return drones; }
    public long getSimulationTime() { return simulationTime; }
    public boolean isRunning() { return running; }
    public SimulationMetrics getMetrics() { return metrics; }
    
    // Conteneur de métriques
    public static class SimulationMetrics {
        public double coveragePercentage;
        public int anomaliesDetected;
        public double averageDetectionTime;
        public int activeDrones;
        public int rechargingDrones;
        private List<MetricsSnapshot> snapshots = new ArrayList<>();
        
        public void update(List<Drone> drones, Environment env, long time) {
            // Compter actifs/recharge
            activeDrones = (int) drones.stream().filter(d -> d.getState() == DroneState.ACTIVE).count();
            rechargingDrones = (int) drones.stream().filter(d -> d.getState() == DroneState.CHARGING).count();
            
            // Compter les cellules avec anomalies détectées
            double[][] grid = env.getAnomalyIntensity();
            int cellsWithAnomaly = 0;
            for (double[] row : grid) {
                for (double val : row) {
                    if (val > SimulationParams.ANOMALY_DETECTION_THRESHOLD) {
                        cellsWithAnomaly++;
                    }
                }
            }
            
            int totalCells = env.getWidth() * env.getHeight();
            coveragePercentage = (double) cellsWithAnomaly / totalCells * 100.0;
            
            anomaliesDetected = env.getAnomalies().size();
            
            // Ajouter un snapshot chaque 5 secondes
            if (time % 5000 == 0) {
                snapshots.add(new MetricsSnapshot(time, coveragePercentage, anomaliesDetected, activeDrones, rechargingDrones));
            }
        }
        
        public void reset() {
            coveragePercentage = 0;
            anomaliesDetected = 0;
            averageDetectionTime = 0;
            activeDrones = 0;
            rechargingDrones = 0;
            snapshots.clear();
        }
        
        public List<MetricsSnapshot> toSnapshots() {
            return new ArrayList<>(snapshots);
        }
        
        public static class MetricsSnapshot {
            public long time;
            public double coverage;
            public int anomalies;
            public int activeDrones;
            public int rechargingDrones;
            
            public MetricsSnapshot(long time, double coverage, int anomalies, int active, int charging) {
                this.time = time;
                this.coverage = coverage;
                this.anomalies = anomalies;
                this.activeDrones = active;
                this.rechargingDrones = charging;
            }
        }
    }
}
