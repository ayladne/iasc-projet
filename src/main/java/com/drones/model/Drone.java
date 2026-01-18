package com.drones.model;
import com.drones.config.SimulationParams;
import java.util.*;
public class Drone {
    private int id;
    private double x, y;
    private double targetX, targetY;
    private DroneState state;
    private long autonomyRemaining; // ms
    private long measurementTimer; // ms, compte à rebours pendant la mesure
    private long rechargingTimer; // ms, compte à rebours pendant la recharge
    private List<Measurement> measurements;
    private Deque<double[]> waypoints; // file d'attente de cibles (x,y)
    
    public Drone(int id, double startX, double startY) {
        this.id = id;
        this.x = startX;
        this.y = startY;
        this.targetX = startX;
        this.targetY = startY;
        this.state = DroneState.ACTIVE;
        this.autonomyRemaining = SimulationParams.DRONE_AUTONOMY_MS;
        this.measurementTimer = 0;
        this.rechargingTimer = 0;
        this.measurements = new ArrayList<>();
        this.waypoints = new ArrayDeque<>();
    }
    
    public int getId() { return id; }
    public double getX() { return x; }
    public double getY() { return y; }
    public DroneState getState() { return state; }
    public long getAutonomyRemaining() { return autonomyRemaining; }
    public List<Measurement> getMeasurements() { return measurements; }
    
    // Ajouter une mesure (provenant d'une lecture de capteur)
    public void addMeasurement(double intensity, long timestamp, double x, double y) {
        measurements.add(new Measurement(intensity, timestamp, x, y));
    }
    
    // Effacer les mesures locales (télécharger à la base)
    public void clearMeasurements() {
        measurements.clear();
    }
    
    // Définir les points de passage pour le chemin planifié
    public void setWaypoints(List<double[]> points) {
        waypoints.clear();
        waypoints.addAll(points);
    }
    
    // Obtenir le prochain point de passage
    private boolean updateTargetWaypoint() {
        if (waypoints.isEmpty()) {
            return false;
        }
        double[] next = waypoints.peek();
        targetX = next[0];
        targetY = next[1];
        
        // Vérifier si cible est atteinte
        double dist = Math.sqrt(Math.pow(x - targetX, 2) + Math.pow(y - targetY, 2));
        if (dist < 0.5) {
            waypoints.poll();
            return !waypoints.isEmpty();
        }
        return true;
    }
    
    // Se déplacer vers la cible
    private void moveToward(double tx, double ty, double tickDurationS) {
        double dist = Math.sqrt(Math.pow(tx - x, 2) + Math.pow(ty - y, 2));
        if (dist < 0.1) return; // déjà là
        
        double speed = SimulationParams.DRONE_SPEED;
        double moveDistance = speed * tickDurationS;
        double ratio = Math.min(1.0, moveDistance / dist);
        
        x += (tx - x) * ratio;
        y += (ty - y) * ratio;
    }
    
    // Mettre à jour l'état du drone à chaque tick
    public void update(long tickDurationMs) {
        double tickDurationS = tickDurationMs / 1000.0;
        
        switch (state) {
            case ACTIVE:
                // Mettre à jour le point de passage si nécessaire
                updateTargetWaypoint();
                // Se déplacer vers la cible
                moveToward(targetX, targetY, tickDurationS);
                // Consommer l'autonomie
                autonomyRemaining -= tickDurationMs;
                if (autonomyRemaining <= 0) {
                    setState(DroneState.RETURNING);
                    targetX = 0;
                    targetY = 0;
                }
                break;
                
            case MEASURING:
                // Compte à rebours de la mesure
                measurementTimer -= tickDurationMs;
                autonomyRemaining -= tickDurationMs;
                if (measurementTimer <= 0) {
                    setState(DroneState.ACTIVE);
                }
                if (autonomyRemaining <= 0) {
                    setState(DroneState.RETURNING);
                }
                break;
                
            case RETURNING:
                // Se déplacer vers la base (0, 0)
                double dist = Math.sqrt(x * x + y * y);
                if (dist < 0.5) {
                    // Base atteinte
                    setState(DroneState.CHARGING);
                    rechargingTimer = SimulationParams.DRONE_RECHARGE_MS;
                    measurements.clear(); // télécharger à la base
                } else {
                    moveToward(0, 0, tickDurationS);
                    autonomyRemaining -= tickDurationMs;
                }
                break;
                
            case CHARGING:
                rechargingTimer -= tickDurationMs;
                if (rechargingTimer <= 0) {
                    setState(DroneState.ACTIVE);
                    autonomyRemaining = SimulationParams.DRONE_AUTONOMY_MS;
                }
                break;
        }
    }
    
    // Commencer une mesure à la position actuelle
    public void startMeasurement() {
        setState(DroneState.MEASURING);
        measurementTimer = SimulationParams.MEASUREMENT_DURATION_MS;
    }
    
    // Définir l'état
    public void setState(DroneState newState) {
        this.state = newState;
    }
    
    // Commodité: est à la base?
    public boolean isAtBase() {
        return Math.sqrt(x * x + y * y) < 0.5;
    }
    
    // Enregistrement des mesures
    public static class Measurement {
        public double intensity;
        public long timestamp;
        public double x, y;
        
        public Measurement(double intensity, long timestamp, double x, double y) {
            this.intensity = intensity;
            this.timestamp = timestamp;
            this.x = x;
            this.y = y;
        }
    }
}
