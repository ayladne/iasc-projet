package com.drones.metrics;

import com.drones.model.Drone;
import com.drones.model.Environment;
import java.util.*;

public class MetricsCollector {
    private List<MetricsSnapshot> snapshots;
    private long lastSnapshotTime;
    private static final long SNAPSHOT_INTERVAL = 5000; // 5 seconds
    
    public MetricsCollector() {
        snapshots = new ArrayList<>();
        lastSnapshotTime = 0;
    }
    
    public void snapshot(List<Drone> drones, Environment env, long time) {
        if (time - lastSnapshotTime >= SNAPSHOT_INTERVAL) {
            // Count active/charging drones
            int active = (int) drones.stream().filter(d -> d.getState().name().equals("ACTIVE")).count();
            int charging = (int) drones.stream().filter(d -> d.getState().name().equals("CHARGING")).count();
            
            // Calculate coverage
            double[][] grid = env.getAnomalyIntensity();
            int cellsWithAnomaly = 0;
            for (double[] row : grid) {
                for (double val : row) {
                    if (val > 0.3) cellsWithAnomaly++;
                }
            }
            
            double coverage = (double) cellsWithAnomaly / (env.getWidth() * env.getHeight()) * 100.0;
            
            snapshots.add(new MetricsSnapshot(
                time,
                coverage,
                env.getAnomalies().size(),
                active,
                charging,
                drones.size()
            ));
            
            lastSnapshotTime = time;
        }
    }
    
    public List<MetricsSnapshot> getSnapshots() {
        return snapshots;
    }
    
    public void reset() {
        snapshots.clear();
        lastSnapshotTime = 0;
    }
    
    public static class MetricsSnapshot {
        public long time;
        public double coveragePercent;
        public int anomaliesCount;
        public int activeDrones;
        public int chargingDrones;
        public int totalDrones;
        
        public MetricsSnapshot(long time, double coverage, int anomalies, 
                              int active, int charging, int total) {
            this.time = time;
            this.coveragePercent = coverage;
            this.anomaliesCount = anomalies;
            this.activeDrones = active;
            this.chargingDrones = charging;
            this.totalDrones = total;
        }
    }
}
