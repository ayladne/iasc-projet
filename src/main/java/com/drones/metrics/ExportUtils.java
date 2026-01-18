package com.drones.metrics;
import com.drones.model.Drone;
import com.drones.control.SimulationEngine;
import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;
public class ExportUtils {
    
    /**
     * Exporter les métriques de simulation dans un fichier CSV
     */
    public static void exportMetricsToCSV(List<SimulationEngine.SimulationMetrics.MetricsSnapshot> snapshots, 
        String filename) throws IOException {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
            // Entêtem
            writer.println("Temps(s),Couverture(%),Anomalies,DronesActifs,DronesEnRecharge");
            
            // Données
            for (SimulationEngine.SimulationMetrics.MetricsSnapshot snapshot : snapshots) {
                writer.printf("%.1f,%.2f,%d,%d,%d%n",
                    snapshot.time / 1000.0,
                    snapshot.coverage,
                    snapshot.anomalies,
                    snapshot.activeDrones,
                    snapshot.rechargingDrones
                );
            }
        }
    }
    
    /**
     * Exporter les mesures des drones dans un fichier CSV
     */
    public static void exportMeasurementsToCSV(List<Drone> drones, String filename) throws IOException {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
            // Entêtem
            writer.println("IDDrone,Temps(s),Intensité,X,Y");
            
            // Collecter toutes les mesures de tous les drones
            for (Drone drone : drones) {
                for (Drone.Measurement m : drone.getMeasurements()) {
                    writer.printf("%d,%.1f,%.3f,%.1f,%.1f%n",
                        drone.getId(),
                        m.timestamp / 1000.0,
                        m.intensity,
                        m.x,
                        m.y
                    );
                }
            }
        }
    }
    
    /**
     * Générer un horodatage pour le nom de fichier d'exportation
     */
    public static String generateTimestamp() {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss");
        return sdf.format(new Date());
    }
}
