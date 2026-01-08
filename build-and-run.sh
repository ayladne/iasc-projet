#!/bin/bash
# build-and-run.sh - Build and run the drone swarm simulator

echo "=========================================="
echo "Drone Swarm Simulator - Build & Run"
echo "=========================================="
echo ""

# Check if Maven is installed
if ! command -v mvn &> /dev/null; then
    echo "❌ Maven not found. Please install Maven first."
    exit 1
fi

echo "📦 Cleaning previous build..."
mvn clean

echo ""
echo "🔨 Compiling project..."
mvn compile -DskipTests

if [ $? -ne 0 ]; then
    echo "❌ Compilation failed!"
    exit 1
fi

echo ""
echo "✅ Build successful!"
echo ""
echo "🚀 Starting simulation..."
echo "   (Close the window to exit)"
echo ""

mvn javafx:run

echo ""
echo "👋 Simulation closed."
