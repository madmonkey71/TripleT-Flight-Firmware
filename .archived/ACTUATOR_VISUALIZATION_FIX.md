# Actuator Visualization Fix Summary

## Issue Identified
During BOOST and COAST phases, the firmware outputs servo command data (`Servo CMDs (P,R,Y): 66, 86, 81`), but the web interface was not displaying this data in the actuator visualizations.

## Root Cause Analysis

### 1. **Missing Servo Command Parser**
- The firmware outputs servo commands as debug messages: `Servo CMDs (P,R,Y): 66, 86, 81`
- The web interface only parsed CSV data and ignored these servo command messages
- Servo commands represent actual servo angles (0-180°) being sent to the actuators

### 2. **Actuator Chart Configuration Issues**
- Chart was configured for only 2 datasets but expected 3 values (Roll, Pitch, Yaw)
- Chart scaling was inadequate for servo angle range (0-180°)
- Dataset labels were generic instead of axis-specific

### 3. **Data Flow Gap**
- CSV data contains `ActuatorOutRoll`, `ActuatorOutPitch`, `ActuatorOutYaw` (normalized -1.0 to 1.0)
- Servo commands contain actual servo angles being commanded
- Both data types need to be visualized for complete actuator monitoring

## Fixes Applied

### 1. **Added Servo Command Parser**
```javascript
function parseServoCommand(message) {
    const servoPattern = /^Servo CMDs \(P,R,Y\):\s*(\d+),\s*(\d+),\s*(\d+)$/;
    const match = message.match(servoPattern);
    
    if (match) {
        return {
            isServoCommand: true,
            timestamp: Date.now(),
            pitchServo: parseInt(match[1]),
            rollServo: parseInt(match[2]),
            yawServo: parseInt(match[3])
        };
    }
    return null;
}
```

### 2. **Enhanced Data Parser**
- Added servo command detection to `categorizeMessage()`
- Updated `parseData()` to handle servo commands alongside CSV data
- Maintains separation between normalized actuator outputs and servo angles

### 3. **Fixed Actuator Chart Configuration**
```javascript
function initActuatorChart(ctx) {
    return new Chart(ctx, {
        data: {
            datasets: [
                { label: 'Roll Actuator', borderColor: 'rgb(255, 99, 132)' },
                { label: 'Pitch Actuator', borderColor: 'rgb(54, 162, 235)' },
                { label: 'Yaw Actuator', borderColor: 'rgb(75, 192, 192)' }
            ]
        },
        options: {
            scales: {
                y: {
                    suggestedMin: -1,    // For normalized outputs
                    suggestedMax: 180    // For servo angles
                }
            }
        }
    });
}
```

### 4. **Updated UI Handler**
```javascript
function updateUI(parsedData) {
    // Handle servo command data
    if (parsedData.isServoCommand) {
        if (actuatorChartInstance) {
            updateChart(actuatorChartInstance, { 
                timestamp: parsedData.timestamp, 
                values: [parsedData.rollServo, parsedData.pitchServo, parsedData.yawServo] 
            });
        }
        return;
    }
    
    // Handle CSV actuator data (normalized values)
    if (actuatorChartInstance && parsedData.ActuatorOutRoll !== undefined) {
        updateChart(actuatorChartInstance, { 
            timestamp: parsedData.Timestamp, 
            values: [parseFloat(parsedData.ActuatorOutRoll), 
                    parseFloat(parsedData.ActuatorOutPitch), 
                    parseFloat(parsedData.ActuatorOutYaw)] 
        });
    }
}
```

### 5. **Enhanced Debug Logging**
- Added specific logging for servo command parsing
- Improved terminal output to show servo commands clearly
- Added test functions for debugging servo command parsing

## Data Types Visualized

### CSV Actuator Data (Continuous)
- **Source**: CSV fields `ActuatorOutRoll`, `ActuatorOutPitch`, `ActuatorOutYaw`
- **Range**: -1.0 to 1.0 (normalized PID outputs)
- **Frequency**: Every CSV message (~200ms intervals)

### Servo Command Data (Active Control Phases)
- **Source**: Debug messages `Servo CMDs (P,R,Y): pitch, roll, yaw`
- **Range**: 0° to 180° (actual servo angles)
- **Frequency**: During BOOST and COAST phases when guidance is active

## Testing and Validation

### Test Cases Added
1. **Servo Command Parsing**: Tests regex pattern matching
2. **Data Integration**: Verifies both CSV and servo data parsing
3. **Chart Updates**: Confirms actuator chart receives data correctly

### Expected Behavior
- **STARTUP/IDLE**: Only CSV actuator data (normalized values near 0)
- **BOOST/COAST**: Both CSV data AND servo commands (0-180° range)
- **RECOVERY**: CSV data only (servos return to neutral)

## Future Improvements

1. **Dual-Scale Charts**: Separate scales for normalized vs. angle data
2. **Phase Indicators**: Visual indicators showing when active control is engaged
3. **Command vs. Feedback**: Display both commanded and actual servo positions
4. **Performance Metrics**: PID error visualization and control effectiveness

## Files Modified

- `web_interface/js/data_parser.js` - Added servo command parsing
- `web_interface/js/visualizations.js` - Fixed actuator chart configuration  
- `web_interface/js/ui_updater.js` - Added servo command handling
- `web_interface/js/main.js` - Enhanced data processing and logging
- `web_interface/test_message_filtering.html` - Added servo command tests

## Result
The actuator visualization now properly displays servo command data during BOOST and COAST phases, providing real-time feedback on the guidance control system's actuator commands. 