// JavaScript file for parsing incoming data from the serial port

let columnMappings = null;
let csvDelimiter = ','; // Default delimiter

/**
 * Initializes the data parser by fetching the column mapping configuration.
 * @param {function} callback - Optional callback to execute after successful initialization.
 * @param {function} errorCallback - Optional callback to execute if initialization fails.
 */
async function initDataParser(callback, errorCallback) {
    try {
        console.log("Fetching data mapping configuration...");
        const response = await fetch('./js/flight_console_data_mapping.json');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status} while fetching mapping json`);
        }
        const mappingConfig = await response.json();
        
        if (!mappingConfig.column_mappings || !mappingConfig.csv_settings) {
            throw new Error("Invalid mapping configuration file format.");
        }

        columnMappings = mappingConfig.column_mappings;
        csvDelimiter = mappingConfig.csv_settings.delimiter || ',';
        
        console.log("Data mapping configuration loaded successfully.");
        console.log(`Using delimiter: '${csvDelimiter}'`);
        console.log(`Number of column mappings: ${columnMappings.length}`);

        if (callback) callback();
    } catch (error) {
        console.error("Error initializing data parser:", error);
        columnMappings = null; // Ensure mappings are null if loading fails
        if (errorCallback) errorCallback(error);
    }
}

/**
 * Parses a single line of CSV data based on the loaded column mappings.
 * @param {string} csvString - A string representing a single line of CSV data.
 * @returns {object|null} A JavaScript object with internal_name as keys and parsed values, 
 *                        or null if parsing fails or mappings are not loaded.
 */
function parseSerialData(csvString) {
    if (!columnMappings) {
        console.error("Data parser not initialized or mapping not loaded. Call initDataParser first.");
        return null;
    }
    if (typeof csvString !== 'string') {
        console.error("Invalid input: csvString must be a string.");
        return null;
    }

    const values = csvString.split(csvDelimiter);
    const expectedNumValues = columnMappings.length;

    // Basic check for the number of fields, could be made more robust
    // For now, we'll allow parsing even if field count is slightly off,
    // relying on default values for missing fields.
    if (values.length !== expectedNumValues) {
        console.warn(`CSV field count mismatch: expected ${expectedNumValues}, got ${values.length}. Data: "${csvString}"`);
        // Potentially, one could decide to return null here if strict matching is required.
    }

    const parsedObject = {};

    for (let i = 0; i < columnMappings.length; i++) {
        const mapping = columnMappings[i];
        let value = values[i]; // Value from CSV string at the current index

        // If the value from CSV is undefined (e.g., fewer columns than expected),
        // or if it's an empty string for a type that expects a value, use default.
        if (value === undefined || (value === "" && mapping.type !== "string")) { // Assuming empty string is not valid for int/float
            if (mapping.default_value !== undefined) {
                value = mapping.default_value;
                // console.log(`Using default value for ${mapping.internal_name}: ${value}`);
            } else {
                console.warn(`No value and no default for ${mapping.internal_name} at index ${i}. Setting to null.`);
                parsedObject[mapping.internal_name] = null;
                continue;
            }
        } else if (typeof value === 'string') {
             value = value.trim(); // Trim whitespace
        }


        try {
            switch (mapping.type) {
                case "int":
                    parsedObject[mapping.internal_name] = parseInt(value, 10);
                    if (isNaN(parsedObject[mapping.internal_name])) {
                        // console.warn(`Failed to parse int for ${mapping.internal_name} from value '${values[i]}'. Using default: ${mapping.default_value}`);
                        parsedObject[mapping.internal_name] = mapping.default_value;
                    }
                    break;
                case "float":
                    parsedObject[mapping.internal_name] = parseFloat(value);
                    if (isNaN(parsedObject[mapping.internal_name])) {
                        // console.warn(`Failed to parse float for ${mapping.internal_name} from value '${values[i]}'. Using default: ${mapping.default_value}`);
                        parsedObject[mapping.internal_name] = mapping.default_value;
                    }
                    break;
                case "string": // Though not in the current JSON, good to have
                    parsedObject[mapping.internal_name] = value;
                    break;
                default:
                    console.warn(`Unknown data type '${mapping.type}' for ${mapping.internal_name}. Storing as string or using default.`);
                    parsedObject[mapping.internal_name] = (mapping.default_value !== undefined) ? mapping.default_value : value;
            }
        } catch (conversionError) {
            console.error(`Error converting value for ${mapping.internal_name} (value: '${value}'):`, conversionError);
            parsedObject[mapping.internal_name] = mapping.default_value;
        }
    }
    return parsedObject;
}

// Example of how main.js might use this:
// initDataParser(() => {
//   console.log("Parser ready.");
//   // Now safe to call parseSerialData
//   const testData = "1,1678886400,3,12,34.0522,-118.2437,150.5,180.0,155.2,152.3,10.2,90.0,1.5,1,1012.5,25.5,0.1,0.2,9.8,0.1,0.2,9.7,0.01,0.02,0.03,0.1,0.2,0.3,22.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,0.2,0.1,0.0,0.0,0.0,0.0,0.0,0.0";
//   const parsed = parseSerialData(testData);
//   console.log("Parsed Data:", parsed);
// });
