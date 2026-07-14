import * as exposes from 'zigbee-herdsman-converters/lib/exposes';
import * as reporting from 'zigbee-herdsman-converters/lib/reporting';

const e = exposes.presets;
const ea = exposes.access;

const batteryVoltageStatuses = {
    0: 'normal',
    1: 'over_voltage',
    2: 'under_voltage',
    3: 'low_voltage_disconnect',
    4: 'fault',
};

const batteryTemperatureStatuses = {
    0: 'normal',
    1: 'over_temperature',
    2: 'low_temperature',
};

const chargingStages = {
    0: 'not_charging',
    1: 'float',
    2: 'boost',
    3: 'equalization',
};

const pvInputStatuses = {
    0: 'normal',
    1: 'no_input',
    2: 'high_voltage',
    3: 'input_error',
};

const loadInputStatuses = {
    0: 'normal',
    1: 'low_voltage',
    2: 'high_voltage',
    3: 'no_input',
};

const loadPowerStatuses = {
    0: 'light',
    1: 'moderate',
    2: 'rated',
    3: 'overload',
};

const analogInputs = {
    2: {property: 'battery_state_of_charge'},
    5: {decode: decodeBatteryStatus},
    6: {decode: decodeChargingStatus},
    7: {decode: decodeDischargingStatus},
};

const electricalEndpoints = {
    1: {prefix: 'array', power: true},
    2: {prefix: 'battery', power: false},
    4: {prefix: 'load', power: true},
};

const temperatureEndpoints = {
    2: 'battery_temperature',
    3: 'controller_temperature',
};

function mappedStatus(lookup, value) {
    return lookup[value] ?? 'unknown';
}

function decodeBatteryStatus(value) {
    return {
        battery_voltage_status: mappedStatus(batteryVoltageStatuses, value & 0x0f),
        battery_temperature_status: mappedStatus(batteryTemperatureStatuses, (value >> 4) & 0x0f),
        battery_internal_resistance_abnormal: (value & (1 << 8)) !== 0,
        battery_rated_voltage_error: (value & (1 << 15)) !== 0,
    };
}

function decodeChargingStatus(value) {
    return {
        charging_stage: mappedStatus(chargingStages, (value >> 2) & 0x03),
        pv_input_status: mappedStatus(pvInputStatuses, (value >> 14) & 0x03),
        charging_running: (value & (1 << 0)) !== 0,
        charging_fault: (value & 0x3fd2) !== 0,
    };
}

function decodeDischargingStatus(value) {
    return {
        load_input_status: mappedStatus(loadInputStatuses, (value >> 14) & 0x03),
        load_power_status: mappedStatus(loadPowerStatuses, (value >> 12) & 0x03),
        load_running: (value & (1 << 0)) !== 0,
        load_fault: (value & 0x0ff2) !== 0,
    };
}

const fromElectricalMeasurement = {
    cluster: 'haElectricalMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => {
        const endpoint = electricalEndpoints[msg.endpoint.ID];
        if (!endpoint) {
            return;
        }

        const result = {};
        if (msg.data.dcVoltage !== undefined) {
            result[`${endpoint.prefix}_voltage`] = msg.data.dcVoltage / 100;
        }
        if (msg.data.dcCurrent !== undefined) {
            result[`${endpoint.prefix}_current`] = msg.data.dcCurrent / 100;
        }
        if (endpoint.power && msg.data.dcPower !== undefined) {
            result[`${endpoint.prefix}_power`] = msg.data.dcPower / 100;
        }
        return result;
    },
};

const fromTemperature = {
    cluster: 'msTemperatureMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => {
        const property = temperatureEndpoints[msg.endpoint.ID];
        const value = msg.data.measuredValue;
        if (!property || value === undefined || value === -32768) {
            return;
        }
        return {[property]: value / 100};
    },
};

const fromAnalogInput = {
    cluster: 'genAnalogInput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => {
        const input = analogInputs[msg.endpoint.ID];
        const value = msg.data.presentValue;
        if (!input || value === undefined) {
            return;
        }
        if (input.decode) {
            return input.decode(value);
        }
        return {[input.property]: value};
    },
};

const fromPowerConfiguration = {
    cluster: 'genPowerCfg',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => {
        const value = msg.data.batteryPercentageRemaining;
        if (msg.endpoint.ID !== 2 || value === undefined || value === 0xff) {
            return;
        }
        return {battery: value / 2};
    },
};

function numeric(name, unit, description) {
    const expose = e.numeric(name, ea.STATE).withDescription(description);
    return unit ? expose.withUnit(unit) : expose;
}

function enumeration(name, lookup, description) {
    return e.enum(name, ea.STATE, [...Object.values(lookup), 'unknown']).withDescription(description);
}

function binary(name, description) {
    return e.binary(name, ea.STATE, true, false).withDescription(description);
}

async function configureElectrical(endpoint, coordinatorEndpoint, includePower) {
    await reporting.bind(endpoint, coordinatorEndpoint, ['haElectricalMeasurement']);

    const attributes = [
        'measurementType',
        'dcVoltageMultiplier',
        'dcVoltageDivisor',
        'dcCurrentMultiplier',
        'dcCurrentDivisor',
        'dcVoltage',
        'dcCurrent',
    ];
    const reportings = [
        {attribute: 'dcVoltage', minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 10},
        {attribute: 'dcCurrent', minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 10},
    ];

    if (includePower) {
        attributes.push('dcPowerMultiplier', 'dcPowerDivisor', 'dcPower');
        reportings.push({attribute: 'dcPower', minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1});
    }

    await endpoint.configureReporting('haElectricalMeasurement', reportings);
    await endpoint.read('haElectricalMeasurement', attributes);
}

async function configureAnalogInput(endpoint, coordinatorEndpoint) {
    await reporting.bind(endpoint, coordinatorEndpoint, ['genAnalogInput']);
    await endpoint.configureReporting('genAnalogInput', [
        {attribute: 'presentValue', minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
    ]);
    await endpoint.read('genAnalogInput', ['presentValue']);
}

const definition = {
    fingerprint: [{modelID: 'EPSolar Zigbee', manufacturerName: 'DIY Solar'}],
    model: 'EPSolar Zigbee',
    vendor: 'DIY Solar',
    description: 'EPSolar solar controller telemetry bridge',
    fromZigbee: [fromElectricalMeasurement, fromTemperature, fromAnalogInput, fromPowerConfiguration],
    toZigbee: [],
    exposes: [
        numeric('array_voltage', 'V', 'Solar array voltage'),
        numeric('array_current', 'A', 'Solar array current'),
        numeric('array_power', 'W', 'Solar array power'),
        numeric('battery_temperature', '°C', 'Solar battery temperature'),
        e.battery(),
        numeric('battery_state_of_charge', '%', 'Solar battery state of charge').withValueMin(0).withValueMax(100),
        numeric('battery_voltage', 'V', 'Solar battery voltage'),
        numeric('battery_current', 'A', 'Solar battery current'),
        numeric('controller_temperature', '°C', 'Solar controller temperature'),
        numeric('load_voltage', 'V', 'Controller load voltage'),
        numeric('load_current', 'A', 'Controller load current'),
        numeric('load_power', 'W', 'Controller load power'),
        enumeration('battery_voltage_status', batteryVoltageStatuses, 'Battery voltage state'),
        enumeration('battery_temperature_status', batteryTemperatureStatuses, 'Battery temperature state'),
        binary('battery_internal_resistance_abnormal', 'Battery internal resistance warning'),
        binary('battery_rated_voltage_error', 'Battery rated-voltage identification error'),
        enumeration('charging_stage', chargingStages, 'Controller charging stage'),
        enumeration('pv_input_status', pvInputStatuses, 'PV input state'),
        binary('charging_running', 'Whether the controller is charging'),
        binary('charging_fault', 'Whether the charging circuit reports a fault'),
        enumeration('load_input_status', loadInputStatuses, 'Load input-voltage state'),
        enumeration('load_power_status', loadPowerStatuses, 'Load power state'),
        binary('load_running', 'Whether the controller load output is running'),
        binary('load_fault', 'Whether the load circuit reports a fault'),
    ],
    configure: async (device, coordinatorEndpoint) => {
        for (const [id, config] of Object.entries(electricalEndpoints)) {
            const endpoint = device.getEndpoint(Number(id));
            if (!endpoint) {
                throw new Error(`Missing electrical endpoint ${id}`);
            }
            await configureElectrical(endpoint, coordinatorEndpoint, config.power);
        }

        for (const id of Object.keys(temperatureEndpoints)) {
            const endpoint = device.getEndpoint(Number(id));
            if (!endpoint) {
                throw new Error(`Missing temperature endpoint ${id}`);
            }
            await reporting.bind(endpoint, coordinatorEndpoint, ['msTemperatureMeasurement']);
            await reporting.temperature(endpoint, {min: 10, max: 300, change: 10});
            await endpoint.read('msTemperatureMeasurement', ['measuredValue']);
        }

        const batteryEndpoint = device.getEndpoint(2);
        if (!batteryEndpoint) {
            throw new Error('Missing battery endpoint 2');
        }
        await reporting.bind(batteryEndpoint, coordinatorEndpoint, ['genPowerCfg']);
        await batteryEndpoint.configureReporting('genPowerCfg', [
            {
                attribute: 'batteryPercentageRemaining',
                minimumReportInterval: 10,
                maximumReportInterval: 300,
                reportableChange: 2,
            },
        ]);
        await batteryEndpoint.read('genPowerCfg', ['batteryPercentageRemaining']);

        for (const id of Object.keys(analogInputs)) {
            const endpoint = device.getEndpoint(Number(id));
            if (!endpoint) {
                throw new Error(`Missing analog input endpoint ${id}`);
            }
            await configureAnalogInput(endpoint, coordinatorEndpoint);
        }
    },
};

export default definition;
