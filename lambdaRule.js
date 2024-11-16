const AWS = require('aws-sdk');
const iotData = new AWS.IotData({ endpoint: 'a25ex5q24thysh-ats.iot.us-east-1.amazonaws.com' });
let thing_name;

// Parámetros para encender y apagar el dispositivo (buzzer)
const TurnOnParams = {
    topic: `$aws/things/Esp32-sensor/shadow/update`,
    payload: '{"state": {"desired": {"builtInBuzzer": 1}}}',
};

const TurnOffParams = {
    topic: `$aws/things/Esp32-sensor/shadow/update`,
    payload: '{"state": {"desired": {"builtInBuzzer": 0}}}',
};

// Funciones para encender y apagar el buzzer
const turnOnBuzzer = async () => {
    console.log('Encendiendo el buzzer...');
    iotData.publish(TurnOnParams, function(err, data) {
        if (err) console.log(err);
    });
};

const turnOffBuzzer = async () => {
    console.log('Apagando el buzzer...');
    iotData.publish(TurnOffParams, function(err, data) {
        if (err) console.log(err);
    });
};

// Lógica principal para manejar el evento de ritmo cardíaco
exports.handler = async (event) => {
    console.log('Received event:', JSON.stringify(event, null, 2));

    thing_name = event.thing_name;
    const heartRate = event.heartRate; // Ritmo cardíaco detectado

    console.log('Ritmo cardíaco:', heartRate);

    // Lógica para encender o apagar el buzzer según el ritmo cardíaco
    if (heartRate > 100) {
        // Si el ritmo cardíaco es mayor a 100, encendemos el buzzer
        await turnOnBuzzer();
    } else if (heartRate < 60) {
        // Si el ritmo cardíaco es menor a 60, apagamos el buzzer
        await turnOffBuzzer();
    }

    return {
        speechText: `El ritmo cardíaco es ${heartRate} latidos por minuto. Acción tomada.`
    };
};
