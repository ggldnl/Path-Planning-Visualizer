var socket = io.connect('http://' + document.domain + ':' + location.port);
export { socket };

socket.on('connect', function() {
    console.log('Connected to WebSocket');
});

socket.on('disconnect', function() {
    console.log('Disconnected from WebSocket');
});

document.getElementById('load-btn').addEventListener('click', function() {

    // Create a file input element
    var fileInput = document.createElement('input');
    fileInput.type = 'file';

    // Set an event listener for when a file is selected
    fileInput.addEventListener('change', function (event) {
        var file = event.target.files[0];

        if (file) {
            // Read the content of the file as text
            var reader = new FileReader();
            reader.onload = function (e) {
                var fileContent = e.target.result;

                try {
                    var jsonData = JSON.parse(fileContent);
                    socket.emit('map_update', {'load': jsonData});
                    console.log('JSON Data:', jsonData);
                } catch (error) {
                    console.error('Error parsing JSON:', error);
                }
            };

            reader.readAsText(file);
        }
    });

    // Trigger a click event on the file input to open the file chooser dialog
    fileInput.click();
});

document.getElementById('load-URDF-btn').addEventListener('click', function () {
    // Create a file input element
    var fileInput = document.createElement('input');
    fileInput.type = 'file';

    // Set an event listener for when a file is selected
    fileInput.addEventListener('change', function (event) {
        var file = event.target.files[0];

        if (file) {
            // Read the content of the file as text
            var reader = new FileReader();
            reader.onload = function (e) {
                var fileContent = e.target.result;

                try {
                    // Use DOMParser to parse XML content
                    var parser = new DOMParser();
                    var xmlDoc = parser.parseFromString(fileContent, 'application/xml');
                    var xmlString = new XMLSerializer().serializeToString(xmlDoc);

                    // Now you can send the XML content through the WebSocket
                    socket.emit('robot_update', { 'load': xmlString });
                    console.log('URDF Data: ', xmlString)
                } catch (error) {
                    console.error('Error parsing XML:', error);
                }
            };

            reader.readAsText(file);
        }
    });

    // Trigger a click event on the file input to open the file chooser dialog
    fileInput.click();
});

document.getElementById('random-btn').addEventListener('click', function() {
    socket.emit('map_update', {'random': null});
});

document.getElementById('robots-linear-velocity-slider').addEventListener('input', function() {
    var value = this.value;
    // console.log('robots-linear-velocity: ', value);
    socket.emit('robot_update', {'linear_velocity': value});
});

document.getElementById('robots-angular-velocity-slider').addEventListener('input', function() {
    var value = this.value;
    // console.log('robots-angular-velocity: ', value);
    socket.emit('robot_update', {'angular_velocity': value});
});

document.getElementById('start-btn').addEventListener('click', function() {
    socket.emit('simulation_control_update', 'start');
});

document.getElementById('stop-btn').addEventListener('click', function() {
    socket.emit('simulation_control_update', 'stop');
});

document.getElementById('step-btn').addEventListener('click', function() {
    socket.emit('simulation_control_update', 'step');
});

document.getElementById('reset-btn').addEventListener('click', function() {
    socket.emit('simulation_control_update', 'reset');
});

document.getElementById('show-path-chk').addEventListener('change', function() {
    socket.emit('simulation_settings_update', {'show_path': this.checked});
});

document.getElementById('show-data-structures-chk').addEventListener('change', function() {
    socket.emit('simulation_settings_update', {'show_data_structures': this.checked});
});

document.getElementById('autostart-chk').addEventListener('change', function() {
    socket.emit('simulation_settings_update', {'autostart': this.checked});
});

document.getElementById('iterations-per-step-slider').addEventListener('input', function() {
    var value = this.value;
    // console.log('iterations-per-step: ', value);
    socket.emit('algorithm_update', {'iterations_per_step': value});
});

document.getElementById('go-to-btn').addEventListener('click', function() {
    socket.emit('algorithm_update', {'expire': true})
});
