var socket = io.connect('http://' + document.domain + ':' + location.port);

socket.on('connect', function() {
    console.log('Connected to WebSocket');
});

socket.on('disconnect', function() {
    console.log('Disconnected from WebSocket');
});

document.getElementById('save-btn').addEventListener('click', function() {
    // TODO
    var message = 'Saving...';
    console.log(message);
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
                    socket.emit('map_io_command', 'load', jsonData);
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

document.getElementById('random-btn').addEventListener('click', function() {
    socket.emit('map_io_command', 'random');
});

document.getElementById('obstacles-spawn-frequency-slider').addEventListener('input', function() {
    var value = this.value;
    console.log('obstacles-spawn-frequency: ', value);
    socket.emit('world_status_update', {'obstacles_spawn_frequency': value});
});

document.getElementById('robots-linear-velocity-slider').addEventListener('input', function() {
    var value = this.value;
    console.log('robots-linear-velocity: ', value);
    socket.emit('world_status_update', {'robots_linear_velocity': value});
});

document.getElementById('robots-angular-velocity-slider').addEventListener('input', function() {
    var value = this.value;
    console.log('robots-angular-velocity: ', value);
    socket.emit('world_status_update', {'robots_angular_velocity': value});
});

document.getElementById('start-btn').addEventListener('click', function() {
    socket.emit('simulation_controls_update', 'start');
});

document.getElementById('stop-btn').addEventListener('click', function() {
    socket.emit('simulation_controls_update', 'stop');
});

document.getElementById('step-btn').addEventListener('click', function() {
    socket.emit('simulation_controls_update', 'step');
});

document.getElementById('reset-btn').addEventListener('click', function() {
    socket.emit('simulation_controls_update', 'reset');
});
