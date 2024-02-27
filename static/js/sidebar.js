import { socket } from './websocket.js';

// var socket = io.connect('http://' + document.domain + ':' + location.port);

const left_sidebar = document.getElementById("left-sidebar");
const left_open_btn = document.getElementById('left-open-btn');
const home_btn = document.getElementById('home-btn');
const track_btn = document.getElementById('track-btn');

left_open_btn.onclick = function (event) {

    if (left_sidebar.classList.contains('shifted')) {
        left_sidebar.classList.remove('shifted');
        left_open_btn.classList.remove('shifted');
        home_btn.classList.remove('shifted');
        track_btn.classList.remove('shifted');
        left_open_btn.innerHTML = '<i class="fas fa-bars"></i>';
    } else {
        left_sidebar.classList.add('shifted');
        left_open_btn.classList.add('shifted');
        home_btn.classList.add('shifted');
        track_btn.classList.add('shifted');
        left_open_btn.innerHTML = '<i class="fas fa-times"></i>';;
    }

}

const right_sidebar = document.getElementById("right-sidebar");
const right_open_btn = document.getElementById('right-open-btn');
const go_to_btn = document.getElementById('go-to-btn');

right_open_btn.onclick = function (event) {

    if (right_sidebar.classList.contains('shifted')) {
        right_sidebar.classList.remove('shifted');
        right_open_btn.classList.remove('shifted');
        go_to_btn.classList.remove('shifted');
        right_open_btn.innerHTML = '<i class="fas fa-gamepad"></i>';
    } else {
        right_sidebar.classList.add('shifted');
        right_open_btn.classList.add('shifted');
        go_to_btn.classList.add('shifted');
        right_open_btn.innerHTML = '<i class="fas fa-times"></i>';;
    }

}

var check_buttons = document.querySelectorAll('.radio-button');

check_buttons.forEach(function (button) {
    button.addEventListener('click', function () {

        // Send selection to backend
        const buttonText = button.textContent.replace(/\s/g, ''); // Remove spaces
        socket.emit('algorithm_control', buttonText);
        // console.log('Selected algorithm:', buttonText)

        // Toggle check
        // toggleCheck(this);

    });
});

/*
function toggleCheck(clickedButton) {
    // Get the parent container
    var buttonContainer = document.getElementById('right-sidebar');

    // Remove 'checked' class from all buttons inside the container
    var buttons = buttonContainer.querySelectorAll('.radio-button');
    buttons.forEach(function (button) {
        button.classList.remove('checked');
    });

    // Add 'checked' class to the clicked button
    clickedButton.classList.add('checked');
}
*/

/*
socket.on('notify_controller_update', function (string_data) {
    check_buttons.forEach(function(button, index) {
        const buttonText = button.textContent.replace(/\s/g, ''); // Remove spaces
        if (buttonText === string_data) {
            toggleCheck(button)
        }
    });
});
*/