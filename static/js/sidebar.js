var socket = io.connect('http://' + document.domain + ':' + location.port);

const left_sidebar = document.getElementById("left-sidebar");
const left_open_btn = document.getElementById('left-open-btn');
const home_btn = document.getElementById('home-btn');

left_open_btn.onclick = function (event) {

    if (left_sidebar.style.width === '250px') {
        left_sidebar.style.width = '0';
        left_open_btn.classList.remove('shifted');
        home_btn.classList.remove('shifted');
        left_open_btn.innerHTML = '<i class="fas fa-bars"></i>';
    } else {
        left_sidebar.style.width = '250px';
        left_open_btn.classList.add('shifted');
        home_btn.classList.add('shifted');
        left_open_btn.innerHTML = '<i class="fas fa-times"></i>';;
    }

}

const right_sidebar = document.getElementById("right-sidebar");
const right_open_btn = document.getElementById('right-open-btn');

right_open_btn.onclick = function (event) {

    if (right_sidebar.style.width === '250px') {
        right_sidebar.style.width = '0';
        right_open_btn.classList.remove('shifted');
        right_open_btn.innerHTML = '<i class="fas fa-gamepad"></i>';
    } else {
        right_sidebar.style.width = '250px';
        right_open_btn.classList.add('shifted');
        right_open_btn.innerHTML = '<i class="fas fa-times"></i>';;
    }

}

var check_buttons = document.querySelectorAll('.radio-button');

check_buttons.forEach(function (button) {
    button.addEventListener('click', function () {

        // Send selection to backend
        socket.emit('controller_update', {'algorithm': this.textContent})

        // Toggle check
        toggleCheck(this);

    });
});

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