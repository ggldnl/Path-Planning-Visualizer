const sidebar = document.getElementById("sidebar");
const open_btn = document.getElementById('open-btn');
const home_btn = document.getElementById('home-btn');

open_btn.onclick = function (event) {

    if (sidebar.style.width === '250px') {
        sidebar.style.width = '0';
        open_btn.classList.remove('shifted');
        home_btn.classList.remove('shifted');
        open_btn.innerHTML = '<i class="fas fa-bars"></i>';
    } else {
        sidebar.style.width = '250px';
        open_btn.classList.add('shifted');
        home_btn.classList.add('shifted');
        open_btn.innerHTML = '<i class="fas fa-times"></i>';;
    }

}
