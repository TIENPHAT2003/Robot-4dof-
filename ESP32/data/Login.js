var pass = "123", ssid = "admin";
function checkPassword(password){
    var checkPassword = document.getElementById("password").value;
    var checkSSID = document.getElementById("ssid").value;
    if(checkSSID == ssid){
        if(checkPassword == pass){
            unlockSuccess();
            document.getElementById("ssid").value = "";
            document.getElementById("password").value = "";
            document.getElementById("PageIntroduction").style.display = "block";
            document.getElementById("PageLogin").style.display = "none";
        }else {
            unlockFail();
            document.getElementById("ssid").value = "";
            document.getElementById("password").value = "";
        }
    }
    else {
        unlockFail();
        document.getElementById("ssid").value = "";
        document.getElementById("password").value = "";
    }
    var jsonreset = "{'Command': 'Reset'}";
    console.log(jsonreset);
    websocket.send(jsonreset);
    var jsonSetHome = "{'Command': 'SetHome'}";
    console.log(jsonSetHome);
    websocket.send(jsonSetHome);
}
function unlockSuccess() {
    var modal = new bootstrap.Modal(document.getElementById('successModal'));
    modal.show();
    document.getElementById('successModaltext').innerHTML = "Login Successful!";
    setTimeout(function () { modal.hide(); }, 1000);
}
function unlockFail() {
    var modal = new bootstrap.Modal(document.getElementById('wrongPasswordModal'));
    modal.show();
    setTimeout(function () { modal.hide(); }, 1000);
}
document.getElementById("password").addEventListener("keyup", function (event) {
    if (event.key == "Enter") {
        event.preventDefault();
        checkPassword();
    }
});
document.getElementById("ssid").addEventListener("keyup", function (event) {
    if (event.key == "Enter") {
        event.preventDefault();
        checkPassword();
    }
});
