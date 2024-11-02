var gateway = `ws://192.168.4.1/ws`;
var websocket;

window.addEventListener('load', onLoad);
function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    // websocket = new WebSocket(gateway);
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}
function onOpen(event) {
    console.log('Connection opened');
}
function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 100);
}
function onMessage(event) {
    var message = JSON.parse(event.data);
    if(message.Command == "Angle"){
        CurrentPosition(event.data);
    }
}
function onLoad(event) {
    initWebSocket();
    initButton();
}
function initButton() {
    document.getElementById("buttonstart").addEventListener("click", openPageControlller);
    document.getElementById("buttonKinematics").addEventListener("click", openKinematics);
    document.getElementById("buttonAuto").addEventListener("click", openAuto);
}
function openPageControlller(){
    document.getElementById("PageController").style.display = "block";
    document.getElementById("PageIntroduction").style.display = "none";
    document.getElementById("PageLogin").style.display = "none";
    var jsonStart = "{'Command':'startprogram'}";
    console.log(jsonStart);
    websocket.send(jsonStart);
}
function openKinematics(){
    document.getElementById("Kinematics").style.display = "block";
    document.getElementById("Auto").style.display = "none";
}
function openAuto(){
    document.getElementById("Kinematics").style.display = "none";
    document.getElementById("Auto").style.display = "block";
}