var     L1 	=	91
var     L2	=	122
var     L3	=	77
var     L4	=	79.5
var     d1	=	(62 + 176)
function updateJointAngleFromInput(joint) {
    const inputValue = document.getElementById(joint).value;
    const slider = document.getElementById(`${joint}-slider`);

    if (inputValue >= slider.min && inputValue <= slider.max) {
        document.getElementById(`${joint}-slider`).value = inputValue;
        document.getElementById(`${joint}-value`).textContent = inputValue;
        calculateForwardKinematics();
    }
}

function updateJointAngleFromSlider(joint) {
    const sliderValue = document.getElementById(`${joint}-slider`).value;
    document.getElementById(joint).value = sliderValue;
    document.getElementById(`${joint}-value`).textContent = sliderValue;
}

function calculateForwardKinematics() {
    var theta1Value = parseFloat(document.getElementById('theta1FK').value);
    var theta2Value = parseFloat(document.getElementById('theta2FK').value);
    var theta3Value = parseFloat(document.getElementById('theta3FK').value);
    var theta4Value = parseFloat(document.getElementById('theta4FK').value);

    let theta1_FK, theta2_FK, theta3_FK, theta4_FK;
    let theta1_FK_rad, theta2_FK_rad, theta3_FK_rad, theta4_FK_rad;
    let Px_FK, Py_FK, Pz_FK, theta_FK, theta_FK_rad;

    theta1_FK = theta1Value;
    theta1_FK_rad = (theta1_FK * Math.PI) / 180;

    theta2_FK = theta2Value;
    theta2_FK_rad = (theta2_FK * Math.PI) / 180;

    theta3_FK = theta3Value;
    theta3_FK_rad = (theta3_FK * Math.PI) / 180;

    theta4_FK = theta4Value;
    theta4_FK_rad = (theta4_FK * Math.PI) / 180;

    theta_FK = theta4_FK + theta2_FK + theta3_FK;
    theta_FK_rad = (theta_FK * Math.PI) / 180

    Px_FK = Math.cos(theta1_FK_rad) * (L1 + L2 * Math.cos(theta2_FK_rad) + L3 * Math.cos(theta2_FK_rad + theta3_FK_rad) + L4 * Math.cos(theta_FK_rad));
    Px_FK = Math.round(Px_FK * 10) / 10;

    Py_FK = Math.sin(theta1_FK_rad) * (L1 + L2 * Math.cos(theta2_FK_rad) + L3 * Math.cos(theta2_FK_rad + theta3_FK_rad) + L4 * Math.cos(theta_FK_rad));
    Py_FK = Math.round(Py_FK * 10) / 10;

    Pz_FK = d1 + L3 * Math.sin(theta2_FK_rad + theta3_FK_rad) + L2 * Math.sin(theta2_FK_rad) + L4 * Math.sin(theta_FK_rad);
    Pz_FK = Math.round(Pz_FK * 10) / 10;

    

    document.getElementById('pxFK').value = Px_FK;
    document.getElementById('pyFK').value = Py_FK;
    document.getElementById('pzFK').value = Pz_FK;
    document.getElementById('thetaFK').value = theta_FK;

    var jsonFK = "{'Command': 'forwardkinematics', 'Theta1':'"+ theta1_FK +"','Theta2':'"+ theta2_FK +"','Theta3':'"+ theta3_FK +"','Theta4':'"+ theta4_FK +"'}";
    console.log(jsonFK);
    websocket.send(jsonFK);
}
document.getElementById('theta1FK').onblur = function(){
    var theta1 = document.getElementById('theta1FK').value;
    if(theta1 > 90 || theta1 <  -90){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}
document.getElementById('theta2FK').onblur = function(){
    var theta2 = document.getElementById('theta2FK').value;
    if(theta2 > 90 || theta2 <  -90){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}
document.getElementById('theta3FK').onblur = function(){
    var theta3 = document.getElementById('theta3FK').value;
    if(theta3 > 90 || theta3 <  -90){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}
document.getElementById('theta4FK').onblur = function(){
    var theta4 = document.getElementById('theta4FK').value;
    if(theta4 > 90 || theta4 <  -90){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}
function CurrentPosition(jsonInput) {
    var jsonObj = JSON.parse(jsonInput);
    var theta1Value = jsonObj.Theta1;
    var theta2Value = jsonObj.Theta2;
    var theta3Value = jsonObj.Theta3;
    var theta4Value = jsonObj.Theta4;

    let theta1_FK, theta2_FK, theta3_FK, theta4_FK;
    let theta1_FK_rad, theta2_FK_rad, theta3_FK_rad, theta4_FK_rad;
    let Px_FK, Py_FK, Pz_FK, theta_FK, theta_FK_rad;

    theta1_FK = theta1Value;
    theta1_FK_rad = (theta1_FK * Math.PI) / 180;

    theta2_FK = theta2Value;
    theta2_FK_rad = (theta2_FK * Math.PI) / 180;

    theta3_FK = theta3Value;
    theta3_FK_rad = (theta3_FK * Math.PI) / 180;

    theta4_FK = theta4Value;
    theta4_FK_rad = (theta4_FK * Math.PI) / 180;

    theta_FK = theta4_FK + theta2_FK + theta3_FK;
    theta_FK_rad = (theta_FK * Math.PI) / 180

    Px_FK = Math.cos(theta1_FK_rad) * (L1 + L2 * Math.cos(theta2_FK_rad) + L3 * Math.cos(theta2_FK_rad + theta3_FK_rad) + L4 * Math.cos(theta_FK_rad));
    Px_FK = Math.round(Px_FK * 10) / 10;

    Py_FK = Math.sin(theta1_FK_rad) * (L1 + L2 * Math.cos(theta2_FK_rad) + L3 * Math.cos(theta2_FK_rad + theta3_FK_rad) + L4 * Math.cos(theta_FK_rad));
    Py_FK = Math.round(Py_FK * 10) / 10;

    Pz_FK = d1 + L3 * Math.sin(theta2_FK_rad + theta3_FK_rad) + L2 * Math.sin(theta2_FK_rad) + L4 * Math.sin(theta_FK_rad);
    Pz_FK = Math.round(Pz_FK * 10) / 10;
    
    document.getElementById('px').value = Px_FK;
    document.getElementById('py').value = Py_FK;
    document.getElementById('pz').value = Pz_FK;
    document.getElementById('theta').value = theta_FK;

    document.getElementById('theta1').value = theta1_FK;
    document.getElementById('theta2').value = theta2_FK;
    document.getElementById('theta3').value = theta3_FK;
    document.getElementById('theta4').value = theta4_FK;
}
