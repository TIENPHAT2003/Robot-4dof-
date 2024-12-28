var Pz_IK_SetPoint = 80;
var Pz_IK_SelectPoint = 25;
document.getElementById('setpoint').value = Pz_IK_SetPoint;
document.getElementById('selectpoint').value = Pz_IK_SelectPoint;

document.getElementById('setpoint').onblur = function(){
    Pz_IK_SetPoint = document.getElementById('setpoint').value;
    console.log("Pz_SetPoint: " + Pz_IK_SetPoint);
};
document.getElementById('selectpoint').onblur = function(){
    Pz_IK_SelectPoint = document.getElementById('selectpoint').value;
    console.log("Pz_SelectPoint: " + Pz_IK_SelectPoint);
};
function tabelData(jsonData) {
    var TableHTML = "";
    TableHTML += "<table class=\"table table-borderless\" style=\"border: 2px solid white; color: white; border-collapse: collapse;\"><thead class=\"thead-dark\"><th style=\"border: 1px solid white;\">Point</th><th style=\"border: 1px solid white;\">Theta1</th><th style=\"border: 1px solid white;\">Theta2</th><th style=\"border: 1px solid white;\">Theta3</th><th style=\"border: 1px solid white;\">Theta4</th></thead><tbody>";
    var Data = JSON.parse(jsonData);
    for (var i = 0; i < Data.Data.length; i++) {
        var point = parseInt(Data.Data[i].Point) + 1;
        var theta1 = Data.Data[i].Theta1;
        var theta2 = Data.Data[i].Theta2;
        var theta3 = Data.Data[i].Theta3;
        var theta4 = Data.Data[i].Theta4;
        TableHTML += "<tr style=\"border: 1px solid white;\"><td style=\"border: 1px solid white;\">" + point + "</td><td style=\"border: 1px solid white;\">" + theta1 + "</td><td style=\"border: 1px solid white;\">" + theta2 + "</td><td style=\"border: 1px solid white;\">" + theta3 + "</td><td style=\"border: 1px solid white;\">" + theta4 + "</td></tr>";
    }
    TableHTML += "</tbody></table>";
    document.getElementById('tabledata').innerHTML = TableHTML;
}
function tabelSetPointData(jsonData) {
    var TableHTMLSet = "";
    TableHTMLSet += "<table class=\"table table-borderless\" style=\"border: 2px solid white; color: white; border-collapse: collapse;\"><thead class=\"thead-dark\"><th style=\"border: 1px solid white;\">Point</th><th style=\"border: 1px solid white;\">Theta1</th><th style=\"border: 1px solid white;\">Theta2</th><th style=\"border: 1px solid white;\">Theta3</th><th style=\"border: 1px solid white;\">Theta4</th></thead><tbody>";
    var Data = JSON.parse(jsonData);
    var point = Data.Command;
    var theta1 = Data.Theta1;
    var theta2 = Data.Theta2;
    var theta3 = Data.Theta3;
    var theta4 = Data.Theta4;
    TableHTMLSet += "<tr style=\"border: 1px solid white;\"><td style=\"border: 1px solid white;\">" + point + "</td><td style=\"border: 1px solid white;\">" + theta1 + "</td><td style=\"border: 1px solid white;\">" + theta2 + "</td><td style=\"border: 1px solid white;\">" + theta3 + "</td><td style=\"border: 1px solid white;\">" + theta4 + "</td></tr>";
    TableHTMLSet += "</tbody></table>";
    document.getElementById('tabledatasetpoint').innerHTML = TableHTMLSet;
}
document.getElementById("btnOn").addEventListener("click", function () {
    var jsonhut = "{'Command':'HUT'}";
    console.log(jsonhut);
    websocket.send(jsonhut);
});

document.getElementById("btnOff").addEventListener("click", function () {
    var jsonnha = "{'Command':'NHA'}";
    console.log(jsonnha);
    websocket.send(jsonnha);
});

function SaveSetPoint(){

    var Px_IK = document.getElementById("setpoint_px").value;
    var Py_IK = document.getElementById("setpoint_py").value;
    var Pz_IK = document.getElementById("setpoint_pz").value;
    var Theta_IK = document.getElementById("setpoint_theta").value;
    // var Theta_IK = -90;

    let Theta1_IK,Theta2_IK,Theta3_IK,Theta4_IK;
    let theta1_IK_rad, theta2_IK_rad, theta3_IK_rad, theta4_IK_rad;
    let anpha = 0, k = 0, E = 0, F = 0, a = 0, b = 0, d = 0, f = 0, var_temp = 0, c23 = 0, s23 = 0, t_rad = 0;

    t_rad = Theta_IK * (Math.PI / 180);
    k = Math.sqrt(Math.pow(Px_IK, 2) + Math.pow(Py_IK, 2));
    theta1_IK_rad = Math.atan2((Py_IK / k), (Px_IK / k));
    Theta1_IK = theta1_IK_rad * (180 / Math.PI);
    Theta1_IK = Math.round(Theta1_IK);

    if (Theta1_IK < -180) {
        Theta1_IK += 360;
    } else if (Theta1_IK > 180) {
        Theta1_IK -= 360;
    }

    E = Px_IK * Math.cos(theta1_IK_rad) + Py_IK * Math.sin(theta1_IK_rad) - L1 - L4 * Math.cos(t_rad);
    F = Pz_IK - d1 - L4 * Math.sin(t_rad);

    a = -2 * L2 * F;
    b = -2 * L2 * E;
    d = Math.pow(L3, 2) - Math.pow(E, 2) - Math.pow(F, 2) - Math.pow(L2, 2);
    f = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    anpha = Math.atan2((-2 * L2 * F) / f, (-2 * L2 * E) / f);

    var_temp = Math.pow(d, 2) / Math.pow(f, 2);
    if (var_temp > 1) var_temp = 1;

    theta2_IK_rad = Math.atan2((-Math.sqrt(1 - var_temp)), d / f) + anpha;
    Theta2_IK = theta2_IK_rad * (180 / Math.PI);
    Theta2_IK = Math.round(Theta2_IK);

    if (Theta2_IK < -180) {
        Theta2_IK += 360;
    } else if (Theta2_IK > 180) {
        Theta2_IK -= 360;
    }

    c23 = (Px_IK * Math.cos(theta1_IK_rad) + Py_IK * Math.sin(theta1_IK_rad) - L1 - L2 * Math.cos(theta2_IK_rad) - L4 * Math.cos(t_rad)) / L3;
    s23 = (Pz_IK - d1 - L2 * Math.sin(theta2_IK_rad) - L4 * Math.sin(t_rad)) / L3;
    theta3_IK_rad = Math.atan2(s23, c23) - theta2_IK_rad;
    Theta3_IK = theta3_IK_rad * (180 / Math.PI);
    Theta3_IK = Math.round(Theta3_IK);

    if (Theta3_IK < -180) {
        Theta3_IK += 360;
    } else if (Theta3_IK > 180) {
        Theta3_IK -= 360;
    }

    theta4_IK_rad = t_rad - theta2_IK_rad - theta3_IK_rad;
    Theta4_IK = theta4_IK_rad * (180 / Math.PI);
    Theta4_IK = Math.round(Theta4_IK);
    if(Theta1_IK > 90 || Theta1_IK < -90 || Theta2_IK > 90 || Theta2_IK < -90 || Theta3_IK > 90 || Theta3_IK < -90 || Theta4_IK > 90 || Theta4_IK < -90) {
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
    else{
        var jsonSetPoint = "{\"Command\": \"SetPoint\", \"Theta1\":\""+ Theta1_IK +"\",\"Theta2\":\""+ Theta2_IK +"\",\"Theta3\":\""+ Theta3_IK +"\",\"Theta4\":\""+ Theta4_IK +"\"}";
        console.log(jsonSetPoint);
        tabelSetPointData(jsonSetPoint);
        websocket.send(jsonSetPoint);

    }
}
function ClearSetPoint(){
    jsonSetPoint = "";
    console.log(jsonSetPoint);
    document.getElementById('tabledatasetpoint').innerHTML = "";
}
function AddSelectPoint() {
    var Px_IK = document.getElementById("selectpoint_px").value;
    var Py_IK = document.getElementById("selectpoint_py").value;
    var Pz_IK = document.getElementById("selectpoint_pz").value;
    var Theta_IK = document.getElementById("selectpoint_theta").value;

    let Theta1_IK,Theta2_IK,Theta3_IK,Theta4_IK;
    let theta1_IK_rad, theta2_IK_rad, theta3_IK_rad, theta4_IK_rad;
    let anpha = 0, k = 0, E = 0, F = 0, a = 0, b = 0, d = 0, f = 0, var_temp = 0, c23 = 0, s23 = 0, t_rad = 0;

    t_rad = Theta_IK * (Math.PI / 180);
    k = Math.sqrt(Math.pow(Px_IK, 2) + Math.pow(Py_IK, 2));
    theta1_IK_rad = Math.atan2((Py_IK / k), (Px_IK / k));
    Theta1_IK = theta1_IK_rad * (180 / Math.PI);
    Theta1_IK = Math.round(Theta1_IK);

    if (Theta1_IK < -180) {
        Theta1_IK += 360;
    } else if (Theta1_IK > 180) {
        Theta1_IK -= 360;
    }

    E = Px_IK * Math.cos(theta1_IK_rad) + Py_IK * Math.sin(theta1_IK_rad) - L1 - L4 * Math.cos(t_rad);
    F = Pz_IK - d1 - L4 * Math.sin(t_rad);

    a = -2 * L2 * F;
    b = -2 * L2 * E;
    d = Math.pow(L3, 2) - Math.pow(E, 2) - Math.pow(F, 2) - Math.pow(L2, 2);
    f = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    anpha = Math.atan2((-2 * L2 * F) / f, (-2 * L2 * E) / f);

    var_temp = Math.pow(d, 2) / Math.pow(f, 2);
    if (var_temp > 1) var_temp = 1;

    theta2_IK_rad = Math.atan2((-Math.sqrt(1 - var_temp)), d / f) + anpha;
    Theta2_IK = theta2_IK_rad * (180 / Math.PI);
    Theta2_IK = Math.round(Theta2_IK);

    if (Theta2_IK < -180) {
        Theta2_IK += 360;
    } else if (Theta2_IK > 180) {
        Theta2_IK -= 360;
    }

    c23 = (Px_IK * Math.cos(theta1_IK_rad) + Py_IK * Math.sin(theta1_IK_rad) - L1 - L2 * Math.cos(theta2_IK_rad) - L4 * Math.cos(t_rad)) / L3;
    s23 = (Pz_IK - d1 - L2 * Math.sin(theta2_IK_rad) - L4 * Math.sin(t_rad)) / L3;
    theta3_IK_rad = Math.atan2(s23, c23) - theta2_IK_rad;
    Theta3_IK = theta3_IK_rad * (180 / Math.PI);
    Theta3_IK = Math.round(Theta3_IK);

    if (Theta3_IK < -180) {
        Theta3_IK += 360;
    } else if (Theta3_IK > 180) {
        Theta3_IK -= 360;
    }

    theta4_IK_rad = t_rad - theta2_IK_rad - theta3_IK_rad;
    Theta4_IK = theta4_IK_rad * (180 / Math.PI);
    Theta4_IK = Math.round(Theta4_IK);

    if( Theta1_IK > 90 || Theta1_IK < -90 || Theta2_IK > 90 || Theta2_IK < -90 || Theta3_IK > 90 || Theta3_IK < -90 || Theta4_IK > 90 || Theta4_IK < -90) {
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
    else {

        const pointData = {
            "Point": app.toString(),
            "Theta1": Theta1_IK,
            "Theta2": Theta2_IK,
            "Theta3": Theta3_IK,
            "Theta4": Theta4_IK
        };

        appData.push(pointData);

        const jsonApp = {
            "Command": "SelectPoint",
            "Data": appData
        };

        const jsonString = JSON.stringify(jsonApp);
        jsonSelectPoint = jsonString;
        console.log(jsonString);
        app++;
        counter++;
        tabelData(jsonString);
    }
}
function SaveSelectPoint(){
    console.log(jsonSelectPoint);
    websocket.send(jsonSelectPoint);
}
function ClearSelectPoint(){
    jsonSelectPoint = "";
    console.log(jsonSelectPoint);
    document.getElementById('tabledata').innerHTML = "";
}
