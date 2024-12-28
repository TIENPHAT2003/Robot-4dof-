var Pz_IK_SetPoint = 50;
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
document.getElementById("btnsethome").addEventListener("click", function () {
    var jsonnha = "{'Command':'Reset'}";
    console.log(jsonnha);
    websocket.send(jsonnha);
});

