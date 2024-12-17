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

