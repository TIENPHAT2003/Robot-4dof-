let counter = 0; 
let selectedPoint = null; 
var jsonSetPoint = ""; 
var jsonSelectPoint = ""; 
var appData = [];
var jsonData = ""; 
var app = 0;
let jsonApp = `{"Command": "SelectPoint", "Data": [`;
const gridContainer = document.getElementById('grid-container');
const gridSize = 800; 
const pointsPerSide = 11; 
const spacing = gridSize / (pointsPerSide - 1);

for (let col = 0; col < pointsPerSide; col++) {
    for (let row = 0; row < pointsPerSide; row++) {
        const point = document.createElement('div');
        point.classList.add('point');
        point.style.left = `${gridSize - row * spacing}px`; 
        point.style.top = `${col * spacing}px`;
        point.setAttribute('data-row', row); 
        point.setAttribute('data-col', col); 
        point.addEventListener('click', () => selectPoint(point));
        gridContainer.appendChild(point);
    } 
}
for (let i = 0; i < pointsPerSide; i++) {
    const xValue = (100 + i * -20);
    const yValue = (100 + i * 20);  

    const xLabel = document.createElement('div');
    xLabel.classList.add('axis-label', 'x-label');
    xLabel.style.left = `${i * spacing}px`;
    xLabel.textContent = xValue;
    gridContainer.appendChild(xLabel);

    const yLabel = document.createElement('div');
    yLabel.classList.add('axis-label', 'y-label');
    yLabel.style.top = `${gridSize - i * spacing}px`; 
    yLabel.textContent = yValue;
    gridContainer.appendChild(yLabel);
}

function selectPoint(point) {
    selectedPoint = point; 
    point.classList.add('selected');

    const row = point.getAttribute('data-row');
    const col = point.getAttribute('data-col');
    
    const xValue = 100 + (10 - col) * 20; 
    const yValue = -100 + row * 20; 

    console.log(`Selected point coordinates: (${xValue}, ${yValue})`);
}

function setPoint() {
    if (selectedPoint) {
        calculateInverseKinematicsSetPoint(selectedPoint);
    } else {
        console.log("No point selected."); 
    }
}

document.getElementById('buttonSetPoint').addEventListener('click', setPoint);
document.getElementById('buttonSelectPoint').addEventListener('click', () => {
    if (selectedPoint) {
        calculateInverseKinematicsSelectPoint(selectedPoint);
    } else {
        console.log("No point selected."); 
    }
});

function calculateInverseKinematicsSetPoint(point){
    const row = point.getAttribute('data-row');
    const col = point.getAttribute('data-col');
    
    const xValue = 100 + (10 - col) * 20 ; 
    const yValue = -100 + row * 20; 


    var Px_IK = parseFloat(`${xValue}`);
    var Py_IK = parseFloat(`${yValue}`);
    var Pz_IK = 50;
    var Theta_IK = -90;

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
        selectedPoint.textContent = "SetPoint"; 
        selectedPoint.classList.add('setpoint'); 

        var jsonSetPoint = "{'Command': 'SetPoint', 'Theta1':'"+ Theta1_IK +"','Theta2':'"+ Theta2_IK +"','Theta3':'"+ Theta3_IK +"','Theta4':'"+ Theta4_IK +"'}";
        console.log(jsonSetPoint);
        websocket.send(jsonSetPoint);

    }
   
}
function calculateInverseKinematicsSelectPoint(point) {
    const row = point.getAttribute('data-row');
    const col = point.getAttribute('data-col');
    
    const xValue = 100 + (10 - col) * 20 ; 
    const yValue = -100 + row * 20 ; 

    var Pz_IK = 26;
    var Px_IK = parseFloat(`${xValue}`);
    var Py_IK = parseFloat(`${yValue}`);

    if (Px_IK > 220 || (Px_IK === 220 && (Py_IK === 100 || Py_IK === -100))) {
        Pz_IK = 16;
    }
    
    var Theta_IK = -90;

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
        selectedPoint.textContent = counter + 1;
        selectedPoint.classList.add('selected');

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
    }
}

document.getElementById('buttonStartAuto').addEventListener('click', () => {
    console.log(jsonSelectPoint);
    websocket.send(jsonSelectPoint);

})


