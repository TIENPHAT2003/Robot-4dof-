function updateJointPositionFromInput(joint) {
    const inputValue = document.getElementById(joint).value;
    const slider = document.getElementById(`${joint}-slider`);

    if (inputValue >= slider.min && inputValue <= slider.max) {
        document.getElementById(`${joint}-slider`).value = inputValue;
        document.getElementById(`${joint}-value`).textContent = inputValue;
    }
}

function updateJointPositionFromSlider(joint) {
    const sliderValue = document.getElementById(`${joint}-slider`).value;
    document.getElementById(joint).value = sliderValue;
    document.getElementById(`${joint}-value`).textContent = sliderValue;
}
function calculateInverseKinematics(){
    var Px_IK = parseFloat(document.getElementById('pxIK').value);
    var Py_IK = parseFloat(document.getElementById('pyIK').value);
    var Pz_IK = parseFloat(document.getElementById('pzIK').value);
    var Theta_IK = parseFloat(document.getElementById('thetaIK').value);
    if(Px_IK == 0 && Py_IK == 0 && Pz_IK == 0){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
    else {

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
            document.getElementById('theta1IK').value = Theta1_IK;
            document.getElementById('theta2IK').value = Theta2_IK;
            document.getElementById('theta3IK').value = Theta3_IK;
            document.getElementById('theta4IK').value = Theta4_IK;
            
            document.getElementById('px').value = Px_IK;
            document.getElementById('py').value = Py_IK;
            document.getElementById('pz').value = Pz_IK;
            document.getElementById('theta').value = Theta_IK;
        
            document.getElementById('theta1').value = Theta1_IK;
            document.getElementById('theta2').value = Theta2_IK;
            document.getElementById('theta3').value = Theta3_IK;
            document.getElementById('theta4').value = Theta4_IK;
            var jsonIK = "{'Command': 'inversekinematics', 'Theta1':'"+ Theta1_IK +"','Theta2':'"+ Theta2_IK +"','Theta3':'"+ Theta3_IK +"','Theta4':'"+ Theta4_IK +"'}";
            console.log(jsonIK);
            websocket.send(jsonIK);
        }
    }
}
document.getElementById('pxIK').onblur = function(){
    var px = document.getElementById('pxIK').value;
    if(px > 400 || px <  -400 || px == 0){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}
document.getElementById('pyIK').onblur = function(){
    var py = document.getElementById('pyIK').value;
    if(py > 400 || py <  -400 || py == 0){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}
document.getElementById('pzIK').onblur = function(){
    var pz = document.getElementById('pzIK').value;
    if(pz > 400 || pz <  -400 ){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}
document.getElementById('thetaIK').onblur = function(){
    var theta = document.getElementById('thetaIK').value;
    if(theta > 90 || theta <  -90){
        var modal = new bootstrap.Modal(document.getElementById('wrongInputModal'));
        modal.show();
        setTimeout(function () { modal.hide(); }, 1000);
    }
}