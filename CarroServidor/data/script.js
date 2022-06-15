
function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('lec_Boton', function(e) {
    console.log("lec_Boton", e.data);
    document.getElementById("Boton").innerHTML = e.data;
  }, false);

  source.addEventListener('lectura_calibracion', function(e) {
    console.log("lectura_calibracion", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("BlancoMaxF").innerHTML = obj.BlancoMaxF;
    document.getElementById("BlancoMaxT").innerHTML = obj.BlancoMaxT;
    document.getElementById("NegroMinF").innerHTML = obj.NegroMinF;
    document.getElementById("NegroMinT").innerHTML = obj.NegroMinT;
    document.getElementById("GrisMinF").innerHTML = obj.GrisMinF;
    document.getElementById("GrisMinT").innerHTML = obj.GrisMinT;
    document.getElementById("GrisMaxF").innerHTML = obj.GrisMaxF;
    document.getElementById("GrisMaxT").innerHTML = obj.GrisMaxT;
  }, false);

  source.addEventListener('lectura_estadoR1', function(e) {
    console.log("lectura_estadoR1", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("Dato_char").innerHTML = obj.Dato_char;
    document.getElementById("Dato_int").innerHTML = obj.Dato_int;
    //document.getElementById("Dato_float").innerHTML = obj.Dato_float;
    //document.getElementById("Dato_bool").innerHTML = obj.Dato_bool;
    document.getElementById("ColorFront").innerHTML = obj.ColorFront;
    document.getElementById("ColorTras").innerHTML = obj.ColorTras;
    document.getElementById("var").innerHTML = obj.var;    
    document.getElementById("Encontrado").innerHTML = obj.Encontrado;
    document.getElementById("Encontrado2").innerHTML = obj.Encontrado2;
  }, false);

  source.addEventListener('lectura_estadoR2', function(e) {
    console.log("lectura_estadoR2", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("Dato_charR2").innerHTML = obj.Dato_charR2;
    document.getElementById("Dato_intR2").innerHTML = obj.Dato_intR2;
    //document.getElementById("Dato_floatR2").innerHTML = obj.Dato_floatR2;
    //document.getElementById("Dato_boolR2").innerHTML = obj.Dato_boolR2;
    document.getElementById("ColorFrontR2").innerHTML = obj.ColorFrontR2;
    document.getElementById("ColorTrasR2").innerHTML = obj.ColorTrasR2;
    document.getElementById("varR2").innerHTML = obj.varR2;    
    document.getElementById("EncontradoR2").innerHTML = obj.EncontradoR2;   
  }, false);

  source.addEventListener('lectura_estadoPOS3x3', function(e) {
    console.log("lectura_estadoPOS3x3", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("a1f").innerHTML = obj.a1f;
    document.getElementById("a1t").innerHTML = obj.a1t;
    document.getElementById("a2f").innerHTML = obj.a2f;
    document.getElementById("a2t").innerHTML = obj.a2t;
    document.getElementById("a3f").innerHTML = obj.a3f;
    document.getElementById("a3t").innerHTML = obj.a3t;
    document.getElementById("b1f").innerHTML = obj.b1f;
    document.getElementById("b1t").innerHTML = obj.b1t;
    document.getElementById("b2f").innerHTML = obj.b2f;
    document.getElementById("b2t").innerHTML = obj.b2t;
    document.getElementById("b3f").innerHTML = obj.b3f;
    document.getElementById("b3t").innerHTML = obj.b3t;
    document.getElementById("c1f").innerHTML = obj.c1f;
    document.getElementById("c1t").innerHTML = obj.c1t;
    document.getElementById("c2f").innerHTML = obj.c2f;
    document.getElementById("c2t").innerHTML = obj.c2t;
    document.getElementById("c3f").innerHTML = obj.c3f;
    document.getElementById("c3t").innerHTML = obj.c3t;
  }, false);

  source.addEventListener('lectura_estadoRobotuno', function(e) {
    console.log("lectura_estadoRobotuno", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("a1f").innerHTML = obj.a1f;
    document.getElementById("a1t").innerHTML = obj.a1t;
    document.getElementById("a2f").innerHTML = obj.a2f;
    document.getElementById("a2t").innerHTML = obj.a2t;
    document.getElementById("a3f").innerHTML = obj.a3f;
    document.getElementById("a3t").innerHTML = obj.a3t;
    document.getElementById("a4f").innerHTML = obj.a4f;
    document.getElementById("a4t").innerHTML = obj.a4t;
    document.getElementById("b4f").innerHTML = obj.b4f;
    document.getElementById("b4t").innerHTML = obj.b4t;
    document.getElementById("b3f").innerHTML = obj.b3f;
    document.getElementById("b3t").innerHTML = obj.b3t;
    document.getElementById("b2f").innerHTML = obj.b2f;
    document.getElementById("b2t").innerHTML = obj.b2t;
    document.getElementById("b1f").innerHTML = obj.b1f;
    document.getElementById("b1t").innerHTML = obj.b1t;

    document.getElementById("d4f").innerHTML = obj.d4f;
    document.getElementById("d4t").innerHTML = obj.d4t;
    document.getElementById("d3f").innerHTML = obj.d3f;
    document.getElementById("d3t").innerHTML = obj.d3t;
    document.getElementById("d2f").innerHTML = obj.d2f;
    document.getElementById("d2t").innerHTML = obj.d2t;
    document.getElementById("d1f").innerHTML = obj.d1f;
    document.getElementById("d1t").innerHTML = obj.d1t;
    document.getElementById("c1f").innerHTML = obj.c1f;
    document.getElementById("c1t").innerHTML = obj.c1t;
    document.getElementById("c2f").innerHTML = obj.c2f;
    document.getElementById("c2t").innerHTML = obj.c2t;
    document.getElementById("c3f").innerHTML = obj.c3f;
    document.getElementById("c3t").innerHTML = obj.c3t;
    document.getElementById("c4f").innerHTML = obj.c4f;
    document.getElementById("c4t").innerHTML = obj.c4t;
  }, false);

  /*source.addEventListener('lectura_estadoRobotdos', function(e) {
    console.log("lectura_estadoRobotdos", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("d4f").innerHTML = obj.d4f;
    document.getElementById("d4t").innerHTML = obj.d4t;
    document.getElementById("d3f").innerHTML = obj.d3f;
    document.getElementById("d3t").innerHTML = obj.d3t;
    document.getElementById("d2f").innerHTML = obj.d2f;
    document.getElementById("d2t").innerHTML = obj.d2t;
    document.getElementById("d1f").innerHTML = obj.d1f;
    document.getElementById("d1t").innerHTML = obj.d1t;
    document.getElementById("c1f").innerHTML = obj.c1f;
    document.getElementById("c1t").innerHTML = obj.c1t;
    document.getElementById("c2f").innerHTML = obj.c2f;
    document.getElementById("c2t").innerHTML = obj.c2t;
    document.getElementById("c3f").innerHTML = obj.c3f;
    document.getElementById("c3t").innerHTML = obj.c3t;
    document.getElementById("c4f").innerHTML = obj.c4f;
    document.getElementById("c4t").innerHTML = obj.c4t;    
    document.getElementById("b4f").innerHTML = obj.b4f;
    document.getElementById("b4t").innerHTML = obj.b4t;
    document.getElementById("b3f").innerHTML = obj.b3f;
    document.getElementById("b3t").innerHTML = obj.b3t;
    document.getElementById("b2f").innerHTML = obj.b2f;
    document.getElementById("b2t").innerHTML = obj.b2t;
    document.getElementById("b1f").innerHTML = obj.b1f;
    document.getElementById("b1t").innerHTML = obj.b1t;
    document.getElementById("a1f").innerHTML = obj.a1f;
    document.getElementById("a1t").innerHTML = obj.a1t;
    document.getElementById("a2f").innerHTML = obj.a2f;
    document.getElementById("a2t").innerHTML = obj.a2t;
    document.getElementById("a3f").innerHTML = obj.a3f;
    document.getElementById("a3t").innerHTML = obj.a3t;
    document.getElementById("a4f").innerHTML = obj.a4f;
    document.getElementById("a4t").innerHTML = obj.a4t;
  }, false); */
}

function botonPosition(element){
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/"+element.id, true);
  console.log(element.id);
  xhr.send();
}