#ifndef TRIMATIK_HTML_SIMPLE_H
#define TRIMATIK_HTML_SIMPLE_H

const char indexPageHTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="de-DE">
<head>
<title>Trimatik-MC</title>
<meta charset="utf-8">
<style>
body
{
  background-color: black;
  color: white;
  font-family: Arial;
}
h1
{
  padding: 10px;
  text-align: center;
  border: 2px solid lightgray;
  border-radius: 25px;
  font: italic small-caps bold 30px sans-serif;
  background-color:#E53817;
}
body > div
{
  border: 2px solid lightgray;
  border-radius: 25px;
  font: 16px sans-serif;
  padding: 20px;
  background-color:FireBrick;
}
a:link, a:visited
{
  color: lightgray;
}
a:hover, a:active 
{
  color: white;
}
.sliderDial
{
  height: 25px;
  width: 140px;
  background: #d3d3d3;
  border-radius: 5px;
}
.sliderContainer
{
  float:left;
}
.sliderContainer > div
{
  float:left;
  text-align:center;
  line-height:1.5em;
}
.numberBox
{
  font: inherit;
  width: 6em;
  text-align: right;
}
</style>
</head>

<body>
<div>
  Trimatik-MC:<br>
  <textarea id="msg" name="msgBox" rows="10" cols="56" style="display:block;float:left;background-color:#cccccc">-leer-</textarea>
  <textarea id="gasValuesBox" name="gasValues" rows="10" cols="32" style="display:block;float:right;">-leer-</textarea>
  <div class="sliderContainer">
    <div>
      <label for="rtTag">&#9788;</label><br>
      <input type="range" min="-9" max="8" step="1" value="0" class="sliderDial" id="rtTag"><br>
      <output id="rtTagVal">N</output>
    </div>
  </div>
  <div class="sliderContainer">
    <div>
      <label for="rtNacht">&#9789;</label><br>
      <input type="range" min="-9" max="6" step="1" value="0" class="sliderDial" id="rtNacht"><br>
      <output id="rtNachtVal">N</output><br><br>
    </div>
  </div>
  <div class="sliderContainer">
    <div>
      <label for="tWw">&#128688;</label><br>
      <input type="range" min="30" max="60" step="2" value="44" class="sliderDial" id="tWw"><br>
      <output id="tWwVal">N</output><br><br>
    </div>
  </div>
  <div class="sliderContainer">
    <div>
      <label for="rtNeigung">Neigung</label><br>
      <input type="range" min="0" max="15" step="1" value="8" class="sliderDial" id="rtNeigung"><br>
      <output id="rtNeigungVal">N</output>
    </div>
  </div>
  <div class="sliderContainer">
    <div>
      <label for="rtNiveau">Niveau</label><br>
      <input type="range" min="-12" max="33" step="3" value="0" class="sliderDial" id="rtNiveau"><br>
      <output id="rtNiveauVal">N</output>
    </div>
  </div>
  <div id="svgscroll" style="background-color: Tan; overflow-x: scroll; overflow-y: auto; padding:5px; direction:rtl;clear:left;">
    <svg id="svg" viewBox="-10 -10 3620 340" xmlns="http://www.w3.org/2000/svg" height="340">
      <g id="heizung" style="fill:lightpink;"></g>
      <g id="labels" style="font: normal 18px sans-serif;text-anchor:middle;" fill="gray"></g>
      <g transform="scale(1,-1) translate(0.5,-290.25)" fill="none" stroke="black">
        <g id="bgBrenner" fill="red" fill-opacity="0.1" stroke="none"></g>
        <path id="markWWLadung" stroke="orange" stroke-width="7" d=""><title>WW-Pumpe</title></path>
        <path id="markHKPumpe" stroke="yellow" stroke-width="7" opacity="0.6" d=""><title>HK-Pumpe</title></path>
        <path id="markBrenner" stroke="red" stroke-width="3" d=""><title>Brenner</title></path>
        <path id="divxy" stroke="gray" stroke-width="1" d=""/>
        <path id="axisxy" d="M0,-40V310 M-10,0H3600"/>
        <path id="graphSoll" stroke="magenta" d=""><title>Sollvorlauf</title></path>
        <path id="graphX25" stroke="yellow" opacity="0.6" d=""><title>0x25</title></path>
        <path id="graphX45" stroke="magenta" opacity="0.5" d=""><title>0x45</title></path>-->
        <path id="graphGas" stroke="black"  d=""><title>Leistung [1 Strich = 2,5 kW]</title></path>
        <path id="graphAussen" stroke="green" stroke-width="2" d=""><title>Aussentemperatur</title></path>
        <path id="graphKessel1" stroke="red"  d=""><title>Kesselwasser</title></path>
        <path id="graphWasser" stroke="blue"  d=""><title>Warmwasser</title></path>
      </g>
    </svg>
  </div>
  <p><button id="refresh" class="button">Diagramm aktualisieren</button> -
    <span style="background-color:green">&nbsp;Außentemperatur&nbsp;</span> -
    <span style="background-color:blue">&nbsp;Wassertemperatur&nbsp;</span> -
    <span style="background-color:red">&nbsp;Kesseltemperatur&nbsp;</span> -
    <span style="background-color:magenta">&nbsp;Solltemperatur&nbsp;</span> -
    <span style="background-color:black">&nbsp;Wärmebedarf&nbsp;</span>
    <br>
    <a href='update'>FW-Update</a> - <a href='info.txt'>ESP-Info</a> - <a href='wifiscan.txt'>WLAN-Scan</a> - <a href='logreport.txt'>Log-Report</a> - <a href='messwerte.csv'>Messwerte-Download (CSV)</a></p>
  <p>Status: <span id='status'>undefined</span></p>
</div>

<script>
  var gateway = `${window.location.hostname}:${window.location.port}`;
  var websocket;
  var xZoom = 20;
  var dataGas = [];
  var dataAussen = [];
  var dataKessel = [];
  var dataWasser = [];
  var dataRelais = [];
  var dataProgramm = [];
  var dataStatus = [];
  var lastTS = 0;
  var gasLocked = true;

  const rtTagOut = document.getElementById('rtTagVal');
  const rtNachtOut = document.getElementById('rtNachtVal');
  const tWwOut = document.getElementById('tWwVal');
  const rtNeigungOut = document.getElementById('rtNeigungVal');
  const NeigungTab = [0.2, 0.4, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6 ];
  const rtNiveauOut = document.getElementById('rtNiveauVal');
  const status = document.getElementById('status');

  window.addEventListener('load', onLoad);
  window.addEventListener('visibilitychange', visibilitychange);

  function visibilitychange()  // wird nicht nach einem Page-Load ausgelöst
  {
    if(document.hidden)
    {
      websocket.close();
    }
    else
    {
      if(websocket.readyState == WebSocket.CLOSED) initWebSocket();
      loadParamJSON();
    }
  }
  function initWebSocket()
  {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket('ws://'+gateway+'/ws');
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
    document.getElementById("msg").style.backgroundColor = '#ccffcc';
  }
  function onOpen(event)
  {
    console.log('Connection opened');
    document.getElementById("msg").style.backgroundColor = '#ffffff';
  }
  function onClose(event)
  {
    // falls die Verbdingung geschlossen wurde, nach 2s neu aufbauen
    console.log('Connection closed');
    document.getElementById("msg").style.backgroundColor = '#cccccc';
    if(!document.hidden) setTimeout(initWebSocket, 2000);
  }
  // neue Nachricht vom Server auswerten
  function onMessage(event)
  {
    let obj = document.getElementById('msg');
    obj.value = event.data;
    let msg = event.data.split("\n");
    let relais = parseInt(msg[6].substr(-2),16);
  }
  function sendSettingMsg(event)
  {
    if(websocket.readyState == WebSocket.OPEN)
    {
      websocket.send("Tag:"+rtTagOut.value + "\nNacht:"+rtNachtOut.value + "\nWasser:"+tWwOut.value + "\nNeigung:"+rtNeigungOut.value*10 + "\nNiveau:"+rtNiveauOut.value);
    }
  }
  function loadParamJSON()
  {
    const request = new XMLHttpRequest();
    request.open('GET', 'param.json', true);
    request.onload = function()
      {
        if (request.status === 200)
        {
          var param = JSON.parse(request.responseText);
      
          rtTagOut.value = param.Tag;
          rtNachtOut.value = param.Nacht;
          tWwOut.value = param.Wasser;
          rtNeigungOut.value = param.Neigung/10;
          rtNiveauOut.value = param.Niveau;
          
          var slider = document.getElementById('rtTag');
          slider.value = param.Tag;
          slider = document.getElementById('rtNacht');
          slider.value = param.Nacht;
          slider = document.getElementById('tWw');
          slider.value = (param.Wasser > 30) ? param.Wasser : 30;
          slider = document.getElementById('rtNeigung');
          slider.value = NeigungTab.indexOf(param.Neigung/10);
          slider = document.getElementById('rtNiveau');
          slider.value = param.Niveau;
        }
      };
    request.onerror = function() { console.error('An error occurred fetching the JSON'); };
    request.send();
  }
  function onLoad(event)
  {
    let base = document.createElement('base');
    base.href = 'http://'+gateway;
    document.getElementsByTagName('head')[0].appendChild(base);
  
    let slider = document.getElementById('rtTag');
    rtTagOut.value = slider.value;
    slider.addEventListener ("input", function () { rtTagOut.value = this.value; });
    slider.addEventListener ('change', sendSettingMsg);

    slider = document.getElementById('rtNacht');
    rtNachtOut.value = slider.value;
    slider.addEventListener ("input", function () { rtNachtOut.value = this.value; });
    slider.addEventListener ('change', sendSettingMsg);

    slider = document.getElementById('tWw');
    tWwOut.value = (slider.value > 30) ? slider.value : 0;
    slider.addEventListener ("input", function () { tWwOut.value = (this.value > 30) ? this.value : 0; });
    slider.addEventListener ('change', sendSettingMsg);

    slider = document.getElementById('rtNeigung');
    rtNeigungOut.value = NeigungTab[slider.value];
    slider.addEventListener ("input", function () { rtNeigungOut.value = NeigungTab[this.value]; });
    slider.addEventListener ('change', sendSettingMsg);

    slider = document.getElementById('rtNiveau');
    rtNiveauOut.value = slider.value;
    slider.addEventListener ("input", function () { rtNiveauOut.value = this.value; });
    slider.addEventListener ('change', sendSettingMsg);
  
    initWebSocket();
    document.getElementById('refresh').addEventListener('click', loadChartBIN);
    document.getElementById('svgscroll').addEventListener('wheel', zoomChart);
    document.getElementById('svgscroll').addEventListener('touchmove', touchmoveChart);
    document.getElementById('svgscroll').addEventListener('touchend', touchendChart);
    
    loadParamJSON();
    loadChartBIN();
  }
  function zoomChart(evt)
  {
    evt.preventDefault();
    var xZoomNeu = xZoom;
    if(evt.deltaY>0) xZoomNeu++;
    else if(evt.deltaY<0) xZoomNeu--;
    if(xZoomNeu<1) xZoomNeu=1;
    if(xZoom != xZoomNeu)
    {
      xZoom = xZoomNeu;
      updateChart();
    }
  }
  var lastDist = 0;
  function touchmoveChart(evt)
  {
    var touch1 = evt.touches[0];
    var touch2 = evt.touches[1];

    if (touch1 && touch2)
    {
      evt.preventDefault();

      var dist = touch1.clientX - touch2.clientX;
      if(dist<0) dist = -dist;

      if (!lastDist) lastDist = dist;

      var xZoomNeu = xZoom * (lastDist / dist);
      if(xZoomNeu<1) xZoomNeu=1;
      if(xZoom != xZoomNeu)
      {
        xZoom = xZoomNeu;
        updateChart();
      }
      lastDist = dist;
    }
  }
  function touchendChart()
  {
    lastDist = 0;
  }
  function updateChart()
  {
    if(lastTS==0) return;
    let recs;
    let tStart = dataGas[0][0];
    let xmax = lastTS - tStart; // Dauer der Aufzeichnung in Sekunden
    let ymax = 85;
    let ymin = -20;
    let yspan = ymax - ymin;
    
    // Raster
    xmax = Math.ceil(xmax/60) * 60;   // x auf 60s-Intervalle
    
    // Abbildung: 600px = 300s
    if(xmax<60) xmax=60;
    
    const w = xmax / xZoom;   // 1px/s
    const h = 300;
    var x,y, grf, trace, lbl;
    // Beginn der Aufzeichnung
    const t0 = new Date(tStart*1000); // erster Timestamp
    // der erste Stundenstrich ist bei der nÃ¤chsten vollen Stunde
    const q0 = 15*60 - (t0.getMinutes() % 15)*60 - t0.getSeconds();
    const s0 = 60*60 - t0.getMinutes()*60 - t0.getSeconds();
    
    grf = "";
    lbl = "";
    if(w*60/xmax > 3) // wenn die Minuten-Striche mehr als ein Pixel auseinanderliegen
    {
      for(p=60-t0.getSeconds(); p<=xmax; p=p+60)
      {
        x = Math.round(w * p / xmax);
        grf = grf + "M"+x+",-3V3 ";
      }
    }
    if(w*15*60/xmax > 3)  // wenn die 15-Minuten-Striche mehr als ein Pixel auseinanderliegen
    {
      for(p=q0; p<=xmax; p=p+15*60)
      {
        x = Math.round(w * p / xmax);
        grf = grf + "M"+x+",-5V5 ";
      }
    }
    if(w*3600/xmax > 3) // wenn die Stunden-Striche mehr als ein Pixel auseinanderliegen
    {
      let hh = t0.getHours() + 1;
      for(p=s0; p<=xmax; p=p+3600)
      {
        x = Math.round(w * p / xmax);
        grf = grf + "M"+x+",-5V300 ";
        if(hh == 24) hh = 0;
        lbl += '<text x="'+x+'" y="6">'+hh+'</text>';
        hh++;
      }
    }
    if(h*10/ymax > 3) // wenn die Werte-Striche mehr als ein Pixel auseinanderliegen
    {
      for(p=ymin; p<=ymax; p=p+10)
      {
        y = Math.round(h * p / ymax);
        grf = grf + "M-3,"+y+"H"+w+" ";
        if((p<ymax) && (p%20==0)) lbl += '<text x="16" y="'+(300-y)+'">'+p+'</text>';
      }
    }
    trace = document.getElementById('divxy');
    trace.setAttribute('d',grf);
    trace = document.getElementById('labels');
    trace.innerHTML = lbl;
    
    // [2] Aussentemperatur
    grf = "M";
    recs = dataAussen.length;
    for(p=0; p<recs; p++)
    {
      x = Math.round(w * (dataAussen[p][0]-tStart) / xmax);
      y = (h * (dataAussen[p][1]) / ymax).toFixed(1);
      grf = grf + " "+x+","+y;
    }
    if(dataAussen[recs-1][0] < lastTS)
    {
      grf = grf + " H"+Math.round(w * (lastTS-tStart) / xmax);
    }
    trace = document.getElementById('graphAussen');
    trace.setAttribute('d',grf);

    // [3] Kesseltemperatur
    grf = "M";
    recs = dataKessel.length;
    for(p=0; p<recs; p++)
    {
      x = Math.round(w * (dataKessel[p][0]-tStart) / xmax);
      y = (h * (dataKessel[p][1]) / ymax).toFixed(1);
      grf = grf + " "+x+","+y;
    }
    trace = document.getElementById('graphKessel1');
    trace.setAttribute('d',grf);
    if(dataKessel[recs-1][0] < lastTS)
    {
      grf = grf + " H"+Math.round(w * (lastTS-tStart) / xmax);
    }

    // [4] Warmwassertemperatur
    grf = "M";
    recs = dataWasser.length;
    for(p=0; p<recs; p++)
    {
      x = Math.round(w * (dataWasser[p][0]-tStart) / xmax);
      y = (h * (dataWasser[p][1]) / ymax).toFixed(1);
      grf = grf + " "+x+","+y;
    }
    if(dataWasser[recs-1][0] < lastTS)
    {
      grf = grf + " H"+Math.round(w * (lastTS-tStart) / xmax);
    }
    trace = document.getElementById('graphWasser');
    trace.setAttribute('d',grf);

    // [5] Relais x25 & 1 : Brennerrelais
    grf = "";
    let aktiv = false;
    recs = dataRelais.length;
    for(p=0; p<recs; p++)
    {
      if((dataRelais[p][1] & 1) != 0 && !aktiv)
      {
        aktiv = true;
        x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
        grf = grf + " M"+x+",-1";
      }
      else if((dataRelais[p][1] & 1) == 0 && aktiv)
      {
        aktiv = false;
        x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
        grf = grf + " H"+x;
      }
    }
    if(aktiv)
    {
      x = Math.round(w * (lastTS-tStart) / xmax);
      grf = grf + " H"+x;
    }
    trace = document.getElementById('markBrenner');
    trace.setAttribute('d',grf);

    // [5] Relais x25 & 4 : WW-Ladung
    grf = "";
    aktiv = false;
    recs = dataRelais.length;
    for(p=0; p<recs; p++)
    {
      if((dataRelais[p][1] & 4) != 0 && !aktiv)
      {
        aktiv = true;
        x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
        grf = grf + " M"+x+",-3";
      }
      else if((dataRelais[p][1] & 4) == 0 && aktiv)
      {
        aktiv = false;
        x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
        grf = grf + " H"+x;
      }
    }
    if(aktiv)
    {
      x = Math.round(w * (lastTS-tStart) / xmax);
      grf = grf + " H"+x;
    }
    trace = document.getElementById('markWWLadung');
    trace.setAttribute('d',grf);

    // [5] Relais x25 & 0x40 : HK-Pumpe
    grf = "";
    aktiv = false;
    recs = dataRelais.length;
    for(p=0; p<recs; p++)
    {
      if((dataRelais[p][1] & 0x40) != 0 && !aktiv)
      {
        aktiv = true;
        x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
        grf = grf + " M"+x+",3";
      }
      else if((dataRelais[p][1] & 0x40) == 0 && aktiv)
      {
        aktiv = false;
        x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
        grf = grf + " H"+x;
      }
    }
    if(aktiv)
    {
      x = Math.round(w * (lastTS-tStart) / xmax);
      grf = grf + " H"+x;
    }
    trace = document.getElementById('markHKPumpe');
    trace.setAttribute('d',grf);

    // [5] x25 & 1 : Brenner-Hintergrund aus Gaszähleränderung und Brenner-Relais
    grf = "";
    aktiv = false;
    recs = dataGas.length;
    trace = document.getElementById('bgBrenner');
    while (trace.firstChild) trace.removeChild(trace.firstChild);
    let r = 0;
    let rnum = dataRelais.length;
    for(p=1; p<recs; p++)
    {
      let dt = dataGas[p][0] - dataGas[p-1][0];
      if(dt>0)  // wenn sich der Zählerstand geändert hat (eigentlich jeder Datenpunkt)
      {
        if(!aktiv)  // und noch kein Zählbeginn festgestellt wurde: Startpunkt merken
        {
          aktiv = true;
          lastInc = p;
        }
        else if(dt > 30)  // wenn länger als 20s keine Änderung war, dann laufenden Abschnitt beenden und neuen beginnen
        {
          x = Math.round(w * (dataGas[lastInc][0]-tStart) / xmax);  // Zählbeginn
          let node = document.createElementNS('http://www.w3.org/2000/svg','rect');
          node.setAttribute('x',x);
          node.setAttribute('y',0);
          // längstenfalls könnte der Brennvorgang unerkannt noch 11 Sekunden laufen, oder er wird durch das Relais beendet.
          if(r < rnum)
          {
            dt = dataRelais[r][0] - dataGas[p-1][0];  // die Zeit zwischen BrennerAus und letztem Inkrement
            if(dt > 30) 
            dt = dataGas[p-1][0];  // wenn Brenner noch länger aktiv ist, dann ist die Begrenzung eingeschritten: letzes Inkrement istmaßgeblich
            else 
            dt = dataRelais[r][0];
          }
          else
            dt = dataGas[p-1][0];
          x = Math.round(w * (dt-dataGas[lastInc][0]) / xmax);
          node.setAttribute('width',x);
          node.setAttribute('height',h);
          trace.appendChild(node);
          lastInc = p;
        }
        else  // Gas läuft, den Zeitpunkt nach dataGas[p][0] finden, mit dataRelais[r][1] & 1 == 0 : nächster Brennerabschaltzeitpunkt
        {
          while(r < rnum && dataRelais[r][0] < dataGas[p][0]) r++;
          // dataRelais[r] ist nach dataGas
          while(r < rnum && dataRelais[r][1] & 1) r++;
          // dataRelais[r] ist nun "BrennerAus"
        }
      }
    }
    if(aktiv)
    {
      x = Math.round(w * (dataGas[lastInc][0]-tStart) / xmax);  // Zählbeginn
      let node = document.createElementNS('http://www.w3.org/2000/svg','rect');
      node.setAttribute('x',x);
      node.setAttribute('y',0);
      // es kam kein weiterer Zählimpuls, an dem wir uns orientieren könnten. D.h. es wird das Aufzeichnungsende herangezogen, um das Ende des Bereichs abzugrenzen
      // längstenfalls könnte der Brennvorgang unerkannt noch 11 Sekunden laufen, oder er wird durch das Relais beendet.
      dt = lastTS;
      if(r < rnum)
      {
        if(dataRelais[r][0] < dt) 
        dt = dataRelais[r][0];
      }
      if(dataGas[p-1][0] + 30 < dt) 
        dt = dataGas[p-1][0];
      
      x = Math.round(w * (dt-dataGas[lastInc][0]) / xmax);
      node.setAttribute('width',x);
      node.setAttribute('height',h);
      trace.appendChild(node);
      lastInc = p;
    }
  
    // [2] Aussentemperatur
    // [6] x45 : Schaltuhr
    const rtTag = 20+parseInt(rtTagOut.value);
    const rtNacht = 14+parseInt(rtNachtOut.value);
    const rtNeigung = parseFloat(rtNeigungOut.value);
    const rtNiveau = parseInt(rtNiveauOut.value);
    grf = "M";
    p = 0;
    recs = dataAussen.length;
    r = 0;
    rnum = dataProgramm.length;
    let mode = dataProgramm[0][1] & 0x11;
    let rt = (mode!=0) ? rtTag : rtNacht;
    datOld = -100;
    while(r<rnum)
    {
      let dat;
      while((r<rnum) && ((dataProgramm[r][1] & 0x11) == mode)) r++;
      while(p<recs && (r==rnum || dataAussen[p][0]<dataProgramm[r][0]))
      {
        dat = rt + rtNiveau - rtNeigung * ( 1.7 * (dataAussen[p][1]-rt) + 0.018 * (dataAussen[p][1]-rt)**2 );
        if(dat > 0) dat = dat.toFixed(2);
        else        dat = 0;
        if(datOld != dat)
        {
            x = Math.round(w * (dataAussen[p][0]-tStart) / xmax);
            y = (h * (dat) / ymax).toFixed(1);
            grf = grf+x+","+y + " ";
          datOld = dat;
        }
        p++;
      }
      if(r==rnum) break;
      mode = (dataProgramm[r][1] & 0x11);
      rt = (mode!=0) ? rtTag : rtNacht;
      dat = rt + rtNiveau - rtNeigung * ( 1.7 * (dataAussen[p-1][1]-rt) + 0.018 * (dataAussen[p-1][1]-rt)**2 );
      if(dat > 0) dat = dat.toFixed(2);
      else        dat = 0;
      if(datOld != dat)
      {
        x = Math.round(w * (dataProgramm[r][0]-tStart) / xmax);
        grf = grf+x+","+y + " ";
        y = (h * (dat) / ymax).toFixed(1);
        grf = grf + " V"+y+" L";
        datOld = dat;
      }
    }
    if(dataAussen[recs-1][0] < lastTS)
    {
      grf = grf + Math.round(w * (lastTS-tStart) / xmax)+","+y;
    }
    trace = document.getElementById('graphSoll');
    trace.setAttribute('d',grf);

    // [5] x25 & 1 : Brenner => Verbrauch Gas/Taktintervall
    grf = "M";
    p = 0;
    recs = dataRelais.length;
    r = 0;
    rnum = dataGas.length;
    while(p<recs && (dataRelais[p][1] & 1) == 0) p++; // p zeigt auf den Brenner-Einschaltpunkt
    while(p<recs)
    {
      while(r<rnum && (dataGas[r][0] < dataRelais[p][0])) r++; // r zeigt auf das erste Inkrement des Gaszählers nach dem Brennerstart
      let dt = dataRelais[p][0];
      while(p<recs && (dataRelais[p][1] & 1) != 0) p++; // p zeigt auf den Brenner-Ausschaltpunkt
      while(p<recs && (dataRelais[p][1] & 1) == 0) p++; // p zeigt auf den Brenner-Einschaltpunkt
      if(p<recs)
      {
        x = Math.round(w * (dt-tStart) / xmax);
        dt = dataRelais[p][0] - dt;
        if(r<rnum)
        {
          dv = dataGas[r][1];  // Zählerstand kurz nach nächstem Brennerstart
          while(r<rnum && (dataGas[r][0] < dataRelais[p][0])) r++; // r zeigt auf das erste Inkrement des Gaszählers nach dem nächsten Brennerstart
          dv = dataGas[r-1][1] - dv;
          y = (h * (4*3600*11*dv/dt) / ymax).toFixed(1);  // in kWh/Stunde
          grf = grf + " "+x+","+y;
          x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
          grf = grf + " "+x+","+y;
        }
        else
        {
          grf = grf + " "+x+",0";
          x = Math.round(w * (dataRelais[p][0]-tStart) / xmax);
          grf = grf + " "+x+",0";
        }
      }
    }
    trace = document.getElementById('graphGas');
    trace.setAttribute('d',grf);

    trace = document.getElementById('svg');
    trace.viewBox.baseVal.width = w+20;
    
    // x45 Schaltuhr [6] / Gaszählerstand [1]: bei jeder Änderung von Schaltuhr wird der Zählerstand protokolliert
    grf = "";
    recs = dataProgramm.length;
    r = 0;
    rnum = dataGas.length;
    for(p=1; p<recs; p++)
    {
      while(r<rnum && (dataGas[r][0]<dataProgramm[p][0])) r++; // r ist der Gaszählerstand nach dem Umschalten
      let d = new Date(dataProgramm[p][0]*1000);
      if(grf==="") grf = d.toLocaleDateString() + ' ' + d.toLocaleTimeString() + ': ' + dataGas[r-1][1];
      else grf = grf + '\n' + d.toLocaleDateString() + ' ' + d.toLocaleTimeString() + ': ' + dataGas[r-1][1];
    }
    trace = document.getElementById('gasValuesBox');
    trace.value = grf;
    trace.selectionStart = grf.length;

    trace = document.getElementById('svg');
    trace.viewBox.baseVal.width = w+20;
  }

  function loadChartBIN()
  {
    document.getElementById("refresh").disabled = true;
    const request = new XMLHttpRequest();
    request.addEventListener("progress", function(e)
      {
        status.innerHTML = e.loaded + ' geladen.';
      });
    request.addEventListener('error', function(e)
      {
        status.innerHTML = 'Nur ' + e.loaded + ' geladen, dann Fehler.';
      });
    let since;
    if(lastTS > 0) since = lastTS-1; else since = 0;
    request.open('GET', 'messwerte.bin?since='+since, true);
    request.responseType = "arraybuffer";
    request.onload = (event) =>
      {
        if (request.status === 200)
        {
          const arrayBuffer = request.response;
          if (arrayBuffer)
          {
            let len = arrayBuffer.byteLength;
            if(len>11)
            {
              const dv = new DataView(arrayBuffer);
              // Startwerte
              let ts = 0;
              let Gas = 0;      // 0x01
              let Aussen = 0;   // 0x31
              let Kessel = 0;   // 0x3a
              let Wasser = 0;   // 0x35
              let Relais = 0;   // 0x25
              let Programm = 0; // 0x45
              let Status = 0;   // 0x2f
              let Tag = 0;      // 0x7f
              ts = dv.getUint32(0,true); // ts, littleEndian=true
              Gas = dv.getUint32(4,true); // Gas
              skip = false;
              if(lastTS==0) dataGas.push([ts, Gas/100]);
              else skip = true;    // bei einem Update wird der Init-Datensatz nicht aufgezeichnet
              let idx = 8;
              while(idx < len)
              {
                let dt = dv.getInt16(idx, true);
                idx+=2;
                let id = dv.getUint8(idx);
                idx++;
  
                if(dt < 0)
                {
                  dat = new Date(ts*1000);
                  alert(ts +':'+ dt + ' <> ' + dv.getUint16(idx, true)+' : ' +dv.getUint8(idx+2)+'<='+dv.getUint8(idx+3) + '\n' + dat);
                }
                if(dt!=0)
                {
                  ts += dt;
                  skip = false;
                }
        
                switch(id)
                {
                  case 0x01: // 1 : Gas
                    Gas++;
                    if(!skip) dataGas.push([ts, Gas/100]);
                  break;
                    case 0x31: // 2 : Aussen
                    Aussen=dv.getInt8(idx)/4;
                    if(!skip) dataAussen.push([ts, Aussen]);
                  break;
                    case 0x3a: // 3 : Kessel
                    Kessel=dv.getUint8(idx)/2;
                    if(!skip) dataKessel.push([ts, Kessel]);
                  break;
                    case 0x35: // 4 : Wasser
                    Wasser=dv.getUint8(idx)/2;
                    if(!skip) dataWasser.push([ts, Wasser]);
                  break;
                    case 0x25: // 5 : Relais
                    Relais=dv.getUint8(idx);
                    if(!skip) dataRelais.push([ts, Relais]);
                  break;
                    case 0x45: // 6 : Programm
                    Programm=dv.getUint8(idx);
                    if(!skip) dataProgramm.push([ts, Programm]);
                  break;
                    case 0x2f: // 8 : Status
                    Status=dv.getUint8(idx);
                    if(!skip) dataStatus.push([ts, Status+Tag]);
                  break;
                    case 0x7f: // 11 : Tag
                    Tag=dv.getUint8(idx);
                    if(!skip) dataStatus.push([ts, Status+Tag]);
                  break;
                }
                idx++;
              }
              if(lastTS != ts)
              {
                lastTS = ts;
                updateChart();
              }
              status.innerHTML = "Letzte Aktualisierung " + new Date().toLocaleTimeString();
            }
          }
        }
        else
        {
          status.innerHTML = "Status " + request.status + " bei Anfrage " + new Date().toLocaleTimeString();
        }
        document.getElementById("refresh").disabled = false;
      };
    request.onerror = function()
      {
        console.error('An error occurred fetching the CSV'); 
        status.innerHTML = "Fehler bei Anfrage " + new Date().toLocaleTimeString();
        document.getElementById("refresh").disabled = false;
      };
    request.send();
    status.innerHTML = "Messwerte angefragt...";
    document.getElementById("refresh").disabled = true;
  }
</script>
</body></html>
)rawliteral";

#endif
