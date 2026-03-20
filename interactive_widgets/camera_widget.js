// Constants (CarMaker 640x480 at 90 FOV)
const FX = 320.0, FY = 320.0;
const CX = 320.0, CY = 240.0;
const IMG_W = 640, IMG_H = 480;

// Elements
const canvasTD = document.getElementById('topdownCanvas');
const ctxTD = canvasTD.getContext('2d');
const canvasImg = document.getElementById('imageCanvas');
const ctxImg = canvasImg.getContext('2d');

const sX1 = document.getElementById('sliderX1');
const sZ1 = document.getElementById('sliderZ1');
const sX2 = document.getElementById('sliderX2');
const sZ2 = document.getElementById('sliderZ2');
const sPx = document.getElementById('sliderPx');

const vX1 = document.getElementById('valX1');
const vZ1 = document.getElementById('valZ1');
const vX2 = document.getElementById('valX2');
const vZ2 = document.getElementById('valZ2');
const vPx = document.getElementById('valPx');

const mathErr = document.getElementById('mathErr');
const resultBox = document.getElementById('resultBox');
const resultVal = document.getElementById('resultVal');
const resultDesc = document.getElementById('resultDesc');

// Drawing utils
function drawGrid(ctx, w, h, step, dark=false) {
    ctx.strokeStyle = dark ? '#151525' : '#222238';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for(let x=0; x<=w; x+=step) { ctx.moveTo(x,0); ctx.lineTo(x,h); }
    for(let y=0; y<=h; y+=step) { ctx.moveTo(0,y); ctx.lineTo(w,y); }
    ctx.stroke();
}

function w2c_td(x, z) {
    return { cx: (x + 10) * (400/20), cy: 400 - (z * (400/30)) };
}

function drawPoint(ctxImg, ctxTD, p0, X, Z, PxErr, color, nameStr) {
    const Y = -0.3; // Fixed height
    let u = null, v = null, inFrame = false;
    
    if (Z > 0.1) {
        u = FX * (X/Z) + CX;
        v = FY * (Y/Z) + CY;
        inFrame = (u >= 0 && u <= IMG_W && v >= 0 && v <= IMG_H);
    }
    
    const err = PxErr * (Z / FX);
    const renderColor = inFrame ? color : '#E24B4A';

    // --- TOP DOWN DRAW ---
    const pt = w2c_td(X, Z);
    ctxTD.strokeStyle = renderColor;
    ctxTD.lineWidth = 1.5;
    ctxTD.setLineDash([2,4]);
    ctxTD.beginPath();
    ctxTD.moveTo(p0.cx, p0.cy); ctxTD.lineTo(pt.cx, pt.cy);
    ctxTD.stroke();
    ctxTD.setLineDash([]);

    ctxTD.fillStyle = renderColor;
    ctxTD.beginPath();
    ctxTD.moveTo(pt.cx, pt.cy-8);
    ctxTD.lineTo(pt.cx-6, pt.cy+6);
    ctxTD.lineTo(pt.cx+6, pt.cy+6);
    ctxTD.fill();

    // Text Label TopDown
    ctxTD.fillStyle = renderColor;
    ctxTD.font = "10px monospace";
    ctxTD.fillText(nameStr, pt.cx + 10, pt.cy);

    // --- IMGE 2D DRAW ---
    if (u !== null) {
        const pu = Math.max(5, Math.min(IMG_W-5, u));
        const pv = Math.max(5, Math.min(IMG_H-5, v));
        const box_w_px = Math.max((0.22 * FX) / Z, 4); 
        const box_h_px = Math.max((0.30 * FY) / Z, 4); 

        ctxImg.strokeStyle = renderColor;
        ctxImg.lineWidth = 0.5;
        ctxImg.beginPath();
        ctxImg.moveTo(0, pv); ctxImg.lineTo(640, pv);
        ctxImg.moveTo(pu, 0); ctxImg.lineTo(pu, 480);
        ctxImg.stroke();

        ctxImg.fillStyle = renderColor;
        ctxImg.beginPath();
        ctxImg.arc(pu, pv, Math.max(box_w_px/4, 2), 0, Math.PI*2);
        ctxImg.fill();
        
        ctxImg.strokeStyle = '#E24B4A'; // Uncertainty always red
        ctxImg.lineWidth = 1.5;
        ctxImg.beginPath();
        ctxImg.rect(pu - (box_w_px/2) - PxErr, pv - box_h_px, box_w_px + (PxErr*2), box_h_px);
        ctxImg.stroke();
        
        ctxImg.fillStyle = renderColor;
        ctxImg.font = "12px monospace";
        ctxImg.fillText(`${nameStr}: ${err.toFixed(2)}m err`, pu + 15, pv - 20);
        ctxImg.fillText(`(${Math.round(u)}, z:${Math.round(Z)}m)`, pu + 15, pv - 5);
    }
    return { inFrame, err };
}


function update() {
    const X1 = parseFloat(sX1.value);
    const Z1 = parseFloat(sZ1.value);
    const X2 = parseFloat(sX2.value);
    const Z2 = parseFloat(sZ2.value);
    const PxErr = parseFloat(sPx.value);

    // Update DOM
    vX1.textContent = X1.toFixed(2) + " m";
    vZ1.textContent = Z1.toFixed(1) + " m";
    vX2.textContent = X2.toFixed(2) + " m";
    vZ2.textContent = Z2.toFixed(1) + " m";
    vPx.textContent = PxErr.toFixed(0) + " px";

    // --- PREP CANVASES ---
    ctxTD.clearRect(0,0,400,400);
    ctxTD.fillStyle = '#0d0d1a';
    ctxTD.fillRect(0,0,400,400);
    drawGrid(ctxTD, 400, 400, 40);

    ctxTD.strokeStyle = '#7F77DD';
    ctxTD.lineWidth = 1;
    ctxTD.setLineDash([5,5]);
    ctxTD.beginPath();
    const p0 = w2c_td(0,0);
    const pl = w2c_td(-30,30);
    const pr = w2c_td(30,30);
    ctxTD.moveTo(p0.cx, p0.cy); ctxTD.lineTo(pl.cx, pl.cy);
    ctxTD.moveTo(p0.cx, p0.cy); ctxTD.lineTo(pr.cx, pr.cy);
    ctxTD.stroke();
    ctxTD.setLineDash([]);
    
    ctxTD.fillStyle = 'rgba(127, 119, 221, 0.05)';
    ctxTD.beginPath();
    ctxTD.moveTo(p0.cx, p0.cy); ctxTD.lineTo(pl.cx, pl.cy); ctxTD.lineTo(pr.cx, pr.cy);
    ctxTD.fill();

    ctxTD.fillStyle = '#EF9F27';
    ctxTD.beginPath();
    ctxTD.arc(p0.cx, p0.cy, 6, 0, Math.PI*2);
    ctxTD.fill();

    ctxImg.clearRect(0,0,640,480);
    ctxImg.fillStyle = '#080812';
    ctxImg.fillRect(0,0,640,480);
    drawGrid(ctxImg, 640, 480, 40, true);

    ctxImg.strokeStyle = '#EF9F27';
    ctxImg.lineWidth = 1.5;
    ctxImg.globalAlpha = 0.5;
    ctxImg.beginPath();
    ctxImg.moveTo(CX-10, CY); ctxImg.lineTo(CX+10, CY);
    ctxImg.moveTo(CX, CY-10); ctxImg.lineTo(CX, CY+10);
    ctxImg.stroke();
    ctxImg.globalAlpha = 1.0;

    // --- DRAW POINTS ---
    // Point 1 (Green)
    const pt1 = drawPoint(ctxImg, ctxTD, p0, X1, Z1, PxErr, '#1D9E75', 'PC1');
    // Point 2 (Purple)
    const pt2 = drawPoint(ctxImg, ctxTD, p0, X2, Z2, PxErr, '#7F77DD', 'PC2');

    const maxErr = Math.max(pt1.err, pt2.err);
    mathErr.textContent = `Max Err: ${maxErr.toFixed(3)}m`;

    // --- VERDICT ---
    resultVal.textContent = maxErr.toFixed(3) + " m";
    if (maxErr <= 0.20) {
        resultBox.className = "result-box pass";
        resultDesc.textContent = "Safe. Both points are within the ≤ 0.20m boundary objective.";
    } else {
        resultBox.className = "result-box";
        resultDesc.textContent = "Unsafe! Exceeds the 0.20m objective. Camera depth gives massive lateral error at range.";
    }
}

sX1.addEventListener('input', update);
sZ1.addEventListener('input', update);
sX2.addEventListener('input', update);
sZ2.addEventListener('input', update);
sPx.addEventListener('input', update);

update();
