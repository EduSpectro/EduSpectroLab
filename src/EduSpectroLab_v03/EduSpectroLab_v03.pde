import processing.serial.*;
import javax.swing.*; 
import java.io.*;

// ==========================================
// 1. グローバル変数と座標定数
// ==========================================
processing.serial.Serial myPort; 

// センサー情報と波長校正係数
String sensorSerial = "18G00823"; // デフォルト
float A0 = 2.996919681e+02;
float B1 = 2.726331830e+00;
float B2 = -1.231431793e-03;
float B3 = -7.612678129e-06;
float B4 = 1.000508775e-08;
float B5 = 3.403704330e-12;

// --- UI配置 定数 ---
final int PX = 960;         
final int BY_AUTO = 30;     
final int BY_MODE = 120;    
final int BY_INTV = 285;    
final int BY_SAMP = 420;    
final int BY_EXPO = 490;    
final int BY_SMOT = 605;    
final int BY_SAVE = 590;    

float currentYMax = 65535, targetYMax = 65535;
boolean portSelected = false, isMeasuring = false;
boolean autoMode = true, autoScale = true; 
int mode = 0; // 0:EMISS, 1:TRANS, 2:ABS
boolean hasDark = false, hasRef = false;
int smoothingSize = 5;

//float exposureS = 0.5, lastSentExposureS = 0.5; 
float exposureS = 0.1, lastSentExposureS = 0.1; 
boolean exposureChanged = false; 
long lastReceiveMillis = 0;
boolean commError = false;

int totalReadPixels = 1149, dummyOffset = 0, activePixelCount = 1149;
int totalPacketSize = 4 + (totalReadPixels * 2) + 4;

int samplingTarget = 1, samplingCounter = 0;
float[] samplingAccumulator = new float[activePixelCount];
boolean isMeasureSequence = false, isDarkCalibSequence = false, isRefCalibSequence = false;

boolean intervalActive = false, pendingFinalize = false;
int intervalStepS = 10, intervalTargetCount = 5, intervalCurrentCount = 0;
long lastIntervalTriggerMillis = 0;
String intervalSessionName = "", intervalSessionFolder = "";
ArrayList<float[]> intervalHistory = new ArrayList<float[]>(); 
String statusMsg = ""; 

int[] activeData = new int[activePixelCount]; 
int[] darkBuffer = new int[activePixelCount];
int[] refBuffer  = new int[activePixelCount];
float[] processedData = new float[activePixelCount]; 

float[] overlayBuffer = new float[activePixelCount];
boolean hasOverlay = false;

String CONFIG_DIR = "system_config_do_not_delete";
String DATA_DIR = "SavedMeasurements";

// 波長変換関数
float getWavelength(int i) {
  float pix = 1.0 + (i / 1148.0) * (288.0 - 1.0);
  return A0 + B1*pix + B2*pow(pix, 2) + B3*pow(pix, 3) + B4*pow(pix, 4) + B5*pow(pix, 5);
}

// ==========================================
// 2. セットアップ & ドロー
// ==========================================

void setup() {
  size(1200, 720);
  textFont(createFont("SansSerif", 14));
  File d1 = new File(sketchPath(CONFIG_DIR)); if (!d1.exists()) d1.mkdirs();
  File d2 = new File(sketchPath(DATA_DIR)); if (!d2.exists()) d2.mkdirs();
  loadCalibration();
}

void draw() {
  background(245); 
  if (!portSelected) { drawPortSelector(); return; }

  receiveSerial();
  checkCommStatus(); 
  
  if (!isMeasuring && !commError) {
    long waitTime = int(lastSentExposureS * 1000) + 100;
    if (millis() - lastReceiveMillis > waitTime) {
      if (autoMode || isMeasureSequence || isDarkCalibSequence || isRefCalibSequence) sendRequest();
      else if (intervalActive && millis() - lastIntervalTriggerMillis > intervalStepS * 1000) { isMeasureSequence = true; sendRequest(); }
    }
  }
  
  processData();
  updateAutoScale();
  drawGraphArea();
  drawControlInterface();
  
  if (pendingFinalize) {
    pendingFinalize = false;
    SwingUtilities.invokeLater(new Runnable() { public void run() { finalizeInterval(); } });
  }
}

// ==========================================
// 3. 通信・演算ロジック
// ==========================================

void receiveSerial() {
  if (myPort == null) return;
  while (myPort.available() >= totalPacketSize) {
    if (myPort.read() != 0x00) continue; 
    if (myPort.available() < 3) return; 
    int b2 = myPort.read(); int b3 = myPort.read(); int b4 = myPort.read();
    if (b2 == 0x00 && b3 == 0xFF && b4 == 0xFF) {
      byte[] rawData = new byte[totalReadPixels * 2];
      myPort.readBytes(rawData);
      for (int i = 0; i < activePixelCount; i++) {
        int rI = (i + dummyOffset) * 2;
        samplingAccumulator[i] += (((rawData[rI+1] & 0xFF) << 8) | (rawData[rI] & 0xFF));
      }
      myPort.readBytes(new byte[4]); 
      samplingCounter++; isMeasuring = false; lastReceiveMillis = millis(); 
      if (samplingCounter >= samplingTarget) {
        int[] avgResult = new int[activePixelCount];
        for (int i=0; i<activePixelCount; i++) avgResult[i] = int(samplingAccumulator[i]/samplingTarget);
        if (isDarkCalibSequence) { arrayCopy(avgResult, darkBuffer); arrayCopy(avgResult, activeData); hasDark = true; isDarkCalibSequence = false; statusMsg = "Dark Calibrated."; }
        else if (isRefCalibSequence) { arrayCopy(avgResult, refBuffer); arrayCopy(avgResult, activeData); hasRef = true; isRefCalibSequence = false; statusMsg = "Ref Stored."; }
        else arrayCopy(avgResult, activeData);
        if (intervalActive && isMeasureSequence) {
          intervalCurrentCount++; processData(); 
          float[] hCopy = new float[activePixelCount]; arrayCopy(processedData, hCopy); intervalHistory.add(hCopy);
          saveCSV(intervalSessionFolder + "/" + intervalSessionName + "_n" + nf(intervalCurrentCount, 3), "Auto");
          lastIntervalTriggerMillis = millis();
          if (intervalCurrentCount >= intervalTargetCount) { intervalActive = false; pendingFinalize = true; }
        }
        samplingCounter = 0; isMeasureSequence = false; 
        for(int i=0; i<activePixelCount; i++) samplingAccumulator[i] = 0;
      }
      break; 
    }
  }
}

void sendRequest() { 
  if (myPort != null) {
    if (exposureChanged) { myPort.write("exp:" + int(exposureS * 1000) + "\n"); exposureChanged = false; }
    lastSentExposureS = exposureS;
    if (myPort.available() > totalPacketSize) myPort.clear();
    myPort.write("send\n"); isMeasuring = true; 
  } 
}

void processData() {
  for (int i = 0; i < activePixelCount; i++) {
    float val = activeData[i] - (hasDark ? darkBuffer[i] : 0);
    val = max(val, 0);
    if (mode == 0) processedData[i] = val; 
    else {
      float ref = refBuffer[i] - (hasDark ? darkBuffer[i] : 0);
      float t = val / max(ref, 100);
      if (mode == 1) processedData[i] = t * 100.0; 
      else processedData[i] = -log(max(t, 0.001))/log(10); 
    }
  }
  if (smoothingSize > 0) {
    float[] temp = new float[activePixelCount]; arrayCopy(processedData, temp);
    for (int i=smoothingSize; i<activePixelCount-smoothingSize; i++) {
      float s=0; for(int j=-smoothingSize; j<=smoothingSize; j++) s+=processedData[i+j];
      temp[i] = s / (smoothingSize*2+1);
    }
    processedData = temp;
  }
}

// 【修正】現在のデータとオーバーレイデータの両方を考慮したオートスケール
void updateAutoScale() {
  if (!autoScale) { targetYMax = (mode == 0) ? 65535 : (mode == 1) ? 110 : 2.0; }
  else {
    float maxV = 0.01;
    
    // 基準光の強さを計算（不適切領域判定用）
    float maxRefCount = 0;
    if (hasRef) { for (int val : refBuffer) if (val > maxRefCount) maxRefCount = val; }
    float lowRefThreshold = maxRefCount * 0.05;

    for (int i = 0; i < activePixelCount; i++) {
      // 基準光が十分なエリア、またはEMISSモードの場合のみ計算対象にする
      boolean isValid = (mode == 0) || (mode > 0 && hasRef && (refBuffer[i] - (hasDark ? darkBuffer[i] : 0)) >= lowRefThreshold);
      
      if (isValid) {
        if (processedData[i] > maxV) maxV = processedData[i];
        if (hasOverlay && overlayBuffer[i] > maxV) maxV = overlayBuffer[i];
      }
    }
    
    targetYMax = maxV * 1.2;
    if (mode == 0 && targetYMax > 65535) targetYMax = 65535;
  }
  currentYMax += (targetYMax - currentYMax) * 0.1;
}

// ==========================================
// 4. UI 描画
// ==========================================

void drawGraphArea() {
  int xO = 80, yO = 55, gW = 850, gH = 485; 
  fill(255); stroke(180); strokeWeight(1); rect(xO, yO, gW, gH);
  if (commError) { fill(255, 100, 100, 50); rect(xO, yO, gW, gH); fill(150, 0, 0); textAlign(CENTER, CENTER); textSize(24); text("Waiting for Response...", xO + gW/2, yO + gH/2); }
  
  // 低光量エリア警告表示 (TRANS/ABSモード時)
  if (mode > 0 && hasRef) {
    float maxRef = 0;
    for (int val : refBuffer) if (val > maxRef) maxRef = val;
    float lowRefThreshold = maxRef * 0.05; 
    noStroke(); fill(255, 180, 180, 80); 
    for (int i = 0; i < activePixelCount; i++) {
      if ((refBuffer[i] - (hasDark ? darkBuffer[i] : 0)) < lowRefThreshold) {
        float x = map(i, 0, activePixelCount, xO, xO + gW);
        rect(x, yO, 1.5, gH); 
      }
    }
  }
  
  textAlign(RIGHT); fill(80); textSize(12);
  for (int i=0; i<=10; i++) {
    float val = i*(currentYMax/10);
    float y = map(val, 0, currentYMax, yO+gH, yO);
    stroke(230); line(xO, y, xO+gW, y);
    String label = (mode==0) ? str(int(val)) : (mode==1) ? (int(val)+"%") : nf(val, 1, 2);
    text(label, xO-10, y+5);
  }
  
  textAlign(CENTER);
  float nmMin = getWavelength(0);
  float nmMax = getWavelength(activePixelCount - 1);
  for (int nm = 350; nm <= 900; nm += 50) {
    float x = map(nm, nmMin, nmMax, xO, xO + gW);
    if (x >= xO && x <= xO + gW) {
      stroke(220); line(x, yO, x, yO+gH);
      fill(80); text(nm, x, yO+gH+20);
    }
  }
  
  if (hasOverlay) {
    stroke(180, 180, 180, 150); strokeWeight(1.5); noFill();
    beginShape();
    for (int i=0; i<activePixelCount; i++) {
      float x = map(i, 0, activePixelCount, xO, xO+gW);
      float y = map(overlayBuffer[i], 0, currentYMax, yO+gH, yO);
      if(y >= yO) vertex(x, y);
    }
    endShape();
  }

  stroke(30, 100, 200); strokeWeight(2); noFill();
  beginShape();
  for (int i=0; i<activePixelCount; i++) {
    float x = map(i, 0, activePixelCount, xO, xO+gW);
    float y = map(processedData[i], 0, currentYMax, yO+gH, yO);
    if(y >= yO) vertex(x, y);
  }
  endShape();
  
  textAlign(LEFT); textSize(11);
  fill(hasDark ? color(0, 150, 0) : color(150)); text("Dark: " + (hasDark?"ACTIVE":"None"), xO, yO-25);
  fill(hasRef ? color(0, 150, 0) : color(150));  text("Ref: " + (hasRef?"READY":"None"), xO + 110, yO-25);
  if(hasOverlay) { fill(120); text("Overlay: ON", xO + 220, yO-25); }

  fill(100); textSize(13);
  String msg = statusMsg;
  if (isDarkCalibSequence) msg = "● CAPTURING DARK [" + (samplingCounter+1) + "/" + samplingTarget + "]...";
  else if (isRefCalibSequence) msg = "● CAPTURING REF [" + (samplingCounter+1) + "/" + samplingTarget + "]...";
  else if (intervalActive) msg = "● INTERVAL [" + intervalCurrentCount + "/" + intervalTargetCount + "] NEXT in " + int((intervalStepS*1000 - (millis()-lastIntervalTriggerMillis))/1000) + "s";
  else if (samplingTarget > 1 && (autoMode || isMeasureSequence)) msg = "● AVERAGING [" + (samplingCounter+1) + "/" + samplingTarget + "]...";
  else if (isMeasuring) msg = "● MEASURING...";
  fill(0, 100, 200); text(msg, xO+5, yO-10);
}

void drawControlInterface() {
  int bY = 595;
  fill(80); textAlign(LEFT); textSize(12); text("OVERLAY:", 80, bY - 10);
  drawBtn(80, bY, 120, 30, "SET CURRENT", color(200, 220, 255), false);
  drawBtn(210, bY, 120, 30, "LOAD CSV", color(200, 220, 255), false);
  drawBtn(340, bY, 100, 30, "CLEAR", color(240), !hasOverlay);
  
  drawBtn(600, BY_SAVE, 160, 45, "SAVE CSV", color(180, 220, 180), autoMode || isMeasureSequence);
  drawBtn(775, BY_SAVE, 155, 45, "RESET PORT", color(255, 180, 180), false);

  fill(235); noStroke(); rect(PX-10, 15, 230, 685, 10);
  if (commError) fill(255, 0, 0); else if (isMeasuring) fill(255, 200, 0); else fill(0, 200, 100);
  ellipse(PX+200, 20, 12, 12);
  
  drawBtn(PX, BY_AUTO, 210, 32, "AUTO MODE: "+(autoMode?"ON":"OFF"), autoMode?color(255,150,0):color(200), intervalActive);
  drawBtn(PX, BY_AUTO+37, 210, 32, "AUTO SCALE: "+(autoScale?"ON":"OFF"), autoScale?color(100,200,100):color(200), false);
  
  boolean busy = autoMode || isMeasureSequence || isDarkCalibSequence || isRefCalibSequence || intervalActive;
  drawBtn(PX, BY_MODE, 210, 32, "MODE: "+(mode==0?"EMISS":mode==1?"TRANS":"ABS"), color(100, 150, 250), busy);
  drawBtn(PX, BY_MODE+37, 210, 32, "MEASURE", color(255, 200, 100), busy);
  drawBtn(PX, BY_MODE+74, 210, 32, "DARK CALIB", hasDark?color(150,200,150):color(250,200,200), busy);
  drawBtn(PX, BY_MODE+111, 210, 32, "REF CALIB", hasRef?color(150,200,150):color(250,200,200), busy || mode==0);

  stroke(200); line(PX, BY_INTV-10, PX+210, BY_INTV-10);
  fill(50); textAlign(CENTER); textSize(12); text("INTERVAL MEASURE", PX+105, BY_INTV);
  textSize(11); text("Step: " + intervalStepS + "s / Count: " + intervalTargetCount, PX+105, BY_INTV+17);
  drawBtn(PX, BY_INTV+25, 50, 25, "S-", color(220), busy && !intervalActive);
  drawBtn(PX+53, BY_INTV+25, 50, 25, "S+", color(220), busy && !intervalActive);
  drawBtn(PX+106, BY_INTV+25, 50, 25, "C-", color(220), busy && !intervalActive);
  drawBtn(PX+160, BY_INTV+25, 50, 25, "C+", color(220), busy && !intervalActive);
  drawBtn(PX, BY_INTV+55, 210, 30, intervalActive ? "STOP" : "START INTERVAL", intervalActive?color(255,100,100):color(180,255,180), autoMode || isMeasureSequence);

  stroke(200); line(PX, BY_SAMP-10, PX+210, BY_SAMP-10);
  fill(50); textSize(12); text("SAMPLING (Avg)", PX+105, BY_SAMP);
  for(int i=0; i<5; i++) {
    color c = (samplingTarget == i+1) ? color(100,150,255) : color(220);
    drawBtn(PX + i*42, BY_SAMP+10, 38, 25, str(i+1), c, isMeasureSequence || intervalActive);
  }
  
  stroke(200); line(PX, BY_EXPO-10, PX+210, BY_EXPO-10);
  fill(50); text("EXPOSURE: " + nf(exposureS, 1, 3) + " s", PX+105, BY_EXPO);
  boolean bExp = isMeasureSequence || intervalActive;
  drawBtn(PX,     BY_EXPO+10, 68, 25, "+1.0",  color(200, 240, 200), bExp);
  drawBtn(PX+71,  BY_EXPO+10, 68, 25, "+0.1",  color(210, 245, 210), bExp);
  drawBtn(PX+142, BY_EXPO+10, 68, 25, "+0.01", color(220, 250, 220), bExp);
  drawBtn(PX,     BY_EXPO+38, 68, 25, "-1.0",  color(240, 200, 200), bExp);
  drawBtn(PX+71,  BY_EXPO+38, 68, 25, "-0.1",  color(245, 210, 210), bExp);
  drawBtn(PX+142, BY_EXPO+38, 68, 25, "-0.01", color(250, 220, 220), bExp);
  
  stroke(200); line(PX, BY_SMOT-10, PX+210, BY_SMOT-10);
  fill(50); text("SMOOTHING: " + smoothingSize, PX+105, BY_SMOT);
  drawBtn(PX, BY_SMOT+10, 100, 30, "LESS", color(220), false);
  drawBtn(PX+110, BY_SMOT+10, 100, 30, "MORE", color(220), false);
}

// ==========================================
// 5. 各種関数 & マウス入力
// ==========================================

void loadCalibration() {
  String path = sketchPath(CONFIG_DIR + "/calibration_coefficients.txt");
  String[] lines = loadStrings(path);
  if (lines != null && lines.length >= 7) { 
    sensorSerial = lines[0]; // 1行目がシリアル番号
    A0=float(lines[1]); B1=float(lines[2]); B2=float(lines[3]); B3=float(lines[4]); B4=float(lines[5]); B5=float(lines[6]); 
  } else {
    saveStrings(path, new String[]{sensorSerial, str(A0), str(B1), str(B2), str(B3), str(B4), str(B5)});
  }
}

void checkCommStatus() {
  long timeoutLimit = int(lastSentExposureS * 1000) + 10000;
  commError = (isMeasuring && (millis() - lastReceiveMillis > timeoutLimit));
}

void drawPortSelector() {
  fill(50); textAlign(CENTER); textSize(24); text("SELECT SPECTROMETER PORT", width/2, 100);
  String[] pts = processing.serial.Serial.list();
  for (int i=0; i<pts.length; i++) { fill(255); stroke(100); rect(width/2-150, 150+i*55, 300, 45, 5); fill(50); textSize(16); text(pts[i], width/2, 178+i*55); }
}

void drawBtn(int x, int y, int w, int h, String t, color c, boolean dis) { 
  fill(dis?225:c); stroke(dis?235:150); rect(x, y, w, h, 5); 
  fill(dis?180:0); textAlign(CENTER, CENTER); textSize(12); text(t, x + w/2, y + h/2); 
}

boolean overBtn(int x, int y, int w, int h) { return (mouseX >= x && mouseX <= x+w && mouseY >= y && mouseY <= y+h); }

void resetAccumulator() { samplingCounter = 0; for(int i=0; i<activePixelCount; i++) samplingAccumulator[i] = 0; isMeasuring = false; lastReceiveMillis = millis(); }

void mousePressed() {
  if (!portSelected) {
    String[] pts = processing.serial.Serial.list();
    for (int i=0; i<pts.length; i++) if (overBtn(width/2-150, 150+i*55, 300, 45)) { 
      try { myPort = new processing.serial.Serial(this, pts[i], 230400); portSelected = true; lastReceiveMillis = millis(); myPort.write("exp:" + int(exposureS * 1000) + "\n"); } catch(Exception e) {}
    }
    return;
  }
  
  if (mouseX < 950) {
    if (overBtn(80, 595, 120, 30)) { arrayCopy(processedData, overlayBuffer); hasOverlay = true; }
    if (overBtn(210, 595, 120, 30)) { loadOverlayFromCSV(); }
    if (overBtn(340, 595, 100, 30)) { hasOverlay = false; }
    if (overBtn(600, BY_SAVE, 160, 45)) { if(!autoMode && !isMeasureSequence) saveCSVManual(); }
    if (overBtn(775, BY_SAVE, 155, 45)) { if(myPort != null) myPort.stop(); portSelected = false; commError = false; resetAccumulator(); }
  } else {
    if (overBtn(PX, BY_AUTO, 210, 32)) { autoMode = !autoMode; isMeasureSequence = false; }
    if (overBtn(PX, BY_AUTO+37, 210, 32)) { autoScale = !autoScale; }
    if (isMeasureSequence || isDarkCalibSequence || isRefCalibSequence) return;
    if (overBtn(PX, BY_MODE, 210, 32)) { mode = (mode + 1) % 3; hasOverlay = false; resetAccumulator(); }
    if (overBtn(PX, BY_MODE+37, 210, 32)) { isMeasureSequence = true; resetAccumulator(); }
    if (overBtn(PX, BY_MODE+74, 210, 32)) { isDarkCalibSequence = true; resetAccumulator(); }
    if (overBtn(PX, BY_MODE+111, 210, 32) && mode != 0) { isRefCalibSequence = true; resetAccumulator(); }
    if (!intervalActive) {
      if (overBtn(PX, BY_INTV+25, 50, 25)) { if (intervalStepS > 10) intervalStepS -= 5; else intervalStepS = max(1, intervalStepS - 1); }
      if (overBtn(PX+53, BY_INTV+25, 50, 25)) { if (intervalStepS >= 10) intervalStepS += 5; else intervalStepS += 1; }
      if (overBtn(PX+106, BY_INTV+25, 50, 25)) intervalTargetCount = max(1, intervalTargetCount - 1);
      if (overBtn(PX+160, BY_INTV+25, 50, 25)) intervalTargetCount += 1;
    }
    if (overBtn(PX, BY_INTV+55, 210, 30) && !autoMode) {
      intervalActive = !intervalActive;
      if (intervalActive) {
        String name = JOptionPane.showInputDialog("Project Name"); if (name == null || name.equals("")) { intervalActive = false; } else {
          intervalSessionName = name.replace(",", "_"); intervalSessionFolder = DATA_DIR + "/" + intervalSessionName + "_" + year()+nf(month(),2)+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2);
          intervalCurrentCount = 0; intervalHistory.clear(); lastIntervalTriggerMillis = 0; 
        }
      }
    }
    for(int i=0; i<5; i++) if (overBtn(PX + i*42, BY_SAMP+10, 38, 25)) { samplingTarget = i + 1; resetAccumulator(); }
    float oldExp = exposureS;
    if (overBtn(PX, BY_EXPO+10, 68, 25)) exposureS += 1.0;
    if (overBtn(PX+71, BY_EXPO+10, 68, 25)) exposureS += 0.1;
    if (overBtn(PX+142, BY_EXPO+10, 68, 25)) exposureS += 0.01;
    if (overBtn(PX, BY_EXPO+38, 68, 25)) exposureS -= 1.0;
    if (overBtn(PX+71, BY_EXPO+38, 68, 25)) exposureS -= 0.1;
    if (overBtn(PX+142, BY_EXPO+38, 68, 25)) exposureS -= 0.01;
    if (exposureS != oldExp) { exposureS = constrain(exposureS, 0.02, 60.0); exposureChanged = true; if(myPort != null) myPort.clear(); resetAccumulator(); lastReceiveMillis = millis(); }
    if (overBtn(PX, BY_SMOT+10, 100, 30)) smoothingSize = max(0, smoothingSize-1);
    if (overBtn(PX+110, BY_SMOT+10, 100, 30)) smoothingSize = min(20, smoothingSize+1);
  }
}

// ==========================================
// 6. CSV保存 & ユーティリティ
// ==========================================

void loadOverlayFromCSV() { selectInput("Select CSV for Overlay:", "overlayFileSelected"); }

void overlayFileSelected(File selection) {
  if (selection == null) return;
  String[] lines = loadStrings(selection.getAbsolutePath());
  if (lines == null) return;
  int dataCount = 0;
  for (String line : lines) {
    if (line.startsWith("#") || line.startsWith("wavelength")) continue;
    String[] parts = split(line, ",");
    if (parts.length >= 2 && dataCount < activePixelCount) { overlayBuffer[dataCount] = float(parts[1]); dataCount++; }
  }
  if (dataCount > 0) { hasOverlay = true; statusMsg = "Overlay Loaded."; }
}

void saveCSVManual() {
  String name = JOptionPane.showInputDialog("Name/Memo"); if (name == null) return;
  saveCSV(DATA_DIR + "/spec_" + name.replace(",", "_") + "_" + year()+nf(month(),2)+nf(day(),2), name);
}

void saveCSV(String fullPathNoExt, String memo) {
  PrintWriter out = createWriter(fullPathNoExt + ".csv");
  String d = year()+"/"+nf(month(),2)+"/"+nf(day(),2)+" "+nf(hour(),2)+":"+nf(minute(),2)+":"+nf(second(),2);
  // 【追加】ヘッダにシリアル番号を記載
  out.println("# Serial: " + sensorSerial + ", Coefficients: A0=" + A0 + ", B1=" + B1 + ", B2=" + B2 + ", B3=" + B3 + ", B4=" + B4 + ", B5=" + B5);
  out.println("# Condition: Date=" + d + ", Mode="+(mode==0?"Emiss":mode==1?"Trans":"Abs")+", Exp="+exposureS+"s, Avg="+samplingTarget+", Smooth="+smoothingSize+", Memo="+memo);
  out.println("wavelength_nm,final_val,raw_count,dark_count,ref_count");
  for (int i = 0; i < activePixelCount; i++) {
    float nm = getWavelength(i);
    String fv = (mode==2)?nf(processedData[i],1,4):nf(processedData[i],1,2);
    out.println(nm + "," + fv + "," + activeData[i] + "," + (hasDark ? darkBuffer[i] : 0) + "," + (hasRef ? refBuffer[i] : 0));
  }
  out.flush(); out.close();
}

void finalizeInterval() {
  intervalActive = false;
  String summaryPath = intervalSessionFolder + "/_SUMMARY_merged.csv";
  PrintWriter out = createWriter(summaryPath);
  out.print("wavelength_nm");
  for (int j=1; j<=intervalHistory.size(); j++) out.print(",Measure_" + nf(j,3));
  out.println();
  for (int i=0; i<activePixelCount; i++) {
    out.print(getWavelength(i));
    for (int j=0; j<intervalHistory.size(); j++) out.print("," + nf(intervalHistory.get(j)[i], 1, 4));
    out.println();
  }
  out.flush(); out.close();
  statusMsg = "Interval Complete.";
  JOptionPane.showMessageDialog(null, "Interval Complete. Merged file saved in folder.");
}
